"""
Polygon representation and maniupulation utilities.

These tools rely heavily on the Shapely package.
"""

import os
import collada
import numpy as np
import trimesh
import warnings

from scipy.spatial import ConvexHull
from shapely.affinity import rotate, translate
from shapely.geometry import Point, Polygon, CAP_STYLE, JOIN_STYLE
from shapely.geometry.polygon import orient

from .general import replace_special_yaml_tokens
from .pose import Pose, rot2d


def add_coords(coords, offset):
    """
    Adds an offset (x,y) vector to a Shapely compatible list
    of coordinate tuples.

    :param coords: A list of 2D coordinates representing the polygon.
    :type coords: list[(float, float)]
    :param offset: The (x,y) offset vector.
    :type offset: (float, float)
    :return: The offset list of 2D coordinates representing the new polygon.
    :rtype: list[(float, float)]
    """
    x, y = offset
    return [(c[0] + x, c[1] + y) for c in coords]


def box_to_coords(dims, origin=[0, 0], ang=0):
    """
    Converts box dimensions and origin to a Shapely compatible
    list of coordinate tuples.

    :param dims: The box dimensions (width, height).
    :type dims: (float, float)
    :param origin: The box (x,y) origin
    :type origin: (float, float), optional
    :param ang: The angle to rotate the box, in radians.
    :type ang: float, optional
    :return: A list of 2D coordinate representing the polygon.
    :rtype: list[(float, float)]

    Example:

    .. code-block:: python

       coords = box_to_coords(dims=[2.5, 2.5], origin=[1, 2], ang=0.5)
    """
    x, y = origin
    w, h = dims
    coords = [
        rot2d((-0.5 * w, -0.5 * h), ang),
        rot2d((0.5 * w, -0.5 * h), ang),
        rot2d((0.5 * w, 0.5 * h), ang),
        rot2d((-0.5 * w, 0.5 * h), ang),
    ]
    coords.append(coords[0])
    coords = add_coords(coords, (x, y))
    return coords


def get_polygon_centroid(poly):
    """
    Gets a Shapely polygon centroid as a list.

    :param poly: Shapely polygon.
    :type poly: :class:`shapely.geometry.Polygon`
    :return: The centroid (x, y) coordinates.
    :rtype: [float, float]
    """
    return list(poly.centroid.coords)[0]


def inflate_polygon(poly, radius):
    """
    Inflates a Shapely polygon with options preconfigured for
    this world modeling framework.

    :param poly: Shapely polygon.
    :type poly: :class:`shapely.geometry.Polygon`
    :param radius: Inflation radius, in meters.
    :type radius: float
    :return: The inflated Shapely polygon.
    :rtype: :class:`shapely.geometry.Polygon`
    """
    inflated_poly = poly.buffer(
        radius, cap_style=CAP_STYLE.flat, join_style=JOIN_STYLE.mitre
    )
    return orient(inflated_poly)


def transform_polygon(polygon, pose):
    """
    Transforms a Shapely polygon by a Pose object.
    The order of operations is first translation, and then rotation
    about the new translated position.

    :param poly: Shapely polygon.
    :type poly: :class:`shapely.geometry.Polygon`
    :param pose: Pose to transform the polygon.
    :type pose: :class:`pyrobosim.utils.pose.Pose`
    :return: The transformed Shapely polygon.
    :rtype: :class:`shapely.geometry.Polygon`
    """
    if pose is not None:
        polygon = translate(polygon, xoff=pose.x, yoff=pose.y)
        polygon = rotate(
            polygon, pose.get_yaw(), origin=(pose.x, pose.y), use_radians=True
        )
    return polygon


def polygon_and_height_from_footprint(footprint, pose=None, parent_polygon=None):
    """
    Returns a Shapely polygon and vertical (Z) height given footprint metadata.
    Valid footprint metadata comes from YAML files, and can include:

    * ``"type"``: Type of footprint. Supported geometries include:
        * ``"box"``: Box geometry
            * ``"dims"``: (x, y) dimensions
        * ``"circle"``: Circle geometry
            * ``"radius"``: radius of circle
        * ``"polygon"``: Generic polygon geometry
            * ``"coords"``: List of (x, y) coordinates
        * ``"mesh"``: Load geometry as 2D convex hull from mesh file
            * ``"model_path"``: Path to folder containing the .sdf and mesh files
            * ``"mesh path"``: Path to mesh file relative to model_path
        * ``"parent"``: Requires ``parent_polygon`` argument to also be passed in
            * ``"padding"``: Additional padding relative to the parent polygon
    * ``"offset"``: Offset (x, y) or (x, y, yaw) from the specified geometry above

    :param footprint: Footprint metadata from YAML file
    :type footprint: dict
    :param pose: Pose with which to transform the resulting polygon
    :type pose: :class:`pyrobosim.utils.pose.Pose`
    :param parent_polygon: Shapely polygon representing the parent geometry, if applicable
    :type parent_polygon: :class:`shapely.geometry.Polygon`
    :return: Shapely polygon representing the loaded polygon, plus the vertical (Z) height.
    :rtype: (class:`shapely.geometry.Polygon`, float)
    """
    # Parse through the footprint type and corresponding properties
    height = None
    ftype = footprint["type"]
    if ftype == "parent":
        polygon = parent_polygon
        if "padding" in footprint:
            polygon = inflate_polygon(polygon, -footprint["padding"])
    else:
        if ftype == "box":
            polygon = Polygon(box_to_coords(footprint["dims"]))
        elif ftype == "circle":
            polygon = Point(0, 0).buffer(footprint["radius"])
        elif ftype == "polygon":
            polygon = Polygon(footprint["coords"])
        elif ftype == "mesh":
            polygon, height = polygon_and_height_from_mesh(footprint)
        else:
            warnings.warn(f"Invalid footprint type: {ftype}")
            return None

    # Offset the polygon, if specified
    if "offset" in footprint:
        polygon = transform_polygon(polygon, Pose.from_list(footprint["offset"]))

    if pose is not None and ftype != "parent":
        polygon = transform_polygon(polygon, pose)

    # Get the height from the footprint, if one was specified.
    # This will override the height calculated from the mesh.
    if "height" in footprint:
        height = footprint["height"]
    return (polygon, height)


def polygon_and_height_from_mesh(mesh_data):
    """
    Returns the 2D footprint and the max height from a mesh
    NOTE: Right now this supports only DAE files, which is a
    commonly used format for Gazebo models.

    :param mesh_data: Mesh geometry metadata from YAML file
    :type footprint: dict
    :return: Shapely polygon representing the 2D convex hull of the mesh, plus the vertical (Z) height.
    :rtype: (class:`shapely.geometry.Polygon`, float)
    """
    mesh_filename = replace_special_yaml_tokens(
        os.path.join(mesh_data["model_path"], mesh_data["mesh_path"])
    )
    mesh = trimesh.load_mesh(mesh_filename, "dae")

    # Get the unit scale.
    c = collada.Collada(mesh_filename)
    scale = c.assetInfo.unitmeter

    # Get the convex hull of the 2D points.
    footprint_pts = [[p[0] * scale, p[1] * scale] for p in mesh.convex_hull.vertices]
    hull = ConvexHull(footprint_pts)
    hull_pts = hull.points[hull.vertices, :]

    # Get the height as the max of the 3D points.
    height = max([p[2] for p in mesh.convex_hull.vertices]) * scale

    return (Polygon(hull_pts), height)


def sample_from_polygon(polygon, max_tries=100):
    """
    Samples a valid (x, y) tuple that is inside a Shapely polygon.
    This is done using rejection sampling, in which we sample from the
    x-y bounds of the polygon and check whether the point is inside the
    (potentially more complex) polygon geometry.

    :param polygon: Shapely polygon from which to sample
    :type polygon: :class:`shapely.geometry.Polygon`
    :param max_tries: Maximum tries for sampling.
    :type max_tries: float
    :return: Sampled pose contained within the polygon. If no pose could be found, returns (None, None)
    :rtype: (float, float)
    """
    xmin, ymin, xmax, ymax = polygon.bounds
    for _ in range(max_tries):
        sample_x = np.random.uniform(xmin, xmax)
        sample_y = np.random.uniform(ymin, ymax)
        if polygon.contains(Point(sample_x, sample_y)):
            return sample_x, sample_y

    warnings.warn(f"Exceeded max polygon samples: {max_tries}")
    return None, None


def convhull_to_rectangle(points):
    """
    Find the smallest bounding rectangle for a set of points.
    Returns a set of points representing the corners of the bounding box.

    :param points: an Nx2 matrix of convex hull XY coordinates
    :type points: :class:`numpy.ndarray`
    :return: A tuple of rectangle origin pose and XY dimensions
    :rtype: (:class:`pyrobosim.utils.pose.Pose`, list[float])
    """

    # calculate edge angles
    edges = np.zeros((len(points) - 1, 2))
    edges = points[1:] - points[:-1]
    angles = np.zeros((len(edges)))
    angles = np.arctan2(edges[:, 1], edges[:, 0])
    angles = np.abs(np.mod(angles, np.pi / 2))
    angles = np.unique(angles)

    # Calculate rotation matrices
    rotations = np.vstack(
        [
            np.cos(angles),
            np.cos(angles - np.pi / 2),
            np.cos(angles + np.pi / 2),
            np.cos(angles),
        ]
    ).T
    rotations = rotations.reshape((-1, 2, 2))

    # Rotate the convex hull points
    rot_points = np.dot(rotations, points[:, 0:2].T)

    # Find the bounding points
    min_x = np.nanmin(rot_points[:, 0], axis=1)
    max_x = np.nanmax(rot_points[:, 0], axis=1)
    min_y = np.nanmin(rot_points[:, 1], axis=1)
    max_y = np.nanmax(rot_points[:, 1], axis=1)

    # Find the box with the lowest area
    areas = (max_x - min_x) * (max_y - min_y)
    best_idx = np.argmin(areas)

    # Return the minimum area box
    x1 = max_x[best_idx]
    x2 = min_x[best_idx]
    y1 = max_y[best_idx]
    y2 = min_y[best_idx]
    yaw = angles[best_idx]
    r = rotations[best_idx]

    # Compute the rectangle points
    rect_pts = np.zeros((5, 2))
    rect_pts[0] = np.dot([x1, y2], r)
    rect_pts[1] = np.dot([x2, y2], r)
    rect_pts[2] = np.dot([x2, y1], r)
    rect_pts[3] = np.dot([x1, y1], r)
    rect_pts[4] = rect_pts[0]

    # Compute the origin pose and dimensions
    orig = np.dot([0.5 * (x1 + x2), 0.5 * (y1 + y2)], r)
    pose = Pose(x=orig[0], y=orig[1], yaw=yaw)

    rect_pts_rot = [rot2d(pt, -yaw) for pt in rect_pts[:-1, :]]
    x_min_rot = min([pt[0] for pt in rect_pts_rot])
    x_max_rot = max([pt[0] for pt in rect_pts_rot])
    y_min_rot = min([pt[1] for pt in rect_pts_rot])
    y_max_rot = max([pt[1] for pt in rect_pts_rot])
    dims = [x_max_rot - x_min_rot, y_max_rot - y_min_rot]

    return (pose, dims, rect_pts)
