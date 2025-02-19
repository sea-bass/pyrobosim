"""
Polygon representation and maniupulation utilities.

These tools rely heavily on the Shapely package.
"""

import os
import collada
import numpy as np
import trimesh
from typing import Any, Sequence

from scipy.spatial import ConvexHull
from shapely.affinity import rotate, translate
from shapely.geometry import Point, Polygon, CAP_STYLE, JOIN_STYLE
from shapely.geometry.polygon import orient

from .general import replace_special_yaml_tokens
from .pose import Pose, rot2d
from ..utils.logging import get_global_logger


def add_coords(
    coords: Sequence[Sequence[float]], offset: tuple[float, float]
) -> list[Sequence[float]]:
    """
    Adds an offset (x,y) vector to a Shapely compatible list
    of coordinate tuples.

    :param coords: A list of 2D coordinates representing the polygon.
    :param offset: The (x,y) offset vector.
    :return: The offset list of 2D coordinates representing the new polygon.
    """
    x, y = offset
    return [(c[0] + x, c[1] + y) for c in coords]


def box_to_coords(
    dims: Sequence[float],
    origin: Sequence[float] = (0.0, 0.0),
    ang: float = 0.0,
) -> list[Sequence[float]]:
    """
    Converts box dimensions and origin to a Shapely compatible
    list of coordinate tuples.

    :param dims: The box dimensions (width, height).
    :param origin: The box (x,y) origin
    :param ang: The angle to rotate the box, in radians.
    :return: A list of 2D coordinate representing the polygon.

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


def get_polygon_centroid(poly: Polygon) -> list[float]:
    """
    Gets a Shapely polygon centroid as a list.

    :param poly: Shapely polygon.
    :return: The centroid (x, y) coordinates.
    """
    centroid_coords = list(poly.centroid.coords)
    if len(centroid_coords) == 0:
        return [0.0, 0.0]

    return centroid_coords[0]  # type: ignore[no-any-return]


def inflate_polygon(poly: Polygon, radius: float) -> Polygon:
    """
    Inflates a Shapely polygon with options preconfigured for
    this world modeling framework.

    :param poly: Shapely polygon.
    :param radius: Inflation radius, in meters.
    :return: The inflated Shapely polygon.
    """
    inflated_poly = poly.buffer(
        radius, cap_style=CAP_STYLE.flat, join_style=JOIN_STYLE.mitre
    )
    if inflated_poly.is_empty:
        # NOTE: I fixed this case in Shapely, but it's not yet released.
        # See https://github.com/shapely/shapely/pull/2214
        # Keeping this around until a release containing this fix is out.
        return inflated_poly

    return orient(inflated_poly)


def transform_polygon(polygon: Polygon, pose: Pose) -> Polygon:
    """
    Transforms a Shapely polygon by a Pose object.
    The order of operations is first translation, and then rotation
    about the new translated position.

    :param poly: Shapely polygon.
    :param pose: Pose to transform the polygon.
    :return: The transformed Shapely polygon.
    """
    if pose is not None:
        polygon = translate(polygon, xoff=pose.x, yoff=pose.y)
        polygon = rotate(
            polygon, pose.get_yaw(), origin=(pose.x, pose.y), use_radians=True
        )
    return polygon


def polygon_and_height_from_footprint(
    footprint: dict[str, Any],
    pose: Pose | None = None,
    parent_polygon: Polygon | None = None,
) -> tuple[Polygon, float | None]:
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
    :param pose: Pose with which to transform the resulting polygon
    :param parent_polygon: Shapely polygon representing the parent geometry, if applicable
    :return: Shapely polygon representing the loaded polygon, plus the vertical (Z) height (which could be None if unset).
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
            raise ValueError(f"Invalid footprint type: {ftype}")

    # Offset the polygon, if specified
    if "offset" in footprint:
        polygon = transform_polygon(polygon, Pose.from_list(footprint["offset"]))

    if pose is not None and ftype != "parent":
        polygon = transform_polygon(polygon, pose)

    # Get the height from the footprint, if one was specified.
    # This will override the height calculated from the mesh.
    if "height" in footprint:
        height = footprint["height"]

    assert isinstance(polygon, Polygon)
    return (polygon, height)


def polygon_and_height_from_mesh(mesh_data: dict[str, str]) -> tuple[Polygon, float]:
    """
    Returns the 2D footprint and the max height from a mesh
    NOTE: Right now this supports only DAE files, which is a
    commonly used format for Gazebo models.

    :param mesh_data: Mesh geometry metadata from YAML file
    :return: Shapely polygon representing the 2D convex hull of the mesh, plus the vertical (Z) height.
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

    return (Polygon(hull_pts), float(height))


def sample_from_polygon(
    polygon: Polygon, max_tries: int = 100
) -> tuple[float | None, float | None]:
    """
    Samples a valid (x, y) tuple that is inside a Shapely polygon.
    This is done using rejection sampling, in which we sample from the
    x-y bounds of the polygon and check whether the point is inside the
    (potentially more complex) polygon geometry.

    :param polygon: Shapely polygon from which to sample
    :param max_tries: Maximum tries for sampling.
    :return: Sampled pose contained within the polygon. If no pose could be found, returns (None, None)
    """
    xmin, ymin, xmax, ymax = polygon.bounds
    for _ in range(max_tries):
        sample_x = np.random.uniform(xmin, xmax)
        sample_y = np.random.uniform(ymin, ymax)
        if polygon.contains(Point(sample_x, sample_y)):
            return sample_x, sample_y

    get_global_logger().warning(f"Exceeded max polygon samples: {max_tries}")
    return None, None


def convhull_to_rectangle(points: np.ndarray) -> tuple[Pose, list[float], np.ndarray]:
    """
    Find the smallest bounding rectangle for a set of points.
    Returns a set of points representing the corners of the bounding box.

    :param points: an Nx2 matrix of convex hull XY coordinates
    :return: A tuple of rectangle origin pose, XY dimensions, and the rectangle points.
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
