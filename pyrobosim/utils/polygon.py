import os
import collada
import trimesh
import warnings
import numpy as np
from scipy.spatial import ConvexHull
from shapely.affinity import rotate, translate
from shapely.geometry import Point, Polygon, CAP_STYLE, JOIN_STYLE

from .general import replace_special_yaml_tokens
from .pose import Pose, rot2d

"""
Polygon utilities
"""


def add_coords(coords, offset):
    """
    Adds an offset (x,y) vector to a Shapely compatible list 
    of coordinate tuples

    Args:
        coords: A list of coordinate tuples representing the polygon
        offset: An (x,y) offset vector list or tuple
    """
    x, y = offset
    return [(c[0]+x, c[1]+y) for c in coords]


def box_to_coords(dims=[1, 1], origin=[0, 0], ang=0):
    """ 
    Converts box dimensions and origin to a Shapely compatible 
    list of coordinate tuples.

    Args:
        dims (list): The box dimensions [width, height]
        origin (list): The box origin [x, y]
        ang (float): The angle (in radians) to rotate the box

    Returns:
        coords: A list of coordinate tuples representing the box

    Examples:
        >>> coords = box_to_coords(dims=[2.5,2.5], origin=[1,2])
    """
    x, y = origin
    w, h = dims
    coords = [
        rot2d((-0.5*w, -0.5*h), ang),
        rot2d((0.5*w, -0.5*h), ang),
        rot2d((0.5*w,  0.5*h), ang),
        rot2d((-0.5*w,  0.5*h), ang),
    ]
    coords.append(coords[0])
    coords = add_coords(coords, (x, y))
    return coords


def get_polygon_centroid(poly):
    """ Gets a Shapely polygon centroid as a list """
    return list(poly.centroid.coords)[0]


def inflate_polygon(poly, radius):
    """ 
    Inflates a Shapely polygon with options preconfigured for 
    this world modeling framework.
    """
    return poly.buffer(radius,
                       cap_style=CAP_STYLE.flat,
                       join_style=JOIN_STYLE.mitre)


def transform_polygon(polygon, pose):
    """ 
    Transforms a Shapely polygon by a Pose object.
    The order of operations is first translation, and then rotation 
    about the new translated position.
    """
    polygon = translate(polygon,
                        xoff=pose.x, yoff=pose.y)
    polygon = rotate(
        polygon, pose.yaw, origin=(pose.x, pose.y), use_radians=True)

    return polygon


def polygon_and_height_from_footprint(footprint, pose=None, parent_polygon=None):
    """
    Returns a Shapely polygon and vertical (Z) height given footprint metadata.
    Valid footprint metadata include:
        - type: Type of footprint. Supported geometries include.
            - box
                - dims: (x, y) dimensions
            - circle
                - radius: radius of circle
            - polygon
                - coords: List of (x, y) coordinates
            - mesh
                - model_path: Path to folder containing the .sdf and mesh files
                - mesh path: Path to mesh file relative to model_path
            - parent: Requires `parent_polygon` to also be passed in
                - padding: Additional padding relative to the parent polygon
        - offset: Offset (x, y) or (x, y, yaw) from the specified geometry above
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
        polygon = transform_polygon(
            polygon, Pose.from_list(footprint["offset"]))

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
    TODO: Right now this supports only DAE file, which seem to be the most common for Gazebo.
    """
    mesh_filename = replace_special_yaml_tokens(
        os.path.join(mesh_data["model_path"], mesh_data["mesh_path"]))
    mesh = trimesh.load_mesh(mesh_filename, "dae")

    # Get the unit scale
    c = collada.Collada(mesh_filename)
    scale = c.assetInfo.unitmeter

    # Get the convex hull of the 2D points
    footprint_pts = [[p[0]*scale, p[1]*scale]
                     for p in mesh.convex_hull.vertices]
    hull = ConvexHull(footprint_pts)
    hull_pts = hull.points[hull.vertices, :]

    # Get the height as the max of the 3D points
    height = max([p[2] for p in mesh.convex_hull.vertices]) * scale

    return (Polygon(hull_pts), height)


def sample_from_polygon(polygon, max_tries=100):
    """ Samples a valid (x, y) tuple from a Shapely polygon """
    xmin, ymin, xmax, ymax = polygon.bounds
    for _ in range(max_tries):
        sample_x = np.random.uniform(xmin, xmax)
        sample_y = np.random.uniform(ymin, ymax)
        if polygon.contains(Point(sample_x, sample_y)):
            return sample_x, sample_y

    warnings.warn(f"Exceeded max polygon samples samples: {max_tries}")
    return None, None
