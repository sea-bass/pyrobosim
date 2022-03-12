import warnings
import numpy as np
from shapely.affinity import rotate, translate
from shapely.geometry import Point, Polygon, CAP_STYLE, JOIN_STYLE

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

def polygon_from_footprint(footprint, pose=None, parent_polygon=None):
    """
    Creates a Shapely polygon given footprint metadata
    Valid footprint metadata include:
        - type: Type of footprint. Supported geometries include.
            - box
                - dims: (x, y) dimensions
            - circle
                - radius: radius of circle
            - polygon
                - coords: List of (x, y) coordinates 
            - parent: Requires `parent_polygon` to also be passed in
                - padding: Additional padding relative to the parent polygon
        - offset: Offset (x, y) or (x, y, yaw) from the specified geometry above
    """
    # Parse through the footprint type and corresponding properties
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
        else:
            warnings.warn(f"Invalid footprint type: {ftype}")
            return None

    # Offset the polygon, if specified
    if "offset" in footprint:
        offset_vec = footprint["offset"]
        if len(offset_vec) == 2:
            offset_pose = Pose(x=offset_vec[0], y=offset_vec[1])
        elif len(offset_vec) == 3:
            offset_pose = Pose(x=offset_vec[0], y=offset_vec[1], yaw=offset_vec[2])
        polygon = transform_polygon(polygon, offset_pose)

    if pose is not None and ftype != "parent":
        polygon = transform_polygon(polygon, pose)
    return polygon

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
