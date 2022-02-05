import math
from shapely.geometry import Point, CAP_STYLE, JOIN_STYLE

class Pose:
    """ Represents a 2.5D pose """
    def __init__(self, x=0.0, y=0.0, z=0.0, yaw=0.0):
        self.x = x
        self.y = y
        self.z = z
        self.yaw = yaw

    def wrap_yaw(self):
        if (self.yaw > math.pi):
            self.yaw -= 2 * math.pi
        elif (self.yaw < -math.pi):
            self.yaw += 2 * math.pi

    def __repr__(self):
        return f"Pose: [x={self.x:.2f}, y={self.y:.2f} z={self.z:.2f} yaw={self.yaw:.2f}]"


def inflate_polygon(poly, radius):
    """ 
    Inflates a Shapely polygon with options preconfigured for 
    our world modeling framework 
    """
    return poly.buffer(radius,
        cap_style=CAP_STYLE.flat,
        join_style=JOIN_STYLE.mitre)
