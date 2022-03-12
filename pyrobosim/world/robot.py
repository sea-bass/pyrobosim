import numpy as np

from ..utils.pose import Pose

class Robot:
    def __init__(self, id=0, name="robot", pose=Pose(), radius=0.2):
        self.id = id
        self.name = name
        self.set_pose(pose)
        self.radius = radius

    def set_pose(self, pose):
        self.pose = pose

    def apply_vel(self, dt, v=0.0, w=0.0):
        dtv = dt * v
        self.pose.x += dtv * np.cos(self.yaw)
        self.pose.y += dtv * np.sin(self.yaw)
        self.pose.yaw += dt * w
        self.pose.wrap_yaw()

    def __repr__(self):
        return f"Robot {self.name}, ID={self.id}\n\t{self.pose}"
