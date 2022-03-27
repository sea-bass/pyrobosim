import numpy as np

from ..utils.pose import Pose

class Robot:
    def __init__(self, id=0, name="robot", pose=Pose(), radius=0.0, path_executor=None):
        # Basic properties
        self.id = id
        self.name = name
        self.set_pose(pose)
        self.radius = radius

        # Navigation properties
        self.set_path_executor(path_executor)

        # World interaction properties
        self.current_action = None
        self.executing_action = False
        self.location = None
        self.manipulated_object = None

    def set_pose(self, pose):
        self.pose = pose

    def apply_vel(self, dt, v=0.0, w=0.0):
        dtv = dt * v
        self.pose.x += dtv * np.cos(self.yaw)
        self.pose.y += dtv * np.sin(self.yaw)
        self.pose.yaw += dt * w
        self.pose.wrap_yaw()

    def set_path_executor(self, executor):
        """ Sets a path executor for navigation """
        if executor is None:
            return
        executor.robot = self
        self.path_executor = executor

    def __repr__(self):
        return f"Robot {self.name}, ID={self.id}\n\t{self.pose}"
