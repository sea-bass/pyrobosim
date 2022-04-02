import time
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
        self.executing_nav = False

        # World interaction properties
        self.world = None
        self.current_action = None
        self.executing_action = False
        self.current_plan = None
        self.executing_plan = False
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

    def execute_action(self, action, blocking=False):
        """ Executes an action, specified as a TaskAction object """
        self.executing_action = True
        self.current_action = action
        if self.world.has_gui:
            self.world.gui.set_buttons_during_action(False)

        # TODO: Make actions callable from the robot, not the world
        if action.type == "navigate":
            if self.world.has_gui:
                success = self.world.gui.wg.navigate(action.target_location)
            else:
                path = self.world.find_path(action.target_location)
                success = self.world.execute_path(path, realtime_factor=1.0, blocking=blocking)
        elif action.type == "pick":
            if self.world.has_gui:
                success = self.world.gui.wg.pick_object(action.object)
            else:
                success = self.world.pick_object(action.object)
        elif action.type == "place":
            if self.world.has_gui:
                success = self.world.gui.wg.place_object(None)
            else:
                success = self.world.place_object(None)
        else:
            print(f"Invalid action type: {action.type}")
            success = False

        if self.world.has_gui:
            self.world.gui.set_buttons_during_action(True)
        print(f"Action completed with success: {success}")
        self.current_action = None
        self.executing_action = False
        return success

    def execute_plan(self, plan, blocking=False, delay=0.5):
        """ Executes a plan, specified as a TaskPlan object """
        self.executing_plan = True
        self.current_plan = plan
        print(f"Executing task plan...")
        if self.world.has_gui:
            self.world.gui.set_buttons_during_action(False)

        success = True
        num_acts = len(plan.actions)
        for n, act_msg in enumerate(plan.actions):
            print(f"Executing action {act_msg.type} [{n+1}/{num_acts}]")
            success = self.execute_action(act_msg)
            if not success:
                print(f"Task plan failed to execute on action {n+1}")
                break
            time.sleep(delay) # Artificial delay between actions

        if self.world.has_gui:
            self.world.gui.set_buttons_during_action(True)
        print(f"Task plan completed with success: {success}")
        self.current_plan = None
        self.executing_plan = False
        return success

    def __repr__(self):
        return f"Robot {self.name}, ID={self.id}\n\t{self.pose}"
