import time
import threading
import numpy as np
import warnings

from .locations import ObjectSpawn
from .objects import Object
from ..utils.knowledge import resolve_to_object
from ..utils.polygon import inflate_polygon, sample_from_polygon, transform_polygon
from ..utils.pose import Pose

class Robot:
    height = 0.0

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

    def follow_path(self, path, realtime_factor=1.0, 
                    use_thread=True, blocking=False):
        """ Follows a specified base path """
        if path is None or self.path_executor is None:
            return

        if use_thread:
            # Start a thread with the path execution
            self.nav_thread = threading.Thread(target=self.path_executor.execute,
                                               args=(path, realtime_factor))
            self.nav_thread.start()
            if blocking:
                success = self.nav_thread.join()
            else:
                success = True
        else:
            success = self.path_executor.execute(path, realtime_factor)
        return success

    def pick_object(self, obj_query):
        """ 
        Picks up an object in the world given an object and location query. 
        Returns True if successful and False otherwise.
        """
        # Validate input
        if self.manipulated_object is not None:
            warnings.warn(f"Robot is already holding {self.robot.manipulated_object.name}.")
            return False

        # Get object
        loc = self.location
        if isinstance(obj_query, Object):
            obj = obj_query
        else:
            obj = self.world.get_object_by_name(obj_query)
            if not obj:
                obj = resolve_to_object(
                    self.world, category=obj_query, location=loc,
                    resolution_strategy="nearest")
            if not obj:
                warnings.warn(f"Found no object {obj_query} to pick.")
                return False

        # Validate the robot location
        if obj.parent != loc:
            warnings.warn(f"{obj.name} is at {obj.parent.name} and robot" +
                          f"is at {loc.name}. Cannot pick.")
            return False
    
        # Denote the target object as the manipulated object
        self.manipulated_object = obj
        obj.parent.children.remove(obj)
        obj.parent = self
        obj.pose = self.pose
        return True

    def place_object(self, loc, pose=None):
        """
        Places an object in a target location and (optionally) pose.
        Returns True if successful and False otherwise.
        """
        # Validate input
        if self.manipulated_object is None:
            warnings.warn("No manipulated object.")
            return False

        # Validate the robot location
        loc = self.location
        if not isinstance(loc, ObjectSpawn):
            warnings.warn("Not an object spawn. Cannot place object.")
            return False

        # Place the object somewhere in the current location
        poly = inflate_polygon(self.manipulated_object.get_raw_polygon(),
                               self.world.object_radius)
        if pose is None:
            # If no pose was specified, sample one
            is_valid_pose = False
            for _ in range(self.world.max_object_sample_tries):
                x_sample, y_sample = sample_from_polygon(loc.polygon)
                yaw_sample = np.random.uniform(-np.pi, np.pi)
                pose_sample = Pose(x=x_sample, y=y_sample, yaw=yaw_sample)
                sample_poly = transform_polygon(poly, pose_sample)
                is_valid_pose = sample_poly.within(loc.polygon)
                for other_obj in loc.children:
                    is_valid_pose = is_valid_pose and not sample_poly.intersects(other_obj.polygon)
                if is_valid_pose:
                    self.manipulated_object.parent = loc
                    self.manipulated_object.set_pose(pose_sample)
                    self.manipulated_object.create_polygons()
                    loc.children.append(self.manipulated_object)
                    self.manipulated_object = None
                    return True
            warnings.warn(f"Could not sample a placement position at {loc.name}")
            return False
        else:
            # If a pose was specified, collision check it
            poly = transform_polygon(poly, pose_sample)
            is_valid_pose = poly.within(loc.polygon)
            for other_obj in loc.children:
                is_valid_pose = is_valid_pose and not poly.intersects(other_obj.polygon)
            if is_valid_pose:
                return True
            else:
                warnings.warn(f"Pose in collision or not in location {loc.name}.")
                return False

    def execute_action(self, action, blocking=False):
        """ Executes an action, specified as a TaskAction object """
        robot = self.world.robot
        self.executing_action = True
        self.current_action = action
        if self.world.has_gui:
            self.world.gui.set_buttons_during_action(False)

        if action.type == "navigate":
            if self.world.has_gui:
                self.executing_nav = True
                self.world.gui.wg.nav_trigger.emit(action.target_location)
                while self.executing_nav:
                    time.sleep(0.5) # Delay to wait for navigation
                success = True # TODO Need to keep track of nav status
            else:
                path = self.world.find_path(action.target_location)
                success = robot.follow_path(path, realtime_factor=1.0, blocking=blocking)
        elif action.type == "pick":
            if self.world.has_gui:
                success = self.world.gui.wg.pick_object(action.object)
            else:
                success = self.pick_object(action.object)
        elif action.type == "place":
            if self.world.has_gui:
                success = self.world.gui.wg.place_object(None)
            else:
                success = self.place_object(None)
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
            success = self.execute_action(act_msg, blocking=blocking)
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
