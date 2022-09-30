""" Defines a robot which operates in a world. """

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
    """Representation of a robot in the world."""

    def __init__(
        self,
        name="robot",
        pose=Pose(),
        radius=0.0,
        height=0.0,
        color=(0.8, 0, 0.8),
        path_planner=None,
        path_executor=None,
    ):
        """
        Creates a robot instance.

        :param name: Robot name.
        :type name: str, optional
        :param pose: Robot initial pose.
        :type pose: :class:`pyrobosim.utils.pose.Pose`
        :param radius: Robot radius, in meters.
        :type radius: float, optional
        :param height: Robot height, in meters.
        :type height: float, optional
        :param color: Robot color, as an RGB tuple or string.
        :type color: tuple[float] / str, optional
        :param path_planner: Path planner for navigation
            (see e.g., :class:`pyrobosim.navigation.rrt.RRTPlanner`).
        :type path_planner: PathPlanner, optional
        :param path_executor: Path executor for navigation (see e.g.,
            :class:`pyrobosim.navigation.execution.ConstantVelocityExecutor`).
        :type path_executor: PathExecutor, optional
        """
        # Basic properties
        self.name = name
        self.set_pose(pose)
        self.radius = radius
        self.height = height
        self.color = color

        # Navigation properties
        self.set_path_planner(path_planner)
        self.set_path_executor(path_executor)
        self.current_path = None
        self.current_goal = None
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
        """
        Sets the robot pose.

        :param pose: New robot pose
        :type pose: :class:`pyrobosim.utils.pose.Pose`
        """
        self.pose = pose

    def set_path_planner(self, path_planner):
        """
        Sets a path planner for navigation.

        :param path_planner: Path planner for navigation
            (see e.g., :class:`pyrobosim.navigation.rrt.RRTPlanner`).
        :type path_planner: PathPlanner, optional
        """
        self.path_planner = path_planner

    def plan_path(self, start=None, goal=None):
        """
        Plans a path to a goal position.

        :param start: Start pose for the robot.
            If not specified, will default to the robot pose.
        :type start: :class:`pyrobosim.utils.pose.Pose`, optional
        :param goal: Goal pose for the robot. If not specified,returns None.
        :type goal: :class:`pyrobosim.utils.pose.Pose`, optional
        """
        if start is None:
            start = self.pose
        if goal is None:
            warnings.warn("Did not specify a goal. Returning None.")
            return None
        return self.path_planner.plan(start, goal)

    def set_path_executor(self, path_executor):
        """
        Sets a path executor for navigation.

        :param path_executor: Path executor for navigation (see e.g.,
            :class:`pyrobosim.navigation.execution.ConstantVelocityExecutor`).
        :type path_executor: PathExecutor, optional
        """
        self.path_executor = path_executor
        if path_executor is None:
            return
        path_executor.robot = self

    def follow_path(
        self,
        path,
        target_location=None,
        realtime_factor=1.0,
        use_thread=True,
        blocking=False,
    ):
        """
        Follows a specified path using the attached path executor.

        :param path: The path to follow.
        :type path: :class:`pyrobosim.utils.motion.Path`
        :param target_location: The target location at the intended goal,
            used for tracking robot state.
        :type target_location: Entity.
        :param realtime_factor: A real-time multiplier on execution speed,
            defaults to 1.0.
        :type realtime_factor: float
        :param use_thread: If True, spawns a new thread to execute the path.
        :type use_thread: bool
        :param blocking: If path executes in a new thread, set to True to block
            and wait for the thread to complete before returning.
        :return: True if path following is successful, or the path following
            thread is successfully started.
        :rtype: bool
        """
        if path is None or self.path_executor is None:
            return

        if use_thread:
            # Start a thread with the path execution
            self.nav_thread = threading.Thread(
                target=self.path_executor.execute, args=(path, realtime_factor)
            )
            self.nav_thread.start()
            if blocking:
                success = self.nav_thread.join()
            else:
                success = True
        else:
            success = self.path_executor.execute(path, realtime_factor)

        # Update the robot state if successful.
        if success and target_location is not None:
            self.location = target_location
        self.current_path = None
        return success

    def pick_object(self, obj_query):
        """
        Picks up an object in the world given an object and/or location query.

        :param obj_query: The object query (name, category, etc.).
        :type obj_query: str
        :return: True if picking succeeds, else False
        :rtype: bool
        """
        # Validate input
        if self.manipulated_object is not None:
            obj_name = self.manipulated_object.name
            warnings.warn(f"Robot is already holding {obj_name}.")
            return False

        # Get object
        loc = self.location
        if isinstance(obj_query, Object):
            obj = obj_query
        else:
            obj = self.world.get_object_by_name(obj_query)
            if not obj:
                obj = resolve_to_object(
                    self.world,
                    category=obj_query,
                    location=loc,
                    resolution_strategy="nearest",
                    robot=self
                )
            if not obj:
                warnings.warn(f"Found no object {obj_query} to pick.")
                return False

        # Validate the robot location
        if obj.parent != loc:
            warnings.warn(
                f"{obj.name} is at {obj.parent.name} and robot "
                + f"is at {loc.name}. Cannot pick."
            )
            return False

        # Denote the target object as the manipulated object
        self.manipulated_object = obj
        obj.parent.children.remove(obj)
        obj.parent = self
        obj.pose = self.pose
        return True

    def place_object(self, pose=None):
        """
        Places an object in a target location and (optionally) pose.

        :param pose: Placement pose (if not specified, will be sampled).
        :type pose: :class:`pyrobosim.utils.pose.Pose`, optional
        :return: True if placement succeeds, else False
        :rtype: bool
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
        is_valid_pose = False
        poly = inflate_polygon(
            self.manipulated_object.raw_polygon, self.world.object_radius
        )
        if pose is None:
            # If no pose was specified, sample one
            for _ in range(self.world.max_object_sample_tries):
                x_sample, y_sample = sample_from_polygon(loc.polygon)
                yaw_sample = np.random.uniform(-np.pi, np.pi)
                pose_sample = Pose(x=x_sample, y=y_sample, yaw=yaw_sample)
                sample_poly = transform_polygon(poly, pose_sample)
                is_valid_pose = sample_poly.within(loc.polygon)
                for other_obj in loc.children:
                    is_valid_pose = is_valid_pose and not sample_poly.intersects(
                        other_obj.polygon
                    )
                if is_valid_pose:
                    pose = pose_sample
                    break
            if not is_valid_pose:
                warnings.warn(f"Could not sample a placement position at {loc.name}")
                return False
        else:
            # If a pose was specified, collision check it
            poly = transform_polygon(poly, pose)
            is_valid_pose = poly.within(loc.polygon)
            for other_obj in loc.children:
                is_valid_pose = is_valid_pose and not poly.intersects(other_obj.polygon)
            if not is_valid_pose:
                warnings.warn(f"Pose in collision or not in location {loc.name}.")
                return False

        if is_valid_pose:
            self.manipulated_object.parent = loc
            self.manipulated_object.set_pose(pose)
            self.manipulated_object.create_polygons()
            loc.children.append(self.manipulated_object)
            self.manipulated_object = None
            return True

    def execute_action(self, action, blocking=False):
        """
        Executes an action, specified as a
        :class:`pyrobosim.planning.actions.TaskAction` object.

        :param action: Action to execute.
        :type action: :class:`pyrobosim.planning.actions.TaskAction`
        :param blocking: True to block execution until the action is complete.
        :type blocking: bool, optional
        :return: True if the action succeeds, or False otherwise.
        :rtype: bool
        """
        self.executing_action = True
        self.current_action = action
        if self.world.has_gui:
            self.world.gui.set_buttons_during_action(False)

        if action.type == "navigate":
            self.current_path = action.path
            if self.world.has_gui:
                self.executing_nav = True
                if isinstance(action.target_location, str):
                    tgt_loc = action.target_location
                else:
                    tgt_loc = action.target_location.name

                self.world.gui.canvas.nav_trigger.emit(self.name, tgt_loc)
                while self.executing_nav:
                    time.sleep(0.5)  # Delay to wait for navigation
                success = True  # TODO Need to keep track of nav status
            else:
                path = self.world.find_path(action.target_location, robot=self)
                success = self.follow_path(
                    path,
                    target_location=action.target_location,
                    realtime_factor=1.0,
                    blocking=blocking,
                )
            self.current_path = None

        elif action.type == "pick":
            if self.world.has_gui:
                success = self.world.gui.canvas.pick_object(self, action.object)
            else:
                success = self.pick_object(action.object)

        elif action.type == "place":
            if self.world.has_gui:
                success = self.world.gui.canvas.place_object(self, action.pose)
            else:
                success = self.place_object(action.pose)

        else:
            print(f"[{self.name}] Invalid action type: {action.type}")
            success = False

        if self.world.has_gui:
            self.world.gui.set_buttons_during_action(True)
        print(f"[{self.name}] Action completed with success: {success}")
        self.current_action = None
        self.executing_action = False
        return success

    def execute_plan(self, plan, blocking=False, delay=0.5):
        """
        Executes a task plan, specified as a
        :class:`pyrobosim.planning.actions.TaskPlan` object.

        :param plan: Task plan to execute.
        :type plan: :class:`pyrobosim.planning.actions.TaskPlan`
        :param blocking: True to block execution until the action is complete.
        :type blocking: bool, optional
        :param delay: Artificial delay between actions for visualization.
        :type delay: float, optional
        :return: True if the plan succeeds, or False otherwise.
        :rtype: bool
        """
        if plan is None:
            print(f"[{self.name}] Plan is None. Returning.")
            return False

        self.executing_plan = True
        self.current_plan = plan
        print(f"[{self.name}] Executing task plan...")
        if self.world.has_gui:
            self.world.gui.set_buttons_during_action(False)

        success = True
        num_acts = len(plan.actions)
        for n, act_msg in enumerate(plan.actions):
            print(f"[{self.name}] Executing action {act_msg.type} [{n+1}/{num_acts}]")
            success = self.execute_action(act_msg, blocking=blocking)
            if not success:
                print(f"[{self.name}] Task plan failed to execute on action {n+1}")
                break
            time.sleep(delay)  # Artificial delay between actions

        if self.world.has_gui:
            self.world.gui.set_buttons_during_action(True)
        print(f"[{self.name}] Task plan completed with success: {success}")
        self.current_plan = None
        self.executing_plan = False
        return success

    def __repr__(self):
        """Returns printable string."""
        return f"Robot: {self.name}"

    def print_details(self):
        """Prints string with details."""
        print(f"Robot: {self.name}, ID={self.id}\n\t{self.pose}")
