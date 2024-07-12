""" Defines a robot which operates in a world. """

import time
import threading
import numpy as np
import warnings

from .dynamics import RobotDynamics2D
from .hallway import Hallway
from .locations import ObjectSpawn
from .objects import Object
from ..manipulation.grasping import Grasp
from ..utils.knowledge import resolve_to_object
from ..utils.polygon import sample_from_polygon, transform_polygon
from ..utils.pose import Pose


class Robot:
    """Representation of a robot in the world."""

    def __init__(
        self,
        name="robot",
        pose=Pose(),
        radius=0.0,
        height=0.0,
        color=(0.8, 0.0, 0.8),
        max_linear_velocity=np.inf,
        max_angular_velocity=np.inf,
        max_linear_acceleration=np.inf,
        max_angular_acceleration=np.inf,
        path_planner=None,
        path_executor=None,
        grasp_generator=None,
        partial_observability=False,
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
        :param max_linear_velocity: The maximum linear velocity magnitude, in m/s.
        :type max_linear_velocity: float
        :param max_angular_velocity: The maximum angular velocity magnitude, in rad/s.
        :type max_angular_velocity: float
        :param max_linear_acceleration: The maximum linear acceleration magnitude, in m/s^2.
        :type max_linear_acceleration: float
        :param max_angular_acceleration: The maximum angular acceleration magnitude, in rad/s^2.
        :type max_linear_acceleration: float
        :param path_planner: Path planner for navigation
            (see e.g., :class:`pyrobosim.navigation.rrt.RRTPlanner`).
        :type path_planner: PathPlanner, optional
        :param path_executor: Path executor for navigation (see e.g.,
            :class:`pyrobosim.navigation.execution.ConstantVelocityExecutor`).
        :type path_executor: PathExecutor, optional
        :param grasp_generator: Grasp generator for manipulating objects.
        :type grasp_generator: :class:`pyrobosim.manipulation.grasping.GraspGenerator`, optional
        :param partial_observability: If False, the robot can access all objects in the world.
            If True, it must detect new objects at specific locations.
        :type partial_observability: bool, optional
        """
        # Basic properties
        self.name = name
        self.radius = radius
        self.height = height
        self.color = color

        if name == "world":
            raise ValueError("Robots cannot be named 'world'.")

        # Dynamics properties
        self.dynamics = RobotDynamics2D(
            robot=self,
            init_pose=pose,
            max_linear_velocity=max_linear_velocity,
            max_angular_velocity=max_angular_velocity,
            max_linear_acceleration=max_linear_acceleration,
            max_angular_acceleration=max_angular_acceleration,
        )

        # Navigation properties
        self.executing_nav = False
        self.last_nav_successful = False
        self.set_path_planner(path_planner)
        self.set_path_executor(path_executor)

        # Manipulation properties
        self.grasp_generator = grasp_generator
        self.last_grasp_selection = None

        # World interaction properties
        self.world = None
        self.current_action = None
        self.executing_action = False
        self.current_plan = None
        self.executing_plan = False
        self.location = None
        self.manipulated_object = None
        self.partial_observability = partial_observability
        self.known_objects = set()
        self.last_detected_objects = []

    def get_pose(self):
        """
        Gets the robot pose.

        :return: The robot pose.
        :rtype: :class:`pyrobosim.utils.pose.Pose`
        """
        return self.dynamics.pose

    def set_pose(self, pose):
        """
        Sets the robot pose.

        :param pose: New robot pose
        :type pose: :class:`pyrobosim.utils.pose.Pose`
        """
        self.dynamics.pose = pose

    def set_path_planner(self, path_planner):
        """
        Sets a path planner for navigation.

        :param path_planner: Path planner for navigation
            (see e.g., :class:`pyrobosim.navigation.rrt.RRTPlanner`).
        :type path_planner: PathPlanner, optional
        """
        self.path_planner = path_planner

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

    def is_moving(self):
        """
        Checks whether the robot is moving, either due to a navigation action or velocity commands.

        :return: True if the robot is moving, False otherwise.
        :rtype: bool
        """
        return self.executing_nav or np.count_nonzero(self.dynamics.velocity) > 0

    def is_in_collision(self):
        """
        Checks whether the last step of dynamics put the robot in collision.

        :return: True if the robot is in collision, False otherwise.
        :rtype: bool
        """
        return self.dynamics.collision

    def at_object_spawn(self):
        """
        Checks whether a robot is at an object spawn.

        :return: True if the robot is at an object spawn, False otherwise.
        :rtype: bool
        """
        return isinstance(self.location, ObjectSpawn)

    def get_known_objects(self):
        """
        Returns a list of objects known by the robot.

        :return: The list of known objects.
        :rtype: list[Object]
        """
        if self.world is None:
            return []

        if self.partial_observability:
            return list(self.known_objects)

        return self.world.objects

    def at_openable_location(self):
        """
        Checks whether the robot is at an openable location.

        :return: True if the robot is at an openable location, else False.
        :type: bool
        """
        return isinstance(self.location, Hallway)

    def _attach_object(self, obj):
        """
        Helper function to attach an object in the world to the robot.
        Be careful calling this function directly as it does not do any validation.
        When possible, you should be using `pick_object`.

        :param obj: Object to manipulate
        :type obj: :class:`pyrobosim.core.objects.Object`
        """
        self.manipulated_object = obj
        obj.parent.children.remove(obj)
        obj.parent = self
        obj.set_pose(self.get_pose())

    def plan_path(self, start=None, goal=None):
        """
        Plans a path to a goal position.

        :param start: Start pose for the robot.
            If not specified, will default to the robot pose.
        :type start: :class:`pyrobosim.utils.pose.Pose`, optional
        :param goal: Goal pose or entity name for the robot.
            If not specified, returns None.
        :type goal: :class:`pyrobosim.utils.pose.Pose` / str, optional
        :return: The path, if one was found, otherwise None.
        :rtype: :class:`pyrobosim.utils.motion.Path` or None
        """
        if self.path_planner is None:
            warnings.warn(f"No path planner attached to robot {self.name}.")
            return None

        if start is None:
            start = self.get_pose()

        if goal is None:
            warnings.warn("Did not specify a goal. Returning None.")
            return None

        # If the goal is not a pose, we need to extract it from the world knowledge.
        if not isinstance(goal, Pose):
            if self.world is None:
                warnings.warn("Cannot specify a string goal if there is no world set.")
                return None
            goal_node = self.world.graph_node_from_entity(goal, robot=self)
            if goal_node is None:
                return None
            goal = goal_node.pose

        path = self.path_planner.plan(start, goal)
        if self.world and self.world.has_gui:
            self.world.gui.canvas.show_planner_and_path(robot=self, path=path)
        return path

    def follow_path(
        self,
        path,
        realtime_factor=1.0,
        use_thread=True,
        blocking=False,
    ):
        """
        Follows a specified path using the attached path executor.

        :param path: The path to follow.
        :type path: :class:`pyrobosim.utils.motion.Path`
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
        self.last_nav_successful = False

        if path is None:
            warnings.warn("No path to execute.")
            self.executing_nav = False
            return False
        if self.path_executor is None:
            warnings.warn("No path executor. Cannot follow path.")
            self.executing_nav = False
            return False

        if path.num_poses == 0:
            success = True
        elif use_thread:
            # Start a thread with the path execution
            self.nav_thread = threading.Thread(
                target=self.path_executor.execute, args=(path, realtime_factor)
            )
            self.nav_thread.start()
            if blocking:
                # Check that robot made it to its goal pose at the end of execution.
                self.nav_thread.join()
                success = self.get_pose().is_approx(path.poses[-1])
            else:
                # Cannot check in the non-blocking case, so we just assume success.
                success = True
        else:
            # Execute in this thread and check that the robot made it to its goal pose.
            success = self.path_executor.execute(path, realtime_factor)
            success |= self.get_pose().is_approx(path.poses[-1])

        # Update the robot state if successful.
        if self.world:
            self.location = self.world.get_location_from_pose(self.get_pose())
        self.last_nav_successful = success
        return success

    def navigate(
        self,
        start=None,
        goal=None,
        path=None,
        realtime_factor=1.0,
        use_thread=True,
        blocking=False,
    ):
        """
        Executes a navigation task, which combines path planning and following.

        :param start: Start pose for the robot.
            If not specified, will default to the robot pose.
        :type start: :class:`pyrobosim.utils.pose.Pose`, optional
        :param goal: Goal pose or entity name for the robot.
            If not specified, returns None.
        :type goal: :class:`pyrobosim.utils.pose.Pose` / str, optional
        :param path: The path to follow.
        :type path: :class:`pyrobosim.utils.motion.Path`
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
        if path is None:
            path = self.plan_path(start, goal)
            if path is None or path.num_poses == 0:
                warnings.warn("Failed to plan a path.")
                self.executing_nav = False
                self.last_nav_successful = False
                return False

        return self.follow_path(
            path,
            realtime_factor=realtime_factor,
            use_thread=use_thread,
            blocking=blocking,
        )

    def pick_object(self, obj_query, grasp_pose=None):
        """
        Picks up an object in the world given an object and/or location query.

        :param obj_query: The object query (name, category, etc.).
        :type obj_query: str
        :param grasp_pose: A pose describing how to manipulate the object.
        :type grasp_pose: :class:`pyrobosim.utils.pose.Pose`, optional
        :return: True if picking succeeds, else False.
        :rtype: bool
        """
        # Validate input
        if self.manipulated_object is not None:
            obj_name = self.manipulated_object.name
            warnings.warn(f"Robot is already holding {obj_name}.")
            return False

        # Get object
        loc = self.location
        if isinstance(self.location, str):
            loc = self.world.get_entity_by_name(self.location)
        if isinstance(obj_query, Object):
            obj = obj_query
        else:
            obj = self.world.get_object_by_name(obj_query)
            if not obj and isinstance(obj_query, str):
                obj = resolve_to_object(
                    self.world,
                    category=obj_query,
                    location=loc,
                    resolution_strategy="nearest",
                    robot=self,
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

        # If a grasp generator has been specified and no explicit grasp has been provided,
        # generate grasps here.
        # TODO: Specify allowed grasp types
        if grasp_pose is not None:
            if self.grasp_generator is not None:
                grasp_properties = self.grasp_generator.properties
            else:
                grasp_properties = None
            self.last_grasp_selection = Grasp(
                properties=grasp_properties,
                origin_wrt_object=Pose(),
                origin_wrt_world=grasp_pose,
            )
        elif self.grasp_generator is not None:
            cuboid_pose = obj.get_grasp_cuboid_pose()
            grasps = self.grasp_generator.generate(
                obj.cuboid_dims,
                cuboid_pose,
                self.get_pose(),
                front_grasps=True,
                top_grasps=True,
                side_grasps=False,
            )

            if len(grasps) == 0:
                warnings.warn(f"Could not generate valid grasps. Cannot pick.")
                return False
            else:
                # TODO: For now, just pick a random grasp.
                self.last_grasp_selection = np.random.choice(grasps)
        if self.last_grasp_selection is not None:
            print(f"Selected {self.last_grasp_selection}")

        # Denote the target object as the manipulated object
        self._attach_object(obj)
        return True

    def place_object(self, pose=None):
        """
        Places an object in a target location and (optionally) pose.

        :param pose: Placement pose (if not specified, will be sampled).
        :type pose: :class:`pyrobosim.utils.pose.Pose`, optional
        :return: True if placement succeeds, else False.
        :rtype: bool
        """
        # Validate input
        if self.manipulated_object is None:
            warnings.warn("No manipulated object. Cannot place.")
            return False

        # Validate the robot location
        loc = self.location
        if isinstance(loc, str):
            loc = self.world.get_entity_by_name(self.location)
        if not isinstance(loc, ObjectSpawn):
            warnings.warn(f"{loc} is not an object spawn. Cannot place object.")
            return False

        # Place the object somewhere in the current location
        is_valid_pose = False
        poly = self.manipulated_object.raw_collision_polygon
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
                        other_obj.collision_polygon
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
                is_valid_pose = is_valid_pose and not poly.intersects(
                    other_obj.collision_polygon
                )
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

    def detect_objects(self, target_object=None):
        """
        Detects all objects at the robot's current location.

        :param target_object: The name of a target object or category.
            If None, the action succeeds regardless of which object is found.
            Otherwise, the action succeeds only if the target object is found.
        :type target_object: str
        :return: True if detection succeeds, else False.
        :rtype: bool
        """
        self.last_detected_objects = []

        if not self.at_object_spawn():
            warnings.warn(f"Robot is not at an object spawn. Cannot detect objects.")
            return False

        # Add all the objects at the current robot's location.
        for obj in self.location.children:
            self.known_objects.add(obj)

        # If a target object was specified, look for a matching instance.
        # We should only return True if one such instance was found.
        if target_object is None:
            self.last_detected_objects = self.location.children
            return True
        else:
            self.last_detected_objects = [
                obj
                for obj in self.location.children
                if obj.name == target_object or obj.category == target_object
            ]
            return len(self.last_detected_objects) > 0

    def open_location(self):
        """
        Opens the robot's current location, if available.

        :return: True if opening the location succeeds, else False.
        :rtype: bool
        """
        if self.location is None:
            warnings.warn("Robot location is not set. Cannot open.")
            return False

        if self.manipulated_object is not None:
            warnings.warn("Robot is holding an object. Cannot open.")
            return False

        if not self.at_openable_location():
            warnings.warn("Robot is not at an openable location.")
            return False

        if isinstance(self.location, Hallway):
            return self.world.open_hallway(self.location)

        # This should not happen
        return False

    def close_location(self):
        """
        Closes the robot's current location, if available.

        :return: True if closing the location succeeds, else False.
        :rtype: bool
        """
        if self.location is None:
            warnings.warn("Robot location is not set. Cannot close.")
            return False

        if self.manipulated_object is not None:
            warnings.warn("Robot is holding an object. Cannot close.")
            return False

        if not self.at_openable_location():
            warnings.warn("Robot is not at a closeable location.")
            return False

        if isinstance(self.location, Hallway):
            return self.world.close_hallway(self.location)

        # This should not happen
        return False

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
            self.executing_nav = True
            path = action.path if action.path.num_poses > 0 else None
            if self.world.has_gui:
                self.world.gui.canvas.navigate(self, action.target_location, path)
                while self.executing_nav:
                    time.sleep(0.5)  # Delay to wait for navigation
                success = self.last_nav_successful
            else:
                success = self.navigate(
                    goal=action.target_location,
                    path=path,
                    realtime_factor=1.0,
                    use_thread=True,
                    blocking=blocking,
                )
            self.executing_nav = False

        elif action.type == "pick":
            if self.world.has_gui:
                success = self.world.gui.canvas.pick_object(
                    self, action.object, action.pose
                )
            else:
                success = self.pick_object(action.object, action.pose)

        elif action.type == "place":
            if self.world.has_gui:
                success = self.world.gui.canvas.place_object(self, action.pose)
            else:
                success = self.place_object(action.pose)

        elif action.type == "detect":
            if self.world.has_gui:
                success = self.world.gui.canvas.detect_objects(self, action.object)
            else:
                success = self.detect_objects(action.object)

        elif action.type == "open":
            if self.world.has_gui:
                success = self.world.gui.canvas.open_location(self)
            else:
                success = self.open_location()

        elif action.type == "close":
            if self.world.has_gui:
                success = self.world.gui.canvas.close_location(self)
            else:
                success = self.close_location()

        else:
            warnings.warn(f"[{self.name}] Invalid action type: {action.type}.")
            success = False

        if self.world.has_gui:
            self.world.gui.set_buttons_during_action(True)
        print(f"[{self.name}] Action completed with success: {success}")
        if blocking:
            self.current_action = None
            self.executing_action = False
        return success

    def execute_plan(self, plan, delay=0.5):
        """
        Executes a task plan, specified as a
        :class:`pyrobosim.planning.actions.TaskPlan` object.

        :param plan: Task plan to execute.
        :type plan: :class:`pyrobosim.planning.actions.TaskPlan`
        :param delay: Artificial delay between actions for visualization.
        :type delay: float, optional
        :return: A tuple containing a boolean for whether the plan succeeded, and the number of completed actions.
        :rtype: tuple(bool, int)
        """
        if plan is None:
            warnings.warn(f"[{self.name}] Plan is None. Returning.")
            return False

        self.executing_plan = True
        self.current_plan = plan

        print(f"[{self.name}] Executing task plan...")
        if self.world.has_gui:
            self.world.gui.set_buttons_during_action(False)

        success = True
        num_completed = 0
        num_acts = len(plan.actions)
        for n, act_msg in enumerate(plan.actions):
            print(f"[{self.name}] Executing action {act_msg.type} [{n+1}/{num_acts}]")
            success = self.execute_action(act_msg, blocking=True)
            if not success:
                print(
                    f"[{self.name}] Task plan failed to execute on action {n+1}/{num_acts}"
                )
                break
            num_completed += 1
            time.sleep(delay)  # Artificial delay between actions

        if self.world.has_gui:
            self.world.gui.set_buttons_during_action(True)

        print(f"[{self.name}] Task plan completed with success: {success}")
        self.executing_plan = False
        self.current_plan = None
        return success, num_completed

    def __repr__(self):
        """Returns printable string."""
        return f"Robot: {self.name}"

    def print_details(self):
        """Prints string with details."""
        details_str = f"Robot: {self.name}, ID={self.id}"
        details_str += f"\n\t{self.get_pose()}"
        if self.partial_observability:
            details_str += "\n\tPartial observability enabled"
        print(details_str)
