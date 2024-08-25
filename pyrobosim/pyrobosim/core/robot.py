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
from ..planning.actions import ExecutionResult, ExecutionStatus
from ..utils.knowledge import query_to_entity
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
        action_execution_options={},
        initial_battery_level=100.0,
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
        :param action_execution_options: A dictionary of action names and their execution options.
            This defines properties such as delays and nondeterminism.
        :type action_execution_options: dict[str, :class:`pyrobosim.planning.actions.ExecutionOptions`]
        :param initial_battery_level: The initial battery charge, from 0 to 100.
        :type initial_battery_level: float
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
        self.last_nav_result = ExecutionResult()
        self.set_path_planner(path_planner)
        self.set_path_executor(path_executor)

        # Manipulation properties
        self.grasp_generator = grasp_generator
        self.last_grasp_selection = None

        # Action execution options
        self.action_execution_options = action_execution_options
        self.current_action = None
        self.executing_action = False
        self.current_plan = None
        self.executing_plan = False
        self.canceling_execution = False
        self.battery_level = initial_battery_level

        # World interaction properties
        self.world = None
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
        return isinstance(self.location, Hallway) or isinstance(
            self.location, ObjectSpawn
        )

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

            if isinstance(goal, str):
                query_list = [elem for elem in goal.split(" ") if elem]
                goal = query_to_entity(
                    self.world,
                    query_list,
                    mode="location",
                    robot=self,
                    resolution_strategy="nearest",
                )
                if not goal:
                    warnings.warn(
                        f"Could not resolve goal location query: {query_list}"
                    )
                    return None

            goal_node = self.world.graph_node_from_entity(goal, robot=self)
            if goal_node is None:
                warnings.warn(f"Could not find graph node associated with goal.")
                return None
            goal = goal_node.pose

        path = self.path_planner.plan(start, goal)
        if self.world and self.world.has_gui:
            show_graphs = True
            self.world.gui.canvas.show_planner_and_path_signal.emit(
                self, show_graphs, path
            )
        return path

    def follow_path(
        self,
        path,
        realtime_factor=1.0,
    ):
        """
        Follows a specified path using the attached path executor.

        :param path: The path to follow.
        :type path: :class:`pyrobosim.utils.motion.Path`
        :param realtime_factor: A real-time multiplier on execution speed,
            defaults to 1.0.
        :type realtime_factor: float
        :return: An object describing the execution result.
        :rtype: :class:`pyrobosim.planning.actions.ExecutionResult`
        """
        self.last_nav_result = ExecutionResult()

        if path is None:
            self.executing_nav = False
            message = "No path to execute."
            warnings.warn(message)
            return ExecutionResult(
                status=ExecutionStatus.PRECONDITION_FAILURE,
                message=message,
            )
        if self.path_executor is None:
            self.executing_nav = False
            message = "No path executor. Cannot follow path."
            warnings.warn(message)
            return ExecutionResult(
                status=ExecutionStatus.PRECONDITION_FAILURE,
                message=message,
            )

        if path.num_poses == 0:
            # Trivial case where the path is empty.
            result = ExecutionResult(status=ExecutionStatus.SUCCESS)
            self.last_nav_result = result
        else:
            # Execute in this thread and check that the robot made it to its goal pose.
            result = self.path_executor.execute(path, realtime_factor)

        # Check that the robot made it to its goal pose at the end of execution.
        at_goal_pose = self.get_pose().is_approx(path.poses[-1])
        if result.is_success() and not at_goal_pose:
            result = ExecutionResult(
                status=ExecutionStatus.POSTCONDITION_FAILURE,
                message="Robot is not at its intended target pose.",
            )

        # Update the robot state.
        exec_options = self.action_execution_options.get("navigate")
        if exec_options:
            self.battery_level = max(
                0.0,
                self.battery_level
                - exec_options.battery_usage
                * self.path_executor.current_distance_traveled,
            )
        if self.world:
            self.location = self.world.get_location_from_pose(self.get_pose())
            if (
                isinstance(self.location, ObjectSpawn)
                and self.location.parent.is_charger
            ):
                print(f"[{self.name}] Battery charged at {self.location.name}!")
                self.battery_level = 100.0
        self.last_nav_result = result
        return result

    def navigate(
        self,
        start=None,
        goal=None,
        path=None,
        realtime_factor=1.0,
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
        :return: An object describing the execution result.
        :rtype: :class:`pyrobosim.planning.actions.ExecutionResult`
        """
        if self.battery_level <= 0.0:
            message = "Out of battery. Cannot navigate."
            warnings.warn(message)
            self.last_nav_result = ExecutionResult(
                status=ExecutionStatus.PRECONDITION_FAILURE, message=message
            )
            return self.last_nav_result

        if path is None:
            path = self.plan_path(start, goal)
            if path is None or path.num_poses == 0:
                self.executing_nav = False
                message = "Failed to plan a path."
                warnings.warn(message)
                self.last_nav_result = ExecutionResult(
                    status=ExecutionStatus.PLANNING_FAILURE,
                    message=message,
                )
                return self.last_nav_result
        elif self.world and self.world.has_gui:
            show_graphs = False
            self.world.gui.canvas.show_planner_and_path_signal.emit(
                self, show_graphs, path
            )

        # Simulate execution options.
        exec_options = self.action_execution_options.get("navigate")
        if exec_options:
            if not exec_options.should_succeed():
                message = f"[{self.name}] Simulated navigation failure."
                print(message)
                self.last_nav_result = ExecutionResult(
                    status=ExecutionStatus.EXECUTION_FAILURE, message=message
                )
                return self.last_nav_result

        return self.follow_path(path, realtime_factor=realtime_factor)

    def reset_path_planner(self):
        """Resets the robot's path planner, if available."""
        if self.path_planner is None:
            warnings.warn(f"[{self.name}] Robot has no path planner. Cannot reset.")
            return

        if not hasattr(self.path_planner, "reset"):
            warnings.warn(
                f"[{self.name}] Path planner does not have a reset() method. Cannot reset."
            )
            return

        self.path_planner.reset()
        if self.world.has_gui:
            show_graphs = True
            path = None
            self.world.gui.canvas.show_planner_and_path_signal.emit(
                self, show_graphs, path
            )

    def pick_object(self, obj_query, grasp_pose=None):
        """
        Picks up an object in the world given an object and/or location query.

        :param obj_query: The object query (name, category, etc.).
        :type obj_query: str
        :param grasp_pose: A pose describing how to manipulate the object.
        :type grasp_pose: :class:`pyrobosim.utils.pose.Pose`, optional
        :return: An object describing the execution result.
        :rtype: :class:`pyrobosim.planning.actions.ExecutionResult`
        """
        # Validate input
        if self.manipulated_object is not None:
            obj_name = self.manipulated_object.name
            message = f"Robot is already holding {obj_name}."
            warnings.warn(message)
            return ExecutionResult(
                status=ExecutionStatus.PRECONDITION_FAILURE, message=message
            )

        # Get object
        loc = self.location
        if isinstance(self.location, str):
            loc = self.world.get_entity_by_name(self.location)
        if isinstance(obj_query, Object):
            obj = obj_query
        else:
            obj = self.world.get_object_by_name(obj_query)
            if not obj and isinstance(obj_query, str):
                obj = query_to_entity(
                    self.world,
                    obj_query,
                    mode="object",
                    robot=self,
                    resolution_strategy="nearest",
                )
            if not obj:
                message = f"Found no object {obj_query} to pick."
                warnings.warn(message)
                return ExecutionResult(
                    status=ExecutionStatus.PRECONDITION_FAILURE, message=message
                )

        # Validate the robot location
        if obj.parent != loc:
            message = f"{obj.name} is at {obj.parent.name} and robot is at {loc.name}. Cannot pick."
            warnings.warn(message)
            return ExecutionResult(
                status=ExecutionStatus.PRECONDITION_FAILURE, message=message
            )
        if not loc.is_open:
            message = f"{loc.parent.name} is not open. Cannot pick object."
            warnings.warn(message)
            return ExecutionResult(
                status=ExecutionStatus.PRECONDITION_FAILURE, message=message
            )
        if self.battery_level <= 0.0:
            message = "Out of battery. Cannot pick."
            warnings.warn(message)
            return ExecutionResult(
                status=ExecutionStatus.PRECONDITION_FAILURE, message=message
            )

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
                message = f"Could not generate valid grasps. Cannot pick object."
                warnings.warn(message)
                return ExecutionResult(
                    status=ExecutionStatus.PLANNING_FAILURE, message=message
                )
            else:
                # TODO: For now, just pick a random grasp.
                self.last_grasp_selection = np.random.choice(grasps)
        if self.last_grasp_selection is not None:
            print(f"Selected {self.last_grasp_selection}")

        # Simulate execution options.
        exec_options = self.action_execution_options.get("pick")
        if exec_options:
            self.battery_level = max(
                0.0, self.battery_level - exec_options.battery_usage
            )
            if not exec_options.should_succeed():
                message = f"[{self.name}] Simulated pick failure."
                print(message)
                return ExecutionResult(
                    status=ExecutionStatus.EXECUTION_FAILURE, message=message
                )

        # Denote the target object as the manipulated object
        self._attach_object(obj)
        return ExecutionResult(status=ExecutionStatus.SUCCESS)

    def place_object(self, pose=None):
        """
        Places an object in a target location and (optionally) pose.

        :param pose: Placement pose (if not specified, will be sampled).
        :type pose: :class:`pyrobosim.utils.pose.Pose`, optional
        :return: An object describing the execution result.
        :rtype: :class:`pyrobosim.planning.actions.ExecutionResult`
        """
        # Validate input
        if self.manipulated_object is None:
            message = "No manipulated object. Cannot place."
            warnings.warn(message)
            return ExecutionResult(
                status=ExecutionStatus.PRECONDITION_FAILURE, message=message
            )

        # Validate the robot location
        loc = self.location
        if isinstance(loc, str):
            loc = self.world.get_entity_by_name(self.location)
        if not isinstance(loc, ObjectSpawn):
            message = f"{loc} is not an object spawn. Cannot place object."
            warnings.warn(message)
            return ExecutionResult(
                status=ExecutionStatus.PRECONDITION_FAILURE, message=message
            )
        if not loc.is_open:
            message = f"{loc.parent.name} is not open. Cannot place object."
            warnings.warn(message)
            return ExecutionResult(
                status=ExecutionStatus.PRECONDITION_FAILURE, message=message
            )
        if self.battery_level <= 0.0:
            message = "Out of battery. Cannot place."
            warnings.warn(message)
            return ExecutionResult(
                status=ExecutionStatus.PRECONDITION_FAILURE, message=message
            )

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
                message = f"Could not sample a placement position at {loc.name}"
                warnings.warn(message)
                return ExecutionResult(
                    status=ExecutionStatus.PLANNING_FAILURE, message=message
                )
        else:
            # If a pose was specified, collision check it
            poly = transform_polygon(poly, pose)
            is_valid_pose = poly.within(loc.polygon)
            for other_obj in loc.children:
                is_valid_pose = is_valid_pose and not poly.intersects(
                    other_obj.collision_polygon
                )
            if not is_valid_pose:
                message = f"Pose in collision or not in location {loc.name}."
                warnings.warn(message)
                return ExecutionResult(
                    status=ExecutionStatus.PLANNING_FAILURE, message=message
                )

        # Simulate execution options.
        exec_options = self.action_execution_options.get("place")
        if exec_options:
            self.battery_level = max(
                0.0, self.battery_level - exec_options.battery_usage
            )
            if not exec_options.should_succeed():
                message = f"[{self.name}] Simulated place failure."
                print(message)
                return ExecutionResult(
                    status=ExecutionStatus.EXECUTION_FAILURE, message=message
                )

        self.manipulated_object.parent = loc
        self.manipulated_object.set_pose(pose)
        self.manipulated_object.create_polygons()
        loc.children.append(self.manipulated_object)
        self.manipulated_object = None
        return ExecutionResult(status=ExecutionStatus.SUCCESS)

    def detect_objects(self, target_object=None):
        """
        Detects all objects at the robot's current location.

        :param target_object: The name of a target object or category.
            If None, the action succeeds regardless of which object is found.
            Otherwise, the action succeeds only if the target object is found.
        :type target_object: str
        :return: An object describing the execution result.
        :rtype: :class:`pyrobosim.planning.actions.ExecutionResult`
        """
        self.last_detected_objects = []

        if not self.at_object_spawn():
            message = f"Robot is not at an object spawn. Cannot detect objects."
            warnings.warn(message)
            return ExecutionResult(
                status=ExecutionStatus.PRECONDITION_FAILURE, message=message
            )
        if not self.location.is_open:
            message = f"{self.location.parent.name} is not open. Cannot detect objects."
            warnings.warn(message)
            return ExecutionResult(
                status=ExecutionStatus.PRECONDITION_FAILURE, message=message
            )

        if self.battery_level <= 0.0:
            message = "Out of battery. Cannot detect objects."
            warnings.warn(message)
            return ExecutionResult(
                status=ExecutionStatus.PRECONDITION_FAILURE, message=message
            )

        # Simulate execution options.
        exec_options = self.action_execution_options.get("detect")
        if exec_options:
            self.battery_level = max(
                0.0, self.battery_level - exec_options.battery_usage
            )
            if not exec_options.should_succeed():
                message = f"[{self.name}] Simulated detection failure."
                print(message)
                return ExecutionResult(
                    status=ExecutionStatus.EXECUTION_FAILURE, message=message
                )

        # Add all the objects at the current robot's location.
        for obj in self.location.children:
            self.known_objects.add(obj)

        # If a target object was specified, look for a matching instance.
        # We should only return SUCCESS if one such instance was found.
        if not target_object:
            self.last_detected_objects = self.location.children
            return ExecutionResult(status=ExecutionStatus.SUCCESS)
        else:
            self.last_detected_objects = [
                obj
                for obj in self.location.children
                if obj.name == target_object or obj.category == target_object
            ]
            if len(self.last_detected_objects) > 0:
                return ExecutionResult(status=ExecutionStatus.SUCCESS)
            else:
                return ExecutionResult(
                    status=ExecutionStatus.EXECUTION_FAILURE,
                    message=f"Failed to detect any objects matching the query '{target_object}'.",
                )

    def open_location(self):
        """
        Opens the robot's current location, if available.

        :return: An object describing the execution result.
        :rtype: :class:`pyrobosim.planning.actions.ExecutionResult`
        """
        if self.location is None:
            message = "Robot location is not set. Cannot open."
            warnings.warn(message)
            return ExecutionResult(
                status=ExecutionStatus.PRECONDITION_FAILURE, message=message
            )

        if self.manipulated_object is not None:
            message = "Robot is holding an object. Cannot open."
            warnings.warn(message)
            return ExecutionResult(
                status=ExecutionStatus.PRECONDITION_FAILURE, message=message
            )

        if not self.at_openable_location():
            message = "Robot is not at an openable location."
            warnings.warn(message)
            return ExecutionResult(
                status=ExecutionStatus.PRECONDITION_FAILURE, message=message
            )

        if self.battery_level <= 0.0:
            message = "Out of battery. Cannot open location."
            warnings.warn(message)
            return ExecutionResult(
                status=ExecutionStatus.PRECONDITION_FAILURE, message=message
            )

        # Simulate execution options.
        exec_options = self.action_execution_options.get("open")
        if exec_options:
            self.battery_level = max(
                0.0, self.battery_level - exec_options.battery_usage
            )
            if not exec_options.should_succeed():
                message = f"[{self.name}] Simulated opening failure."
                print(message)
                return ExecutionResult(
                    status=ExecutionStatus.EXECUTION_FAILURE, message=message
                )

        if isinstance(self.location, ObjectSpawn):
            return self.world.open_location(self.location.parent)
        elif isinstance(self.location, Hallway):
            return self.world.open_hallway(self.location)

        # This should not happen
        return ExecutionResult(status=ExecutionResult.UNKNOWN)

    def close_location(self):
        """
        Closes the robot's current location, if available.

        :return: An object describing the execution result.
        :rtype: :class:`pyrobosim.planning.actions.ExecutionResult`
        """
        if self.location is None:
            message = "Robot location is not set. Cannot close."
            warnings.warn(message)
            return ExecutionResult(
                status=ExecutionStatus.PRECONDITION_FAILURE, message=message
            )

        if self.manipulated_object is not None:
            message = "Robot is holding an object. Cannot close."
            warnings.warn(message)
            return ExecutionResult(
                status=ExecutionStatus.PRECONDITION_FAILURE, message=message
            )

        if not self.at_openable_location():
            message = "Robot is not at a closeable location."
            warnings.warn(message)
            return ExecutionResult(
                status=ExecutionStatus.PRECONDITION_FAILURE, message=message
            )

        if self.battery_level <= 0.0:
            message = "Out of battery. Cannot close location."
            warnings.warn(message)
            return ExecutionResult(
                status=ExecutionStatus.PRECONDITION_FAILURE, message=message
            )

        # Simulate execution options.
        exec_options = self.action_execution_options.get("close")
        if exec_options:
            self.battery_level = max(
                0.0, self.battery_level - exec_options.battery_usage
            )
            if not exec_options.should_succeed():
                message = f"[{self.name}] Simulated closing failure."
                print(message)
                return ExecutionResult(
                    status=ExecutionStatus.EXECUTION_FAILURE, message=message
                )

        if isinstance(self.location, ObjectSpawn):
            return self.world.close_location(self.location.parent)
        elif isinstance(self.location, Hallway):
            return self.world.close_hallway(self.location, ignore_robots=[self])

        # This should not happen
        return ExecutionResult(status=ExecutionResult.UNKNOWN)

    def execute_action(self, action):
        """
        Executes an action, specified as a
        :class:`pyrobosim.planning.actions.TaskAction` object.

        :param action: Action to execute.
        :type action: :class:`pyrobosim.planning.actions.TaskAction`
        :return: An object describing the execution result.
        :rtype: :class:`pyrobosim.planning.actions.ExecutionResult`
        """
        self.executing_action = True
        self.current_action = action
        if self.world.has_gui:
            self.world.gui.set_buttons_during_action(False)

        if action.type == "navigate":
            self.executing_nav = True
            path = action.path if action.path.num_poses > 0 else None
            if self.world.has_gui:
                if action.target_location and not isinstance(
                    action.target_location, str
                ):
                    target_location_name = action.target_location.name
                else:
                    target_location_name = action.target_location

                self.world.gui.canvas.navigate_signal.emit(
                    self, target_location_name, path
                )
                while self.executing_nav:
                    time.sleep(0.5)  # Delay to wait for navigation
                result = self.last_nav_result
            else:
                result = self.navigate(
                    goal=action.target_location,
                    path=path,
                    realtime_factor=1.0,
                )

        elif action.type == "pick":
            if self.world.has_gui:
                result = self.world.gui.canvas.pick_object(
                    self, action.object, action.pose
                )
            else:
                result = self.pick_object(action.object, action.pose)

        elif action.type == "place":
            if self.world.has_gui:
                result = self.world.gui.canvas.place_object(self, action.pose)
            else:
                result = self.place_object(action.pose)

        elif action.type == "detect":
            if self.world.has_gui:
                result = self.world.gui.canvas.detect_objects(self, action.object)
            else:
                result = self.detect_objects(action.object)

        elif action.type == "open":
            if self.world.has_gui:
                result = self.world.gui.canvas.open_location(self)
            else:
                result = self.open_location()

        elif action.type == "close":
            if self.world.has_gui:
                result = self.world.gui.canvas.close_location(self)
            else:
                result = self.close_location()

        else:
            message = f"[{self.name}] Invalid action type: {action.type}."
            warnings.warn(message)
            result = ExecutionResult(
                status=ExecutionStatus.INVALID_ACTION, message=message
            )

        if self.world.has_gui:
            self.world.gui.set_buttons_during_action(True)
        print(f"[{self.name}] Action completed with result: {result.status.name}")
        self.current_action = None
        self.executing_action = False
        return result

    def cancel_actions(self):
        """Cancels any currently running actions for the robot."""
        if not (self.executing_action or self.executing_plan or self.executing_nav):
            warnings.warn("There is no running action or plan to cancel.")
            return

        self.canceling_execution = True
        if self.executing_nav and self.path_executor is not None:
            print(f"[{self.name}] Canceling path execution...")
            self.path_executor.cancel_execution = True

    def execute_plan(self, plan, delay=0.5):
        """
        Executes a task plan, specified as a
        :class:`pyrobosim.planning.actions.TaskPlan` object.

        :param plan: Task plan to execute.
        :type plan: :class:`pyrobosim.planning.actions.TaskPlan`
        :param delay: Artificial delay between actions for visualization.
        :type delay: float, optional
        :return: A tuple containing an execution result and the number of actions completed.
        :rtype: tuple[:class:`pyrobosim.planning.actions.ExecutionResult`, int]
        """
        if plan is None:
            message = f"[{self.name}] Plan is None. Returning."
            warnings.warn(message)
            return (
                ExecutionResult(status=ExecutionStatus.INVALID_ACTION, message=message),
                0,
            )

        self.executing_plan = True
        self.current_plan = plan

        print(f"[{self.name}] Executing task plan...")
        if self.world.has_gui:
            self.world.gui.set_buttons_during_action(False)

        result = ExecutionResult(status=ExecutionStatus.SUCCESS)
        num_completed = 0
        num_acts = len(plan.actions)
        for n, act_msg in enumerate(plan.actions):
            if self.canceling_execution:
                self.canceling_execution = False
                message = f"[{self.name}] Canceled plan execution."
                print(message)
                result = ExecutionResult(
                    status=ExecutionStatus.CANCELED, message=message
                )
                break

            print(f"[{self.name}] Executing action {act_msg.type} [{n+1}/{num_acts}]")
            result = self.execute_action(act_msg)
            if not result.is_success():
                print(
                    f"[{self.name}] Task plan failed to execute on action {n+1}/{num_acts}"
                )
                break
            num_completed += 1
            time.sleep(delay)  # Artificial delay between actions

        if self.world.has_gui:
            self.world.gui.set_buttons_during_action(True)

        print(f"[{self.name}] Task plan completed with status: {result.status.name}")
        self.executing_plan = False
        self.current_plan = None
        return result, num_completed

    def __repr__(self):
        """Returns printable string."""
        return f"Robot: {self.name}"

    def print_details(self):
        """Prints string with details."""
        details_str = f"Robot: {self.name}"
        details_str += f"\n\t{self.get_pose()}"
        details_str += f"\n\tBattery: {self.battery_level:.2f}%"
        if self.partial_observability:
            details_str += "\n\tPartial observability enabled"
        print(details_str)
