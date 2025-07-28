"""Defines a robot which operates in a world."""

import itertools
import time
from threading import Lock
from typing import Any, Sequence

import numpy as np
import shapely
from matplotlib.text import Text
from shapely.geometry import Point, Polygon
from shapely.ops import unary_union

from .dynamics import RobotDynamics2D
from .hallway import Hallway
from .locations import ObjectSpawn
from .objects import Object
from .types import Entity, set_parent
from ..manipulation.grasping import Grasp, GraspGenerator
from ..navigation.types import PathExecutor, PathPlanner
from ..planning.actions import (
    ExecutionOptions,
    ExecutionResult,
    ExecutionStatus,
    TaskAction,
    TaskPlan,
)
from ..sensors.types import Sensor
from ..utils.logging import create_logger
from ..utils.polygon import sample_from_polygon, transform_polygon
from ..utils.path import Path
from ..utils.pose import Pose
from ..utils.general import parse_color
from ..utils.world_collision import check_occupancy


class Robot(Entity):
    """Representation of a robot in the world."""

    def __init__(
        self,
        name: str = "robot",
        pose: Pose = Pose(),
        radius: float = 0.0,
        height: float = 0.0,
        color: Sequence[float] = (0.8, 0.0, 0.8),
        max_linear_velocity: float = np.inf,
        max_angular_velocity: float = np.inf,
        max_linear_acceleration: float = np.inf,
        max_angular_acceleration: float = np.inf,
        path_planner: PathPlanner | None = None,
        path_executor: PathExecutor | None = None,
        grasp_generator: GraspGenerator | None = None,
        sensors: dict[str, Sensor] | None = None,
        start_sensor_threads: bool = True,
        partial_obs_objects: bool = False,
        partial_obs_hallways: bool = False,
        action_execution_options: dict[str, ExecutionOptions] = {},
        initial_battery_level: float = 100.0,
    ) -> None:
        """
        Creates a robot instance.

        :param name: Robot name.
        :param pose: Robot initial pose.
        :param radius: Robot radius, in meters.
        :param height: Robot height, in meters.
        :param color: Visualization color.
         Input can be:

         - an (R, G, B) tuple or list in the range (0.0, 1.0).
         - a string (e.g., "red").
         - a hexadecimal string (e.g., "#FF0000").
        :param max_linear_velocity: The maximum linear velocity magnitude, in m/s.
        :param max_angular_velocity: The maximum angular velocity magnitude, in rad/s.
        :param max_linear_acceleration: The maximum linear acceleration magnitude, in m/s^2.
        :param max_angular_acceleration: The maximum angular acceleration magnitude, in rad/s^2.
        :param path_planner: Path planner for navigation
            (see e.g., :class:`pyrobosim.navigation.rrt.RRTPlanner`).
        :param path_executor: Path executor for navigation (see e.g.,
            :class:`pyrobosim.navigation.execution.ConstantVelocityExecutor`).
        :param grasp_generator: Grasp generator for manipulating objects.
        :param sensors: Optional map of names to sensors (see e.g.,
            :class:`pyrobosim.sensors.lidar.Lidar2D`).
        :param start_sensor_threads: If True, automatically starts the sensor threads.
        :param partial_obs_objects: If False, the robot can access all objects in the world.
            If True, it must detect new objects at specific locations.
        :param partial_obs_hallways: If True, robot doesn't know the ground truth for the world's
            hallway states, and assumes all are open at the start unless sensed otherwise.
        :param action_execution_options: A dictionary of action names and their execution options.
            This defines properties such as delays and nondeterminism.
        :param initial_battery_level: The initial battery charge, from 0 to 100.
        """
        from .world import World

        # Basic properties
        super().__init__(name=name)
        self.radius = radius
        self.height = height
        self.color = parse_color(color)
        self.raw_polygon = Point(0, 0).buffer(radius)
        self.path_planner: PathPlanner | None = None
        self.path_executor: PathExecutor | None = None
        self.sensors: dict[str, Sensor] = {}

        if name == "world":
            raise ValueError("Robots cannot be named 'world'.")

        # Logger for this robot
        self.logger = create_logger(self.name)

        # Dynamics properties
        self.dynamics = RobotDynamics2D(
            init_pose=pose,
            max_linear_velocity=max_linear_velocity,
            max_angular_velocity=max_angular_velocity,
            max_linear_acceleration=max_linear_acceleration,
            max_angular_acceleration=max_angular_acceleration,
        )
        self.state_lock = Lock()

        # World interaction properties
        self.world: World | None = None
        self.location: Entity | None = None
        self.manipulated_object: Object | None = None
        self.partial_obs_objects = partial_obs_objects
        self.known_objects: set[Object] = set()
        self.last_detected_objects: list[Object] = []
        self.viz_text: Text | None = None
        self.partial_obs_hallways = partial_obs_hallways
        self.recorded_closed_hallways: set[Hallway] = set()

        # Polygons for collision checking
        self.total_internal_polygon = Polygon()

        # Sensing properties
        self.set_sensors(sensors)
        self.start_sensor_threads = start_sensor_threads
        if self.start_sensor_threads:
            self.do_start_sensor_threads()

        # Navigation properties
        self.executing_nav = False
        self.last_nav_result = ExecutionResult()
        self.set_path_planner(path_planner)
        self.set_path_executor(path_executor)

        # Manipulation properties
        self.grasp_generator = grasp_generator
        self.last_grasp_selection: Grasp | None = None

        # Action execution options
        self.action_execution_options = action_execution_options
        self.current_action: TaskAction | None = None
        self.executing_action = False
        self.current_plan: TaskPlan | None = None
        self.executing_plan = False
        self.canceling_execution = False
        self.initial_battery_level = initial_battery_level
        self.battery_level = initial_battery_level

        self.logger.info("Created robot.")

    def __del__(self) -> None:
        """Cleans up when deleting the robot instance."""
        self.stop_sensor_threads()

    def get_pose(self) -> Pose:
        """
        Gets the robot pose.

        :return: The robot pose.
        """
        return self.dynamics.pose

    def set_pose(self, pose: Pose) -> None:
        """
        Sets the robot pose.

        :param pose: New robot pose.
        """
        with self.state_lock:
            self.dynamics.pose = pose
            self.polygon = transform_polygon(self.raw_polygon, pose)

            if self.world:
                self.location = self.world.get_location_from_pose(
                    pose, prev_location=self.location
                )

    def set_path_planner(self, path_planner: PathPlanner | None) -> None:
        """
        Sets a path planner for navigation.

        :param path_planner: Path planner for navigation
            (see e.g., :class:`pyrobosim.navigation.rrt.RRTPlanner`).
        """
        self.path_planner = path_planner
        if path_planner is not None:
            path_planner.robot = self
            path_planner.reset()

    def set_path_executor(self, path_executor: PathExecutor | None) -> None:
        """
        Sets a path executor for navigation.

        :param path_executor: Path executor for navigation (see e.g.,
            :class:`pyrobosim.navigation.execution.ConstantVelocityExecutor`).
        """
        self.path_executor = path_executor
        if path_executor is not None:
            path_executor.robot = self
            if self.partial_obs_hallways:
                path_executor.validate_sensors_for_partial_obs_hallways()

    def set_sensors(self, sensors: dict[str, Sensor] | None) -> None:
        """
        Sets a list of sensors.

        :param sensors: Optional map of names to sensors (see e.g.,
            :class:`pyrobosim.sensors.lidar.Lidar2D`).
        """
        if sensors is None:
            return
        self.sensors = sensors
        for sensor in self.sensors.values():
            sensor.robot = self

    def do_start_sensor_threads(self) -> None:
        """Starts the robot's sensor threads."""
        for sensor in self.sensors.values():
            sensor.start_thread()

    def stop_sensor_threads(self) -> None:
        """Stops the robot's active sensor threads."""
        for sensor in self.sensors.values():
            sensor.stop_thread()
        if self.path_executor is not None:
            self.path_executor.cancel_all_threads = True

    def is_moving(self) -> bool:
        """
        Checks whether the robot is moving, either due to a navigation action or velocity commands.

        :return: True if the robot is moving, False otherwise.
        """
        return self.executing_nav or bool(np.count_nonzero(self.dynamics.velocity) > 0)

    def is_busy(self) -> bool:
        """
        Checks whether the robot is currently executing an action or plan.

        :return: True if the robot is busy, else False.
        """
        return self.executing_action or self.executing_plan or self.executing_nav

    def is_in_collision(self, pose: Pose | None = None) -> bool:
        """
        Checks whether the last step of dynamics put the robot in collision.

        :pose: Optional pose to use for collision checking.
            If not specified, uses the robot's current pose.
        :return: True if the robot is in collision, False otherwise.
        """
        if self.world is None:
            return False

        pose = pose or self.dynamics.pose
        return check_occupancy(
            pose, self.world, self
        ) or self.world.collides_with_robots(pose, robot=self)

    def at_object_spawn(self) -> bool:
        """
        Checks whether a robot is at an object spawn.

        :return: True if the robot is at an object spawn, False otherwise.
        """
        return isinstance(self.location, ObjectSpawn)

    def get_known_objects(self) -> list[Object]:
        """
        Returns a list of objects known by the robot.

        :return: The list of known objects.
        """
        if self.world is None:
            return []

        if self.partial_obs_objects:
            return list(self.known_objects)

        return self.world.objects

    def at_openable_location(self) -> bool:
        """
        Checks whether the robot is at an openable location.

        :return: True if the robot is at an openable location, else False.
        """
        return isinstance(self.location, (Hallway, ObjectSpawn))

    def get_known_closed_hallways(self) -> list[Hallway]:
        """
        Returns a list of closed hallway recorded by the robot.

        :return: The list of recorded closed hallways.
        """
        if self.world is None:
            return []

        if self.partial_obs_hallways:
            return list(self.recorded_closed_hallways)

        return [hall for hall in self.world.hallways if not hall.is_open]

    def _attach_object(self, obj: Object) -> None:
        """
        Helper function to attach an object in the world to the robot.
        Be careful calling this function directly as it does not do any validation.
        When possible, you should be using `pick_object`.

        :param obj: Object to attach.
        """
        self.manipulated_object = obj
        set_parent(obj, self)
        obj.set_pose(self.get_pose())

    def plan_path(
        self, start: Pose | None = None, goal: Pose | str | None = None
    ) -> Path | None:
        """
        Plans a path to a goal position.

        :param start: Start pose for the robot.
            If not specified, will default to the robot pose.
        :param goal: Goal pose or entity name for the robot.
            If not specified, returns None.
        :return: The path, if one was found, otherwise None.
        """
        from ..utils.knowledge import graph_node_from_entity, query_to_entity

        if self.path_planner is None:
            self.logger.warning(f"No path planner attached to robot.")
            return None

        if start is None:
            start = self.get_pose()

        if goal is None:
            self.logger.warning("Did not specify a goal. Returning None.")
            return None

        # If the goal is not a pose, we need to extract it from the world knowledge.
        if not isinstance(goal, Pose):
            if self.world is None:
                self.logger.warning(
                    "Cannot specify a string goal if there is no world set."
                )
                return None

            if isinstance(goal, str):
                query_list = [elem for elem in goal.split(" ") if elem]
                entity = query_to_entity(
                    self.world,
                    query_list,
                    mode="location",
                    robot=self,
                    resolution_strategy="nearest",
                )
                if entity is None:
                    self.logger.warning(
                        f"Could not resolve goal location query: {query_list}"
                    )
                    return None

            goal_node = graph_node_from_entity(self.world, entity, robot=self)
            if goal_node is None:
                self.logger.warning(f"Could not find graph node associated with goal.")
                return None
            goal = goal_node.pose

        path = self.path_planner.plan(start, goal)
        if (self.world is not None) and (self.world.gui is not None):
            show_graphs = True
            self.world.gui.canvas.show_planner_and_path_signal.emit(
                self, show_graphs, path
            )
        return path

    def follow_path(
        self,
        path: Path,
        realtime_factor: float = 1.0,
    ) -> ExecutionResult:
        """
        Follows a specified path using the attached path executor.

        :param path: The path to follow.
        :param realtime_factor: A multiplier on the execution time relative to
            real time. Defaults to 1.0. If negative, runs as quickly as possible.
        :return: An object describing the execution result.
        """
        self.last_nav_result = ExecutionResult()

        if path is None or path.num_poses == 0:
            self.executing_nav = False
            message = "No path to execute."
            self.logger.warning(message)
            return ExecutionResult(
                status=ExecutionStatus.PRECONDITION_FAILURE,
                message=message,
            )
        if self.path_executor is None:
            self.executing_nav = False
            message = "No path executor. Cannot follow path."
            self.logger.warning(message)
            return ExecutionResult(
                status=ExecutionStatus.PRECONDITION_FAILURE,
                message=message,
            )
        if self.path_executor.following_path:
            message = "Robot is already following an existing path."
            self.logger.warning(message)
            return ExecutionResult(
                status=ExecutionStatus.PRECONDITION_FAILURE,
                message=message,
            )

        # Follow the path.
        self.executing_nav = True
        exec_options = self.action_execution_options.get("navigate")
        battery_usage = exec_options.battery_usage if exec_options else 0.0
        result = self.path_executor.execute(
            path, realtime_factor=realtime_factor, battery_usage=battery_usage
        )

        # Check that the robot made it to its goal pose at the end of execution.
        at_goal_pose = self.get_pose().is_approx(path.poses[-1])
        if result.is_success() and not at_goal_pose:
            result = ExecutionResult(
                status=ExecutionStatus.POSTCONDITION_FAILURE,
                message="Robot is not at its intended target pose.",
            )

        # Update the robot state.
        self.last_nav_result = result
        self.executing_nav = False
        if self.world:
            if (
                isinstance(self.location, ObjectSpawn)
                and (self.location.parent is not None)
                and self.location.parent.is_charger
            ):
                self.logger.info(f"Battery charged at {self.location.name}!")
                self.battery_level = 100.0

            if self.world.gui is not None:
                self.world.gui.canvas.show_world_state(robot=self)
                self.world.gui.update_buttons_signal.emit()
        return result

    def navigate(
        self,
        start: Pose | None = None,
        goal: Pose | str | None = None,
        path: Path | None = None,
        realtime_factor: float = 1.0,
    ) -> ExecutionResult:
        """
        Executes a navigation task, which combines path planning and following.

        :param start: Start pose for the robot.
            If not specified, will default to the robot pose.
        :param goal: Goal pose or entity name for the robot.
            If not specified, returns None.
        :param path: The path to follow.
        :param realtime_factor: A multiplier on the execution time relative to
            real time. Defaults to 1.0. If negative, runs as quickly as possible.
        :return: An object describing the execution result.
        """
        if self.battery_level <= 0.0:
            message = "Out of battery. Cannot navigate."
            self.logger.warning(message)
            self.last_nav_result = ExecutionResult(
                status=ExecutionStatus.PRECONDITION_FAILURE, message=message
            )
            self.executing_nav = False
            return self.last_nav_result

        if path is None:
            path = self.plan_path(start, goal)
            if path is None or path.num_poses == 0:
                message = "Failed to plan a path."
                self.logger.warning(message)
                self.last_nav_result = ExecutionResult(
                    status=ExecutionStatus.PLANNING_FAILURE,
                    message=message,
                )
                self.executing_nav = False
                return self.last_nav_result
        elif (self.world is not None) and (self.world.gui is not None):
            show_graphs = False
            self.world.gui.canvas.show_planner_and_path_signal.emit(
                self, show_graphs, path
            )

        # Simulate execution options.
        exec_options = self.action_execution_options.get("navigate")
        if exec_options:
            if not exec_options.should_succeed():
                message = "Simulated navigation failure."
                self.logger.info(message)
                self.last_nav_result = ExecutionResult(
                    status=ExecutionStatus.EXECUTION_FAILURE, message=message
                )
                self.executing_nav = False
                return self.last_nav_result

        return self.follow_path(path, realtime_factor=realtime_factor)

    def reset_path_planner(self) -> None:
        """Resets the robot's path planner, if available."""
        if self.path_planner is None:
            self.logger.warning("Robot has no path planner to reset.")
            return

        if not hasattr(self.path_planner, "reset"):
            self.logger.warning(
                "Path planner does not have a reset() method. Cannot reset."
            )
            return

        self.path_planner.reset()
        if (self.world is not None) and (self.world.gui is not None):
            show_graphs = True
            path = None
            self.world.gui.canvas.show_planner_and_path_signal.emit(
                self, show_graphs, path
            )

    def pick_object(
        self, obj_query: str | None, grasp_pose: Pose | None = None
    ) -> ExecutionResult:
        """
        Picks up an object in the world given an object and/or location query.

        :param obj_query: The object query (name, category, etc.).
        :param grasp_pose: A pose describing how to manipulate the object.
        :return: An object describing the execution result.
        """
        # Validate input
        if self.manipulated_object is not None:
            obj_name = self.manipulated_object.name
            message = f"Robot is already holding {obj_name}."
            self.logger.warning(message)
            return ExecutionResult(
                status=ExecutionStatus.PRECONDITION_FAILURE, message=message
            )

        # Get object
        loc = self.location
        if isinstance(obj_query, Object):
            obj: Object | None = obj_query
        if self.world is None:
            message = f"Robot does not have a world attached to it."
            self.logger.error(message)
            return ExecutionResult(
                status=ExecutionStatus.INVALID_ACTION, message=message
            )

        if isinstance(self.location, str):
            loc = self.world.get_entity_by_name(self.location)
        elif isinstance(obj_query, str):
            if obj_query is not None:
                obj = self.world.get_object_by_name(obj_query)

            if obj is None:
                from ..utils.knowledge import query_to_entity

                entity = query_to_entity(
                    self.world,
                    obj_query,
                    mode="object",
                    robot=self,
                    resolution_strategy="nearest",
                )
                assert (entity is None) or (isinstance(entity, Object))
                obj = entity

        if obj is None:
            message = f"Found no object {obj_query} to pick."
            self.logger.warning(message)
            return ExecutionResult(
                status=ExecutionStatus.PRECONDITION_FAILURE, message=message
            )

        # Validate the robot location
        if (obj.parent is not None) and (loc is not None) and (obj.parent != loc):
            message = f"{obj.name} is at {obj.parent.name} and robot is at {loc.name}. Cannot pick."
            self.logger.warning(message)
            return ExecutionResult(
                status=ExecutionStatus.PRECONDITION_FAILURE, message=message
            )
        if (loc is not None) and (loc.parent is not None) and (not loc.is_open):
            message = f"{loc.parent.name} is not open. Cannot pick object."
            self.logger.warning(message)
            return ExecutionResult(
                status=ExecutionStatus.PRECONDITION_FAILURE, message=message
            )
        if self.battery_level <= 0.0:
            message = "Out of battery. Cannot pick."
            self.logger.warning(message)
            return ExecutionResult(
                status=ExecutionStatus.PRECONDITION_FAILURE, message=message
            )

        # If a grasp generator has been specified and no explicit grasp has been provided,
        # generate grasps here.
        # TODO: Specify allowed grasp types
        if self.grasp_generator is not None:
            if grasp_pose is not None:
                self.last_grasp_selection = Grasp(
                    properties=self.grasp_generator.properties,
                    origin_wrt_object=Pose(),
                    origin_wrt_world=grasp_pose,
                )
            else:
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
                    message = "Could not generate valid grasps. Cannot pick object."
                    self.logger.warning(message)
                    return ExecutionResult(
                        status=ExecutionStatus.PLANNING_FAILURE, message=message
                    )
                # TODO: For now, just pick a random grasp.
                self.last_grasp_selection = np.random.choice(grasps)

        if self.last_grasp_selection is not None:
            self.logger.info(f"Selected {self.last_grasp_selection}")

        # Simulate execution options.
        exec_options = self.action_execution_options.get("pick")
        if exec_options:
            self.battery_level = max(
                0.0, self.battery_level - exec_options.battery_usage
            )
            if not exec_options.should_succeed():
                message = "Simulated pick failure."
                self.logger.info(message)
                return ExecutionResult(
                    status=ExecutionStatus.EXECUTION_FAILURE, message=message
                )

        # Denote the target object as the manipulated object
        self._attach_object(obj)
        if self.world.gui is not None:
            self.world.gui.canvas.update_object_plot(self.manipulated_object)
            self.world.gui.canvas.show_world_state(self)
            self.world.gui.update_buttons_signal.emit()
        return ExecutionResult(status=ExecutionStatus.SUCCESS)

    def place_object(self, pose: Pose | None = None) -> ExecutionResult:
        """
        Places an object in a target location and (optionally) pose.

        :param pose: Placement pose (if not specified, will be sampled).
        :return: An object describing the execution result.
        """
        # Validate input
        if self.manipulated_object is None:
            message = "No manipulated object. Cannot place."
            self.logger.warning(message)
            return ExecutionResult(
                status=ExecutionStatus.PRECONDITION_FAILURE, message=message
            )

        # Validate the robot location
        loc = self.location
        if isinstance(loc, str):
            loc = self.world.get_entity_by_name(self.location)
        if not isinstance(loc, ObjectSpawn):
            message = f"{loc} is not an object spawn. Cannot place object."
            self.logger.warning(message)
            return ExecutionResult(
                status=ExecutionStatus.PRECONDITION_FAILURE, message=message
            )
        if (loc.parent is not None) and (not loc.is_open):
            message = f"{loc.parent.name} is not open. Cannot place object."
            self.logger.warning(message)
            return ExecutionResult(
                status=ExecutionStatus.PRECONDITION_FAILURE, message=message
            )
        if self.battery_level <= 0.0:
            message = "Out of battery. Cannot place."
            self.logger.warning(message)
            return ExecutionResult(
                status=ExecutionStatus.PRECONDITION_FAILURE, message=message
            )

        # Place the object somewhere in the current location.
        is_valid_pose = False
        poly = self.manipulated_object.raw_collision_polygon
        if pose is None:
            # If no pose was specified, sample one.
            max_tries = self.world.max_object_sample_tries if self.world else 1000
            for _ in range(max_tries):
                x_sample, y_sample = sample_from_polygon(loc.polygon)
                if (x_sample is None) or (y_sample is None):
                    continue
                yaw_sample = np.random.uniform(-np.pi, np.pi)
                pose_sample = Pose(x=x_sample, y=y_sample, yaw=yaw_sample)
                sample_poly = transform_polygon(poly, pose_sample)
                is_valid_pose = sample_poly.within(loc.polygon)
                for other_obj in loc.children:
                    assert isinstance(other_obj, Object)
                    is_valid_pose = is_valid_pose and not sample_poly.intersects(
                        other_obj.collision_polygon
                    )
                if is_valid_pose:
                    pose = pose_sample
                    break
            if not is_valid_pose:
                message = f"Could not sample a placement position at {loc.name}"
                self.logger.warning(message)
                return ExecutionResult(
                    status=ExecutionStatus.PLANNING_FAILURE, message=message
                )
        else:
            # If a pose was specified, collision check it
            poly = transform_polygon(poly, pose)
            is_valid_pose = poly.within(loc.polygon)
            for other_obj in loc.children:
                assert isinstance(other_obj, Object)
                is_valid_pose = is_valid_pose and not poly.intersects(
                    other_obj.collision_polygon
                )
            if not is_valid_pose:
                message = f"Pose in collision or not in location {loc.name}."
                self.logger.warning(message)
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
                message = "Simulated place failure."
                self.logger.info(message)
                return ExecutionResult(
                    status=ExecutionStatus.EXECUTION_FAILURE, message=message
                )

        obj = self.manipulated_object
        if (self.world is not None) and (self.world.gui is not None):
            gui_patches = self.world.gui.canvas.obj_patches
            if (obj.viz_patch is not None) and (obj.viz_patch in gui_patches):
                gui_patches.remove(obj.viz_patch)
                obj.viz_patch.remove()

        assert pose is not None
        set_parent(self.manipulated_object, loc)
        self.manipulated_object.set_pose(pose)
        self.manipulated_object.create_polygons()

        if (self.world is not None) and (self.world.gui is not None):
            self.world.gui.canvas.axes.add_patch(obj.viz_patch)
            self.world.gui.canvas.obj_patches.append(obj.viz_patch)
            self.world.gui.canvas.update_object_plot(obj)
            self.world.gui.canvas.show_world_state(self)
            self.world.gui.update_buttons_signal.emit()

        self.manipulated_object = None
        return ExecutionResult(status=ExecutionStatus.SUCCESS)

    def detect_objects(self, target_object: str | None = None) -> ExecutionResult:
        """
        Detects all objects at the robot's current location.

        :param target_object: The name of a target object or category.
            If None, the action succeeds regardless of which object is found.
            Otherwise, the action succeeds only if the target object is found.
        :return: An object describing the execution result.
        """
        self.last_detected_objects = []

        if (self.location is None) or (not self.at_object_spawn()):
            message = "Robot is not at an object spawn. Cannot detect objects."
            self.logger.warning(message)
            return ExecutionResult(
                status=ExecutionStatus.PRECONDITION_FAILURE, message=message
            )
        if (
            (self.location is not None)
            and (self.location.parent is not None)
            and (not self.location.is_open)
        ):
            message = f"{self.location.parent.name} is not open. Cannot detect objects."
            self.logger.warning(message)
            return ExecutionResult(
                status=ExecutionStatus.PRECONDITION_FAILURE, message=message
            )

        if self.battery_level <= 0.0:
            message = "Out of battery. Cannot detect objects."
            self.logger.warning(message)
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
                message = "Simulated detection failure."
                self.logger.info(message)
                return ExecutionResult(
                    status=ExecutionStatus.EXECUTION_FAILURE, message=message
                )

        # Add all the objects at the current robot's location.
        for obj in self.location.children:
            assert isinstance(obj, Object)
            self.known_objects.add(obj)

        # If a target object was specified, look for a matching instance.
        # We should only return SUCCESS if one such instance was found.
        if (self.world is not None) and (self.world.gui is not None):
            self.world.gui.canvas.show_objects()
            self.world.gui.update_buttons_signal.emit()
        if not target_object:  # Checking for empty string and None
            self.last_detected_objects = [
                obj for obj in self.location.children if isinstance(obj, Object)
            ]
            return ExecutionResult(status=ExecutionStatus.SUCCESS)
        else:
            self.last_detected_objects = [
                obj
                for obj in self.location.children
                if isinstance(obj, Object)
                and ((obj.name == target_object) or (obj.category == target_object))
            ]
            if len(self.last_detected_objects) > 0:
                return ExecutionResult(status=ExecutionStatus.SUCCESS)
            else:
                return ExecutionResult(
                    status=ExecutionStatus.EXECUTION_FAILURE,
                    message=f"Failed to detect any objects matching the query '{target_object}'.",
                )

    def open_location(self) -> ExecutionResult:
        """
        Opens the robot's current location, if available.

        :return: An object describing the execution result.
        """
        if self.world is None:
            message = "Robot world is not set. Cannot open."
            self.logger.warning(message)
            return ExecutionResult(
                status=ExecutionStatus.PRECONDITION_FAILURE, message=message
            )

        if self.location is None:
            message = "Robot location is not set. Cannot open."
            self.logger.warning(message)
            return ExecutionResult(
                status=ExecutionStatus.PRECONDITION_FAILURE, message=message
            )

        if self.manipulated_object is not None:
            message = "Robot is holding an object. Cannot open."
            self.logger.warning(message)
            return ExecutionResult(
                status=ExecutionStatus.PRECONDITION_FAILURE, message=message
            )

        if not self.at_openable_location():
            message = "Robot is not at an openable location."
            self.logger.warning(message)
            return ExecutionResult(
                status=ExecutionStatus.PRECONDITION_FAILURE, message=message
            )

        if self.battery_level <= 0.0:
            message = "Out of battery. Cannot open location."
            self.logger.warning(message)
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
                message = "Simulated opening failure."
                self.logger.info(message)
                return ExecutionResult(
                    status=ExecutionStatus.EXECUTION_FAILURE, message=message
                )

        # Update recorded_closed_hallways knowledge
        if self.partial_obs_hallways:
            if isinstance(self.location, Hallway) and (
                self.location in self.recorded_closed_hallways
            ):
                self.recorded_closed_hallways.remove(self.location)
                self.logger.info(f"Removed {self.location.name} from closed knowledge.")

        if isinstance(self.location, ObjectSpawn):
            loc_to_open = self.location.parent
        else:
            loc_to_open = self.location
        return self.world.open_location(loc_to_open)

    def close_location(self) -> ExecutionResult:
        """
        Closes the robot's current location, if available.

        :return: An object describing the execution result.
        """
        if self.world is None:
            message = "Robot world is not set. Cannot close."
            self.logger.warning(message)
            return ExecutionResult(
                status=ExecutionStatus.PRECONDITION_FAILURE, message=message
            )

        if self.location is None:
            message = "Robot location is not set. Cannot close."
            self.logger.warning(message)
            return ExecutionResult(
                status=ExecutionStatus.PRECONDITION_FAILURE, message=message
            )

        if self.manipulated_object is not None:
            message = "Robot is holding an object. Cannot close."
            self.logger.warning(message)
            return ExecutionResult(
                status=ExecutionStatus.PRECONDITION_FAILURE, message=message
            )

        if not self.at_openable_location():
            message = "Robot is not at a closeable location."
            self.logger.warning(message)
            return ExecutionResult(
                status=ExecutionStatus.PRECONDITION_FAILURE, message=message
            )

        if self.battery_level <= 0.0:
            message = "Out of battery. Cannot close location."
            self.logger.warning(message)
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
                message = "Simulated closing failure."
                self.logger.info(message)
                return ExecutionResult(
                    status=ExecutionStatus.EXECUTION_FAILURE, message=message
                )

        # Update recorded_closed_hallways knowledge
        if self.partial_obs_hallways:
            if isinstance(self.location, Hallway) and (
                self.location not in self.recorded_closed_hallways
            ):
                self.recorded_closed_hallways.add(self.location)
                self.logger.info(f"Added {self.location.name} to closed knowledge.")

        if isinstance(self.location, ObjectSpawn):
            loc_to_close = self.location.parent
        else:
            loc_to_close = self.location
        return self.world.close_location(loc_to_close, ignore_robots=[self])

    def execute_action(
        self, action: TaskAction, realtime_factor: float = 1.0
    ) -> ExecutionResult:
        """
        Executes an action, specified as a
        :class:`pyrobosim.planning.actions.TaskAction` object.

        :param action: Action to execute.
        :param realtime_factor: A multiplier on the execution time relative to
            real time. Defaults to 1.0. If negative, runs as quickly as possible.
        :return: An object describing the execution result.
        """
        self.executing_action = True
        self.current_action = action
        if (self.world is not None) and (self.world.gui is not None):
            self.world.gui.set_buttons_during_action(False)

        if action.type == "navigate":
            self.executing_nav = True
            path = action.path if action.path.num_poses > 0 else None
            if (self.world is not None) and (self.world.gui is not None):
                if action.target_location and not isinstance(
                    action.target_location, str
                ):
                    target_location_name = action.target_location.name
                else:
                    target_location_name = action.target_location

                self.world.gui.canvas.navigate_signal.emit(
                    self,
                    target_location_name,
                    path,
                    realtime_factor,
                )
                while self.executing_nav:
                    time.sleep(0.25)  # Delay to wait for navigation
                result = self.last_nav_result
            else:
                result = self.navigate(
                    goal=action.target_location,
                    path=path,
                    realtime_factor=realtime_factor,
                )

        elif action.type == "pick":
            result = self.pick_object(action.object, action.pose)

        elif action.type == "place":
            result = self.place_object(action.pose)

        elif action.type == "detect":
            result = self.detect_objects(action.object)

        elif action.type == "open":
            result = self.open_location()

        elif action.type == "close":
            result = self.close_location()

        else:
            message = f"Invalid action type: {action.type}."
            self.logger.warning(message)
            result = ExecutionResult(
                status=ExecutionStatus.INVALID_ACTION, message=message
            )

        self.logger.info(f"Action completed with result: {result.status.name}")
        self.current_action = None
        self.executing_action = False
        return result

    def cancel_actions(self) -> None:
        """Cancels any currently running actions for the robot."""
        if not (self.executing_action or self.executing_plan or self.executing_nav):
            self.logger.warning("There is no running action or plan to cancel.")
            return

        if self.executing_nav and self.path_executor is not None:
            # Stop path executor
            self.logger.info("Canceling path execution...")
            self.path_executor.cancel_execution = True
            while self.executing_nav:
                time.sleep(0.1)

        if self.executing_action:
            # Wait for execute_action to return
            while self.executing_action:
                time.sleep(0.1)

        if self.executing_plan:
            self.canceling_execution = True
            # Wait for execute_plan to return
            while self.executing_plan:
                time.sleep(0.1)

    def execute_plan(
        self,
        plan: TaskPlan,
        delay: float = 0.5,
        realtime_factor: float = 1.0,
    ) -> tuple[ExecutionResult, int]:
        """
        Executes a task plan, specified as a
        :class:`pyrobosim.planning.actions.TaskPlan` object.

        :param plan: Task plan to execute.
        :param delay: Artificial delay between actions for visualization.
        :param realtime_factor: A multiplier on the execution time relative to
            real time. Defaults to 1.0. If negative, runs as quickly as possible.
        :return: A tuple containing an execution result and the number of actions completed.
        """
        if plan is None:
            message = "Plan is None. Returning."
            self.logger.warning(message)
            return (
                ExecutionResult(status=ExecutionStatus.INVALID_ACTION, message=message),
                0,
            )

        self.executing_plan = True
        self.current_plan = plan

        self.logger.info("Executing task plan...")
        if (self.world is not None) and (self.world.gui is not None):
            self.world.gui.set_buttons_during_action(False)

        result = ExecutionResult(status=ExecutionStatus.SUCCESS)
        num_completed = 0
        num_acts = len(plan.actions)
        for n, act_msg in enumerate(plan.actions):
            if self.canceling_execution:
                self.canceling_execution = False
                message = "Canceled plan execution."
                self.logger.info(message)
                result = ExecutionResult(
                    status=ExecutionStatus.CANCELED, message=message
                )
                break

            self.logger.info(f"Executing action {act_msg.type} [{n+1}/{num_acts}]")
            result = self.execute_action(act_msg, realtime_factor=realtime_factor)
            if not result.is_success():
                self.logger.info(
                    f"Task plan failed to execute on action {n+1}/{num_acts}"
                )
                break
            num_completed += 1
            time.sleep(delay)  # Artificial delay between actions

        self.logger.info(f"Task plan completed with status: {result.status.name}")
        self.canceling_execution = False
        self.executing_plan = False
        self.current_plan = None
        return result, num_completed

    def update_polygons(self) -> None:
        """
        Updates the world's collision polygons when hallway state changes.
        """
        if self.world is None:
            return

        if self.partial_obs_hallways:
            # closed_hallways would be obtained from robot knowledge
            closed_hallways = self.get_known_closed_hallways()

            self.total_internal_polygon = unary_union(
                [
                    entity.internal_collision_polygon
                    for entity in itertools.chain(self.world.rooms, self.world.hallways)
                ]
            ).difference(
                unary_union([hall.inflated_closed_polygon for hall in closed_hallways])
            )

            shapely.prepare(self.total_internal_polygon)

        else:
            self.total_internal_polygon = self.world.total_internal_polygon

    def to_dict(self) -> dict[str, Any]:
        """
        Serializes the robot to a dictionary.

        :return: A dictionary containing the robot information.
        """
        pose = self.get_pose()

        robot_dict = {
            "name": self.name,
            "radius": self.radius,
            "height": self.height,
            "color": self.color,
            "pose": pose.to_dict(),
            "max_linear_velocity": float(self.dynamics.vel_limits[0]),
            "max_angular_velocity": float(self.dynamics.vel_limits[-1]),
            "max_linear_acceleration": float(self.dynamics.accel_limits[0]),
            "max_angular_acceleration": float(self.dynamics.accel_limits[-1]),
            "partial_obs_objects": self.partial_obs_objects,
            "partial_obs_hallways": self.partial_obs_hallways,
            "initial_battery_level": self.initial_battery_level,
            "start_sensor_threads": self.start_sensor_threads,
        }

        if self.world is not None:
            location = self.world.get_location_from_pose(pose)
            if location is not None:
                robot_dict["location"] = location.name
        if self.path_planner is not None:
            robot_dict["path_planner"] = self.path_planner.to_dict()
        if self.path_executor is not None:
            robot_dict["path_executor"] = self.path_executor.to_dict()
        if self.grasp_generator is not None:
            robot_dict["grasping"] = self.grasp_generator.to_dict()
        if len(self.sensors) > 0:
            robot_dict["sensors"] = {
                name: sensor.to_dict() for name, sensor in self.sensors.items()
            }
        if len(self.action_execution_options) > 0:
            robot_dict["action_execution_options"] = {
                name: action.to_dict()
                for name, action in self.action_execution_options.items()
            }

        return robot_dict

    def __repr__(self) -> str:
        """Returns printable string."""
        return f"Robot: {self.name}"

    def print_details(self) -> None:
        """Prints string with details."""
        details_str = f"Robot: {self.name}"
        details_str += f"\n\t{self.get_pose()}"
        details_str += f"\n\tBattery: {self.battery_level:.2f}%"
        if self.partial_obs_objects:
            details_str += "\n\tPartial object observability enabled"
        if self.partial_obs_hallways:
            details_str += "\n\tPartial hallway observability enabled"
        print(details_str)
