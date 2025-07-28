"""Utilities for executing actions from the UI in separate QThreads."""

from PySide6.QtCore import QRunnable

from ..core.robot import Robot
from ..core.world import World
from ..utils.path import Path
from ..utils.pose import Pose


class NavRunner(QRunnable):  # type: ignore[misc]
    """
    Helper class that wraps navigation execution in a QThread.
    """

    def __init__(
        self,
        world: World,
        robot: Robot | str | None,
        goal: str,
        path: Path | None,
        realtime_factor: float = 1.0,
    ) -> None:
        """
        Creates a navigation execution thread.

        :param world: A world object linked to this thread.
        :param robot: Robot instance or name to execute action.
        :param goal: Name of goal location (resolved by the world model).
        :param path: Path to goal location.
        :param realtime_factor: A multiplier on the execution time relative to
            real time. Defaults to 1.0. If negative, runs as quickly as possible.
        """
        super(NavRunner, self).__init__()
        self.world = world
        self.robot = robot
        self.goal = goal
        self.path = path
        self.realtime_factor = realtime_factor

    def run(self) -> None:
        """Runs the navigation execution thread."""
        robot = self.robot
        world = self.world

        if isinstance(robot, str):
            robot = world.get_robot_by_name(robot)
        if robot is None:
            self.world.logger.warning("Robot is not specified. Cannot navigate.")
            return

        robot.navigate(
            goal=self.goal,
            path=self.path,
            realtime_factor=self.realtime_factor,
        )


class PickRunner(QRunnable):  # type: ignore[misc]
    """
    Helper class that wraps object picking execution in a QThread.
    """

    def __init__(
        self,
        world: World,
        robot: Robot | str | None,
        obj_query: str,
        grasp_pose: Pose | None,
    ) -> None:
        """
        Creates an object picking execution thread.

        :param world: A world object linked to this thread.
        :param robot: Robot instance or name to execute action.
        :param obj_query: The object query (name, category, etc.).
        :param grasp_pose: A pose describing how to manipulate the object.
        """
        super(PickRunner, self).__init__()
        self.world = world
        self.robot = robot
        self.obj_query = obj_query
        self.grasp_pose = grasp_pose

    def run(self) -> None:
        """Runs the object picking execution thread."""
        robot = self.robot
        world = self.world

        if isinstance(robot, str):
            robot = world.get_robot_by_name(robot)
        if robot is None:
            world.logger.warning("Robot is not specified. Cannot pick objects.")
            return

        robot.pick_object(self.obj_query, self.grasp_pose)


class PlaceRunner(QRunnable):  # type: ignore[misc]
    """
    Helper class that wraps object placement execution in a QThread.
    """

    def __init__(
        self, world: World, robot: Robot | str | None, pose: Pose | None
    ) -> None:
        """
        Creates an object placement execution thread.

        :param world: A world object linked to this thread.
        :param robot: Robot instance or name to execute action.
        :param pose: Optional pose describing how to place the object.
        """
        super(PlaceRunner, self).__init__()
        self.world = world
        self.robot = robot
        self.pose = pose

    def run(self) -> None:
        """Runs the object picking execution thread."""
        robot = self.robot
        world = self.world

        if isinstance(robot, str):
            robot = world.get_robot_by_name(robot)
        if robot is None:
            world.logger.warning("Robot is not specified. Cannot place objects.")
            return

        robot.place_object(pose=self.pose)


class DetectRunner(QRunnable):  # type: ignore[misc]
    """
    Helper class that wraps object detection execution in a QThread.
    """

    def __init__(
        self, world: World, robot: Robot | str | None, query: str | None
    ) -> None:
        """
        Creates an object detection execution thread.

        :param world: A world object linked to this thread.
        :param robot: Robot instance or name to execute action.
        :param query: Query for object detection.
        """
        super(DetectRunner, self).__init__()
        self.world = world
        self.robot = robot
        self.query = query

    def run(self) -> None:
        """Runs the object detection execution thread."""
        robot = self.robot
        world = self.world

        if isinstance(robot, str):
            robot = world.get_robot_by_name(robot)
        if robot is None:
            world.logger.warning("Robot is not specified. Cannot detect objects.")
            return

        robot.detect_objects(self.query)


class OpenRunner(QRunnable):  # type: ignore[misc]
    """
    Helper class that wraps location opening execution in a QThread.
    """

    def __init__(self, world: World, robot: Robot | str | None) -> None:
        """
        Creates a location opening execution thread.

        :param world: A world object linked to this thread.
        :param robot: Robot instance or name to execute action.
        """
        super(OpenRunner, self).__init__()
        self.world = world
        self.robot = robot

    def run(self) -> None:
        """Runs the location opening execution thread."""
        robot = self.robot
        world = self.world

        if isinstance(robot, str):
            robot = world.get_robot_by_name(robot)
        if robot is None:
            world.logger.warning("Robot is not specified. Cannot open locations.")
            return

        robot.open_location()


class CloseRunner(QRunnable):  # type: ignore[misc]
    """
    Helper class that wraps location closing execution in a QThread.
    """

    def __init__(self, world: World, robot: Robot | str | None) -> None:
        """
        Creates a location closing execution thread.

        :param world: A world object linked to this thread.
        :param robot: Robot instance or name to execute action.
        """
        super(CloseRunner, self).__init__()
        self.world = world
        self.robot = robot

    def run(self) -> None:
        """Runs the location closing execution thread."""
        robot = self.robot
        world = self.world

        if isinstance(robot, str):
            robot = world.get_robot_by_name(robot)
        if robot is None:
            world.logger.warning("Robot is not specified. Cannot close locations.")
            return

        robot.close_location()
