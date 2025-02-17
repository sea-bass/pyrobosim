"""
Basic types for path planning and navigation.
"""

from typing import Any

from ..planning.actions import ExecutionResult
from ..utils.path import Path
from ..utils.pose import Pose


class PathPlanner:
    """
    Generic path planner class that helps with type hinting.

    When implementing a new planner, you should subclass from this class.
    """

    def plan(self, start: Pose, goal: Pose) -> Path:
        """
        Plans a path from start to goal.

        :param start: Start pose.
        :param goal: Goal pose.
        :return: Path from start to goal.
        """
        raise NotImplementedError("Must implement in subclass.")

    def reset(self) -> None:
        """Resets the path planner."""
        raise NotImplementedError("Must implement in subclass.")

    def to_dict(self) -> dict[str, Any]:
        """
        Serializes the planner to a dictionary.

        :return: A dictionary containing the planner information.
        """
        raise NotImplementedError("Must implement in subclass.")


class PathExecutor:
    """
    Generic path executor class that helps with type hinting.

    When implementing a new executor, you should subclass from this class.
    """

    following_path = False
    """Flag to track path following."""

    cancel_execution = False
    """Flag to cancel from user."""

    def __init__(self) -> None:
        from ..core.robot import Robot

        self.robot: Robot | None = None

    def execute(
        self, path: Path, realtime_factor: float = 1.0, battery_usage: float = 0.0
    ) -> ExecutionResult:
        """
        Generates and executes a trajectory on the robot.

        :param path: Path to execute on the robot.
        :param realtime_factor: A multiplier on the execution time relative to
            real time, defaults to 1.0.
        :param battery_usage: Robot battery usage per unit distance.
        :return: An object describing the execution result.
        """
        raise NotImplementedError("Must implement in subclass.")

    def to_dict(self) -> dict[str, Any]:
        """
        Serializes the executor to a dictionary.

        :return: A dictionary containing the executor information.
        """
        raise NotImplementedError("Must implement in subclass.")
