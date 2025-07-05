"""Defines actions for task and motion planning."""

from enum import IntEnum
from typing import Any

import numpy as np
import time

from ..utils.path import Path
from ..utils.pose import Pose


class ExecutionOptions:
    """Options for executing actions in simulation."""

    def __init__(
        self,
        delay: float = 0.0,
        success_probability: float = 1.0,
        rng_seed: int | None = None,
        battery_usage: float = 0.0,
    ) -> None:
        """
        Creates a new set of action execution options.

        :param delay: The simulated delay time, in seconds.
        :param success_probability: The simulated success probability, in the range (0, 1).
        :param rng_seed: The seed to use for random number generation.
            Defaults to None, but can be changed for determinism.
        :param battery_usage: Amount of battery reduction from running the action.
            Note that some actions apply this as a fixed reductions, and others apply it
            per some unit of measure (for example, battery per distance moved).
            Must be greater than 0.
        """
        self.delay = delay
        self.success_probability = success_probability
        self.rng_seed = rng_seed
        self.rng = np.random.default_rng(seed=rng_seed)
        self.battery_usage = battery_usage

    def should_succeed(self) -> bool:
        """
        Determines whether the action should succeed, while simulating other aspects such as delays.

        :return: True if the action should succeed, or False otherwise.
        """
        time.sleep(self.delay)
        return float(self.rng.random()) <= self.success_probability

    def to_dict(self) -> dict[str, Any]:
        """
        Serializes the execution options to a dictionary.

        :return: A dictionary containing the execution options information.
        """
        options_dict = {
            "delay": self.delay,
            "success_probability": self.success_probability,
            "battery_usage": self.battery_usage,
        }

        if self.rng_seed is not None:
            options_dict["rng_seed"] = self.rng_seed

        return options_dict


class ExecutionStatus(IntEnum):
    """Enumeration for action or plan execution status."""

    UNKNOWN = -1

    # Action executed successfully.
    SUCCESS = 0

    # Preconditions not sufficient to execute the action.
    # For example, the action was to pick an object but there was no object visible.
    PRECONDITION_FAILURE = 1

    # Planning failed, for example a path planner or grasp planner did not produce a solution.
    PLANNING_FAILURE = 2

    # Preconditions were met and planning succeeded, but execution failed.
    EXECUTION_FAILURE = 3

    # Execution succeeded, but post-execution validation failed.
    POSTCONDITION_FAILURE = 4

    # Invalid action type.
    INVALID_ACTION = 5

    # The action was canceled by a user or upstream program.
    CANCELED = 6


class ExecutionResult:
    """Contains the result of executing actions or plans."""

    def __init__(
        self,
        status: ExecutionStatus = ExecutionStatus.UNKNOWN,
        message: str | None = None,
    ) -> None:
        """
        Creates a new execution result instance.

        :param status: The resulting status code. Defaults to UNKNOWN.
        :param message: An optional message describing the result.
        """
        self.status = status
        self.message = message

    def is_success(self) -> bool:
        """
        Helper function to determine if an execution result is successful.

        :return: True if successful, otherwise False.
        """
        return self.status == ExecutionStatus.SUCCESS

    def __repr__(self) -> str:
        """Returns printable string."""
        return f"Execution result with status: {self.status.name}"


class TaskAction:
    """Task Action representation class."""

    def __init__(
        self,
        type: str,
        robot: str | None = None,
        object: str | None = None,
        room: str | None = None,
        source_location: str | None = None,
        target_location: str | None = None,
        pose: Pose | None = None,
        path: Path = Path(),
        cost: float | None = None,
    ) -> None:
        """
        Creates a new task action representation.

        :param type: Action type.
        :param robot: Name of robot to execute the action.
        :param object: Target object type or name.
        :param room: Target room name.
        :param source_location: Source location type or name.
        :param target_location: Target location type or name.
        :param pose: Optional pose parameter for the action.
        :param path: A specific path to follow, if provided.
        :param cost: Optional action cost.
        """
        # Action-agnostic parameters
        self.type = type.lower()
        self.robot = robot
        self.cost = cost

        # Action-specific parameters
        self.object = object  # Target object name
        self.room = room  # Target room name
        self.source_location = source_location  # Source location name
        self.target_location = target_location  # Target location name
        self.pose = pose  # Target pose
        self.path = path  # Path object containing a list of poses

    def __repr__(self) -> str:
        """Returns printable string describing an action."""
        # Include the robot name if any.
        if self.robot is not None:
            act_str = f"[{self.robot}] "
        else:
            act_str = ""

        # Format actions based on their types
        # NAVIGATE
        if self.type == "navigate":
            act_str += "Navigate"
            if self.source_location is not None:
                act_str += f" from {self.source_location}"
            if self.target_location is not None:
                act_str += f" to {self.target_location}"
            if self.pose is not None:
                act_str += f"\n  At {self.pose}"
            if self.path.num_poses > 0:
                act_str += f"\n  {self.path}"
        # PICK
        elif self.type == "pick":
            act_str += "Pick"
            if self.object is not None:
                act_str += f" {self.object}"
            else:
                act_str += " object"
            if self.target_location is not None:
                act_str += f" from {self.target_location}"
            if self.pose is not None:
                act_str += f"\n  At {self.pose}"
        # PLACE
        elif self.type == "place":
            act_str += "Place"
            if self.object is not None:
                act_str += f" {self.object}"
            else:
                act_str += " object"
            if self.target_location is not None:
                act_str += f" at {self.target_location}"
            if self.pose is not None:
                act_str += f"\n  At {self.pose}"
        # DETECT
        elif self.type == "detect":
            act_str += "Detect"
            if self.object is not None:
                act_str += f" {self.object}"
            else:
                act_str += " objects"
            if self.target_location is not None:
                act_str += f" at {self.target_location}"
        # OPEN / CLOSE
        elif self.type == "open":
            act_str += "Open"
            if self.target_location is not None:
                act_str += f" {self.target_location}"
            else:
                act_str += " current location"
        elif self.type == "close":
            act_str += "Close"
            if self.target_location is not None:
                act_str += f" {self.target_location}"
            else:
                act_str += " current location"
        else:
            return f"Invalid action type: {self.type}"

        if self.cost is not None:
            act_str += f"\n  Cost: {self.cost:.3f}"
        return act_str


class TaskPlan:
    """
    Task Plan representation class.

    A task plan is simply described as a sequence of task actions
    (:class:`pyrobosim.planning.actions.TaskAction`).
    """

    def __init__(
        self, robot: str | None = None, actions: list[TaskAction] = []
    ) -> None:
        """
        Creates a new task plan.

        :param robot: Name of robot to execute the plan.
        :param actions: List of actions.
        """
        self.robot = robot
        self.set_actions(actions)

    def set_actions(self, actions: list[TaskAction]) -> None:
        """
        Sets actions and updates the total cost over all the actions.
        Use this method rather than directly setting the actions variable.

        :param actions: List of actions.
        """
        self.actions = actions
        self.total_cost = sum([a.cost for a in self.actions if a.cost is not None])

    def size(self) -> int:
        """
        Get the total number of actions comprising this task plan.

        :return: Size of plan.
        """
        return len(self.actions)

    def __repr__(self) -> str:
        """Returns printable string describing a task plan."""
        # Check for empty plan
        if len(self.actions) == 0:
            return "Empty plan"

        # Loop through the actions in the plan and print them
        out_str = "\n=== Task Plan: ===\n"
        for i, act in enumerate(self.actions):
            out_str += f"{i+1}. {act}\n"
        out_str += f"=== Total Cost: {self.total_cost:.3f} ===\n"
        return out_str
