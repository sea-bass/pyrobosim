""" Defines actions for task and motion planning. """

from ..utils.motion import Path


class TaskAction:
    """Task Action representation class."""

    def __init__(
        self,
        type,
        robot=None,
        object=None,
        room=None,
        source_location=None,
        target_location=None,
        pose=None,
        path=Path(),
        cost=None,
    ):
        """
        Creates a new task action representation.

        :param type: Action type.
        :type type: str
        :param robot: Name of robot to execute the action.
        :type robot: str, optional
        :param object: Target object type or name.
        :type object: str, optional
        :param room: Target room name.
        :type room: str, optional
        :param source_location: Source location type or name.
        :type source_location: str, optional
        :param target_location: Target location type or name.
        :type target_location: str, optional
        :param pose: Optional pose parameter for the action.
        :type pose: :class:`pyrobosim.utils.pose.Pose`, optional
        :param path: A specific path to follow, if provided.
        :type path: :class:`pyrobosim.utils.motion.Path`, optional
        :param cost: Optional action cost.
        :type cost: float
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

    def __repr__(self):
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
                act_str += f"\nAt {self.pose}"
            if self.path.num_poses > 0:
                act_str += f"\n{self.path}"
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
                act_str += f"\nAt {self.pose}"
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
                act_str += f"\nAt {self.pose}"
        else:
            print(f"Invalid action type {self.action_type}")
            return None

        if self.cost is not None:
            act_str += f", Cost: {self.cost:.3f}"
        return act_str


class TaskPlan:
    """
    Task Plan representation class.

    A task plan is simply described as a sequence of task actions
    (:class:`pyrobosim.planning.actions.TaskAction`).
    """

    def __init__(self, robot=None, actions=[]):
        """
        Creates a new task plan.

        :param robot: Name of robot to execute the plan.
        :type robot: str, optional
        :param actions: List of actions.
        :type actions: list[:class:`pyrobosim.planning.actions.TaskAction`], optional
        """
        self.robot = robot
        self.set_actions(actions)

    def set_actions(self, actions):
        """
        Sets actions and updates the total cost over all the actions.
        Use this method rather than directly setting the actions variable.

        :param actions: List of actions.
        :type actions: list[:class:`pyrobosim.planning.actions.TaskAction`]
        """
        self.actions = actions
        self.total_cost = sum([a.cost for a in self.actions])

    def size(self):
        """
        Get the total number of actions comprising this task plan.

        :return: Size of plan.
        :rtype: int
        """
        return len(self.actions)

    def __repr__(self):
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
