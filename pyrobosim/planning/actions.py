""" Defines actions for task and motion planning """

class TaskAction:
    """ Task Action representation class """

    def __init__(self, type, object=None, room=None,
                 source_location=None, target_location=None,
                 pose=None, cost=None):
        # Action-agnostic parameters
        self.type = type.lower()
        self.cost = cost

        # Action-specific parameters
        self.object = object                    # Target object name
        self.room = room                        # Target room name
        self.source_location = source_location  # Source location name
        self.target_location = target_location  # Target location name
        self.pose = pose                        # Target pose

    def __repr__(self):
        """ Returns a string describing an action """
        
        # Format actions based on their types
        # NAVIGATE
        if self.action_type == "navigate":
            act_str = "Navigate"
            if self.source_location is not None:
                act_str += f" from {self.source_location}"
            if self.target_location is not None:
                act_str += f" to {self.target_location}"
            if self.pose is not None:
                act_str += f" at {self.pose}"
        # PICK
        elif self.action_type == "pick":
            act_str = "Pick"
            if self.object is not None:
                act_str += f" {self.target_object}"
            else:
                act_str += " object" 
            if self.target_location is not None:
                act_str += f" from {self.target_location}"
            if self.pose is not None:
                act_str += f" at {self.pose}"
        # PLACE
        elif self.action_type == "place":
            act_str = "Place"
            if self.object is not None:
                act_str += f" {self.target_object}"
            else:
                act_str += " object" 
            if self.target_location is not None:
                act_str += f" at {self.target_location}"
            if self.pose is not None:
                act_str += f" at {self.pose}"
        else:
            print(f"Invalid action type {self.action_type}")
            return None

        if self.cost is not None:
            act_str += f", Cost: {self.cost:.3f}"
        return act_str


class TaskPlan:
    """ Task Plan representation """
    def __init__(self, actions=[]):
        self.set_actions(actions)

    def set_actions(self, actions):
        """ 
        Sets actions and updates the total cost 
        Use this method rather than directly setting the actions variable
        """
        self.actions = actions
        self.total_cost = sum([a.cost for a in self.actions])

    def size(self):
        """ Get the length of the plan """
        return len(self.actions)

    def __repr__(self):
        """ Formats a plan for printing """
        # Check for empty plan
        if len(self.actions) == 0:
            return "Empty plan"

        # Loop through the actions in the plan and print them
        out_str = "\n=== Task Plan: ===\n"
        for i, act in enumerate(self.actions):
            out_str += f"{i+1}. {act}\n"
        out_str += f"=== Total Cost: {self.total_cost:.3f} ===\n"
        return out_str
