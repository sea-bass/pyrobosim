""" Defines actions for task and motion planning """

class TaskAction:
    """ Task Action representation class """
    def __init__(self, action_type, cost=0.0):
        # Action-agnostic parameters
        self.action_type = action_type.lower()
        self.cost = cost

        # Action-specific parameters
        # (Must be set after construction time)
        self.target_object = None        # Target object name
        self.source_location = None      # Source location name
        self.target_pose = None          # Target pose name

    def __repr__(self):
        """ Returns a string describing an action """
        
        # Format actions based on their types
        # MOVE
        if self.action_type == "move":
            act_str = f"Move from {self.source_location} to {self.target_location}" 
        # PICK
        elif self.action_type == "pick":
            act_str = f"Pick {self.target_object} from {self.target_location}"
        # PLACE
        elif self.action_type == "place":
            act_str = f"Place {self.target_object} at {self.target_location}"
        else:
            print(f"Invalid action type {self.action_type}")
            return None
        
        return act_str


class TaskPlan:
    """ Task Plan representation """
    def __init__(self, actions=[]):
        self.actions = actions

    def total_cost(self):
        """ Get the total cost of a task plan """
        if len(self.actions) == 0:
            return 0
        else:
            return sum([a.cost for a in self.actions])

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
        out_str += f"=== Total Cost: {self.total_cost():.3f} ===\n"
        return out_str
