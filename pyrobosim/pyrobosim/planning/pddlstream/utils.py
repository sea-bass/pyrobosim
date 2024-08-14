"""
Utilities to connect world models with PDDLStream.
"""

import os

from ..actions import TaskAction, TaskPlan
from ...manipulation.grasping import Grasp
from ...utils.general import get_data_folder
from ...utils.motion import Path
from ...utils.pose import Pose


def get_default_domains_folder():
    """
    Returns the default path to the folder containing PDDLStream domains.

    :return: Path to domains folder.
    :rtype: str
    """
    return os.path.join(get_data_folder(), "pddlstream", "domains")


def get_default_stream_info_fn():
    """
    Gets a function that creates the default PDDLStream stream information dictionary.

    :return: PDDLStream stream information function
    :rtype: function
    """
    from .default_mappings import get_stream_info

    return get_stream_info


def get_default_stream_map_fn():
    """
    Gets a function that creates the default PDDLStream stream mappings dictionary
    given a `pyrobosim.core.world.World` object and a `pyrobosim.core.robot.Robot` object.

    :return: PDDLStream stream mappings function
    :rtype: function
    """
    from .default_mappings import get_stream_map

    return get_stream_map


def world_to_pddlstream_init(world, robot):
    """
    Converts a world representation object to a PDDLStream compatible
    initial condition specification.

    :param world: World model.
    :type world: :class:`pyrobosim.core.world.World`
    :param robot: Robot to use for planning.
    :type robot: :class:`pyrobosim.core.robot.Robot`
    :return: PDDLStream compatible initial state representation.
    :rtype: list[tuple]
    """

    # Start with the robot initial conditions
    init_loc = robot.location
    robot_pose = robot.get_pose()
    if not init_loc:
        init_loc = world.get_location_from_pose(robot_pose)
    init = [
        ("Robot", robot),
        ("CanMove", robot),
        ("HandEmpty", robot),
        ("At", robot, init_loc),
        ("Pose", robot_pose),
        ("AtPose", robot, robot_pose),
    ]

    # Loop through all the locations and their relationships.
    # This includes rooms, hallways, and object spawns (which are children of locations).
    for room in world.rooms:
        init.append(("Room", room))
        init.append(("Location", room))
    loc_categories = set()
    for loc in world.locations:
        for spawn in loc.children:
            init.append(("Location", spawn))
            init.append(("Is", spawn, loc.category))
            init.append(("AtRoom", spawn, loc.parent))
            # TODO: Note that open/locked status pertains to the entire location,
            # but the PDDL domain only knows about the individual object spawns.
            # As such, this may not work on locations that have multiple object spawns.
            if loc.is_open:
                init.append(("IsOpen", spawn))
            if loc.is_locked:
                init.append(("IsLocked", spawn))
            # TODO: This assumes no locations have been observed at the start of planning.
            # You could add something like: init.append(("IsObserved", spawn))
        loc_categories.add(loc.category)
    for loc_cat in loc_categories:
        init.append(("Type", loc_cat))
    for hallway in world.hallways:
        init.append(("Hallway", hallway))
        init.append(("Location", hallway))
        if hallway.is_open:
            init.append(("IsOpen", hallway))
        if hallway.is_locked:
            init.append(("IsLocked", hallway))

    # Loop through all the objects and their relationships.
    obj_categories = set()
    for obj in robot.get_known_objects():
        init.append(("Obj", obj))
        init.append(("Is", obj, obj.category))
        # If the object is the current manipulated object, change the state of
        # the robot. Otherwise, the object is at its parent location.
        if robot.manipulated_object == obj:
            init.remove(("HandEmpty", robot))
            init.append(("Holding", robot, obj))
        else:
            init.append(("At", obj, obj.parent))
            init.append(("Pose", obj.pose))
            init.append(("AtPose", obj, obj.pose))
        obj_categories.add(obj.category)
    for obj_cat in obj_categories:
        init.append(("Type", obj_cat))

    return init


def process_goal_specification(goal_literals, world):
    """
    Processes and validates a goal specification for planning.

    :param goal_literals: List of literals describing a goal specification.
    :type goal_literals: list[tuple]
    """
    for i, literal in enumerate(goal_literals):
        # If any predicate has a string argument, check whether it corresponds
        # to a named entity in the world. If it does, replace it.
        for j, elem in enumerate(literal):
            if j > 0 and isinstance(elem, str):
                entity = world.get_entity_by_name(elem)
                if entity:
                    replace_goal_literal_tuple(goal_literals, i, j, entity)


def replace_goal_literal_tuple(goal_literals, literal_idx, arg_idx, new_val):
    """
    Utility function to replace the element of a goal literal tuple in place.

    :param goal_literals: List of literals describing a goal specification.
    :type goal_literals: list[tuple]
    :param literal_idx: Index of goal literal in list to replace.
    :type literal_idx: int
    :param arg_idx: Index of argument in goal literal to replace.
    :type arg_idx: int
    :param new_val: New value to replace inside the goal literal tuple.
    :type new_val: Any
    """
    literal_copy = list(goal_literals[literal_idx])
    literal_copy[arg_idx] = new_val
    goal_literals[literal_idx] = tuple(literal_copy)


def pddlstream_solution_to_plan(solution, robot):
    """
    Converts the output plan of a PDDLStream solution to a plan
    list compatible with plan execution infrastructure.

    :param solution: PDDLStream compatible initial state representation.
    :type solution: list[tuple]
    :param robot: Name of robot to execute plan.
    :type robot: str
    :return: Task plan object.
    :rtype: :class:`pyrobosim.planning.actions.TaskPlan`
    """
    # Unpack the PDDLStream solution and handle the None case
    plan, total_cost, _ = solution
    if plan is None or len(plan) == 0:
        return None

    plan_out = TaskPlan(robot=robot, actions=[])
    for act_pddl in plan:
        # Convert the PDDL action to a TaskAction
        act = TaskAction(act_pddl.name)
        act.robot = robot
        if act.type == "navigate":
            act.source_location = act_pddl.args[1]
            act.target_location = act_pddl.args[2]
            # Search for a path.
            for arg in act_pddl.args[3:]:
                if isinstance(arg, Path):
                    act.path = arg

        elif act.type in ("pick", "place"):
            act.object = act_pddl.args[1]
            act.target_location = act_pddl.args[2]
            # If a pick/place pose is specified, add it.
            if len(act_pddl.args) > 3:
                arg = act_pddl.args[3]
                if isinstance(arg, Pose):
                    act.pose = arg
            # If a grasp pose is specified for the pick action, use that instead.
            if act.type == "pick":
                for arg in act_pddl.args:
                    if isinstance(arg, Grasp):
                        act.pose = arg.origin_wrt_world

        elif act.type in ("detect", "open", "close"):
            act.target_location = act_pddl.args[1]

        else:
            raise ValueError(f"No known action type: {act.type}")

        # Add the action to the task plan
        plan_out.actions.append(act)

    # TODO: Find a way to get the individual action costs from PDDLStream.
    # For now, set total plan cost, which is available from the solution.
    plan_out.total_cost = total_cost
    return plan_out
