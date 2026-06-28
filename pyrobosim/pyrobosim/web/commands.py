"""
Command dispatch for the web frontend.

These functions mirror the GUI's button callbacks and action runners, but run
without Qt: long-running actions (navigate, pick, place, detect, open, close)
are executed on background daemon threads so the web callback returns
immediately, while quick actions (randomize, reset, cancel) run inline.

The world model already guards all GUI hooks with ``if self.world.gui is not
None``, so these actions run safely with no GUI attached.
"""

import threading
from typing import Callable

import numpy as np

from ..core.robot import Robot
from ..core.world import World


def resolve_robot(world: World, robot_name: str | None) -> Robot | None:
    """Resolves a robot name to a Robot, treating ``"world"``/None as no robot."""
    if robot_name is None or robot_name == "world":
        return None
    return world.get_robot_by_name(robot_name)


def _run_async(fn: Callable[[], object]) -> None:
    """Runs a callable on a background daemon thread."""
    threading.Thread(target=fn, daemon=True).start()


def navigate(world: World, robot_name: str, goal: str) -> None:
    """Navigates the selected robot to a goal entity (resolved by the world)."""
    robot = resolve_robot(world, robot_name)
    if robot is None or not goal or robot.is_moving():
        return
    robot.logger.info(f"Navigating to {goal}")
    _run_async(lambda: robot.navigate(goal=goal, path=None, realtime_factor=1.0))


def pick(world: World, robot_name: str, goal: str) -> None:
    """Picks an object (by query) with the selected robot."""
    robot = resolve_robot(world, robot_name)
    if robot is None:
        return
    robot.logger.info(f"Picking {goal}")
    _run_async(lambda: robot.pick_object(goal or None, None))


def place(world: World, robot_name: str, goal: str) -> None:
    """Places the object the selected robot is holding."""
    robot = resolve_robot(world, robot_name)
    if robot is None or robot.manipulated_object is None:
        return
    robot.logger.info(f"Placing {robot.manipulated_object.name}")
    _run_async(lambda: robot.place_object(pose=None))


def detect(world: World, robot_name: str, goal: str) -> None:
    """Detects objects at the selected robot's location."""
    robot = resolve_robot(world, robot_name)
    if robot is None:
        return
    robot.logger.info("Detecting objects")
    _run_async(lambda: robot.detect_objects(goal or None))


def open_location(world: World, robot_name: str, goal: str) -> None:
    """Opens the robot's current location, or a named location for ``"world"``."""
    robot = resolve_robot(world, robot_name)
    if robot is not None and robot.location is not None:
        _run_async(robot.open_location)
    elif robot is None and goal:
        world.open_location(goal)


def close_location(world: World, robot_name: str, goal: str) -> None:
    """Closes the robot's current location, or a named location for ``"world"``."""
    robot = resolve_robot(world, robot_name)
    if robot is not None and robot.location is not None:
        _run_async(robot.close_location)
    elif robot is None and goal:
        world.close_location(goal)


def randomize_pose(world: World, robot_name: str, goal: str) -> None:
    """Moves the selected robot to a random collision-free pose."""
    robot = resolve_robot(world, robot_name)
    if robot is None or robot.is_moving():
        return
    pose = world.sample_free_robot_pose_uniform(robot, ignore_robots=False)
    if pose is not None:
        robot.set_pose(pose)
        if robot.manipulated_object is not None:
            robot.manipulated_object.pose = pose


def reset_world(world: World, robot_name: str, goal: str) -> None:
    """Resets the world to its initial state."""
    world.reset()


def reset_path_planner(world: World, robot_name: str, goal: str) -> None:
    """Resets the selected robot's path planner."""
    robot = resolve_robot(world, robot_name)
    if robot is not None and not robot.is_moving():
        robot.reset_path_planner()


def cancel_action(world: World, robot_name: str, goal: str) -> None:
    """Cancels any running action for the selected robot."""
    robot = resolve_robot(world, robot_name)
    if robot is not None:
        robot.cancel_actions()


def random_goal(world: World, robot_name: str, goal: str) -> str | None:
    """Returns a random navigation goal name (location, hallway, or room)."""
    names = (
        world.get_location_names() + world.get_hallway_names() + world.get_room_names()
    )
    return str(np.random.choice(names)) if names else None


def random_object(world: World, robot_name: str, goal: str) -> str | None:
    """Returns a random object name to use as a manipulation target."""
    names = world.get_object_names()
    return str(np.random.choice(names)) if names else None


# Actions that update the goal text box rather than mutating the world.
GOAL_ACTIONS = {
    "rand-goal": random_goal,
    "rand-obj": random_object,
}

# Actions that operate on the world / selected robot.
WORLD_ACTIONS = {
    "navigate": navigate,
    "pick": pick,
    "place": place,
    "detect": detect,
    "open": open_location,
    "close": close_location,
    "rand-pose": randomize_pose,
    "reset-world": reset_world,
    "reset-planner": reset_path_planner,
    "cancel": cancel_action,
}

# Actions that run on a background thread and whose effect lands a little later,
# so the view should keep refreshing for a short window after they are issued.
ASYNC_ACTIONS = {"navigate", "pick", "place", "detect", "open", "close"}
