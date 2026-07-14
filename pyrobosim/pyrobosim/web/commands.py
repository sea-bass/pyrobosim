"""
World and robot commands for the web frontend.

These mirror the Qt GUI's button callbacks: long-running actions (navigate,
pick, place, detect, open, close) run on background daemon threads so the web
callback returns immediately, while quick ones run inline.
"""

import threading
from typing import Callable

import numpy as np

from ..core.robot import Robot
from ..core.world import World


def resolve_robot(world: World, robot_name: str | None) -> Robot | None:
    """
    Resolves a robot name to a robot in the world.

    :param world: The world to search for the robot.
    :param robot_name: The name of the robot. ``"world"`` or None mean no robot.
    :return: The matching robot, or None if the name does not refer to one.
    """
    if robot_name is None or robot_name == "world":
        return None
    return world.get_robot_by_name(robot_name)


def run_async(fn: Callable[[], object]) -> None:
    """
    Runs a callable on a background daemon thread.

    :param fn: The callable to run.
    """
    threading.Thread(target=fn, daemon=True).start()


def navigate(robot: Robot, goal: str) -> None:
    """
    Navigates a robot to a goal entity (resolved by the world).

    :param robot: The robot to command.
    :param goal: The goal entity query, e.g., a location, room, or hallway name.
    """
    if not goal or robot.is_moving():
        return
    robot.logger.info(f"Navigating to {goal}")
    run_async(lambda: robot.navigate(goal=goal, path=None, realtime_factor=1.0))


def pick(robot: Robot, query: str) -> None:
    """
    Picks an object with a robot.

    :param robot: The robot to command.
    :param query: An optional object query (e.g., a name or category).
    """
    robot.logger.info(f"Picking {query}")
    run_async(lambda: robot.pick_object(query or None, None))


def place(robot: Robot) -> None:
    """
    Places the object a robot is holding.

    :param robot: The robot to command.
    """
    if robot.manipulated_object is None:
        return
    robot.logger.info(f"Placing {robot.manipulated_object.name}")
    run_async(lambda: robot.place_object(pose=None))


def detect(robot: Robot, query: str) -> None:
    """
    Detects objects at a robot's current location.

    :param robot: The robot to command.
    :param query: An optional object query to filter the detections.
    """
    robot.logger.info("Detecting objects")
    run_async(lambda: robot.detect_objects(query or None))


def open_location(world: World, robot: Robot | None, goal: str) -> None:
    """
    Opens the robot's current location, or a named location if no robot.

    :param world: The world containing the robot and locations.
    :param robot: The robot to command, or None to open a location by name.
    :param goal: The location name to open when no robot is given.
    """
    if robot is not None and robot.location is not None:
        run_async(robot.open_location)
    elif robot is None and goal:
        world.open_location(goal)


def close_location(world: World, robot: Robot | None, goal: str) -> None:
    """
    Closes the robot's current location, or a named location if no robot.

    :param world: The world containing the robot and locations.
    :param robot: The robot to command, or None to close a location by name.
    :param goal: The location name to close when no robot is given.
    """
    if robot is not None and robot.location is not None:
        run_async(robot.close_location)
    elif robot is None and goal:
        world.close_location(goal)


def randomize_pose(world: World, robot: Robot) -> None:
    """
    Moves a robot to a random collision-free pose.

    :param world: The world containing the robot.
    :param robot: The robot to move.
    """
    if robot.is_moving():
        return
    pose = world.sample_free_robot_pose_uniform(robot, ignore_robots=False)
    if pose is not None:
        robot.set_pose(pose)
        if robot.manipulated_object is not None:
            robot.manipulated_object.set_pose(pose)


def reset_path_planner(robot: Robot) -> None:
    """
    Resets a robot's path planner.

    :param robot: The robot whose planner to reset.
    """
    if not robot.is_moving():
        robot.reset_path_planner()


def cancel_action(robot: Robot) -> None:
    """
    Cancels any running action for a robot.

    :param robot: The robot whose actions to cancel.
    """
    robot.cancel_actions()


def random_goal(world: World) -> str | None:
    """
    Samples a random navigation goal name (location, hallway, or room).

    :param world: The world to sample a goal from.
    :return: A random goal name, or None if the world has no goal entities.
    """
    names = (
        world.get_location_names() + world.get_hallway_names() + world.get_room_names()
    )
    return str(np.random.choice(names)) if names else None


def random_object(world: World) -> str | None:
    """
    Samples a random object name to use as a manipulation target.

    :param world: The world to sample an object from.
    :return: A random object name, or None if the world has no objects.
    """
    names = world.get_object_names()
    return str(np.random.choice(names)) if names else None
