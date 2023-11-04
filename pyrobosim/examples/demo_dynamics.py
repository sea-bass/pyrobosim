#!/usr/bin/env python3

"""
Test script showing how to command robot velocities and simulate dynamics.
"""
import os
import numpy as np
import time
from threading import Thread

from pyrobosim.core import Robot, World
from pyrobosim.gui import start_gui
from pyrobosim.utils.general import get_data_folder
from pyrobosim.utils.pose import Pose


data_folder = get_data_folder()


def create_world():
    """Create a test world"""
    world = World()

    # Set the location and object metadata
    world.set_metadata(
        locations=os.path.join(data_folder, "example_location_data.yaml"),
        objects=os.path.join(data_folder, "example_object_data.yaml"),
    )

    # Add rooms
    r1coords = [(-1, -1), (1.5, -1), (1.5, 1.5), (0.5, 1.5)]
    world.add_room(
        name="kitchen",
        footprint=r1coords,
        color=[1, 0, 0],
        nav_poses=[Pose(x=0.75, y=0.75, z=0.0, yaw=0.0)],
    )
    r2coords = [(1.75, 2.5), (3.5, 2.5), (3.5, 4), (1.75, 4)]
    world.add_room(name="bedroom", footprint=r2coords, color=[0, 0.6, 0])
    r3coords = [(-1, 1), (-1, 3.5), (-3.0, 3.5), (-2.5, 1)]
    world.add_room(name="bathroom", footprint=r3coords, color=[0, 0, 0.6])

    # Add hallways between the rooms
    world.add_hallway(room_start="kitchen", room_end="bathroom", width=0.7)
    world.add_hallway(
        room_start="bathroom",
        room_end="bedroom",
        width=0.5,
        conn_method="angle",
        conn_angle=0,
        offset=0.8,
    )
    world.add_hallway(
        room_start="kitchen",
        room_end="bedroom",
        width=0.6,
        conn_method="points",
        conn_points=[(1.0, 0.5), (2.5, 0.5), (2.5, 3.0)],
    )

    # Add locations
    table = world.add_location(
        category="table",
        parent="kitchen",
        pose=Pose(x=0.85, y=-0.5, z=0.0, yaw=-np.pi / 2.0),
    )
    desk = world.add_location(
        category="desk", parent="bedroom", pose=Pose(x=3.15, y=3.65, z=0.0, yaw=0.0)
    )
    counter = world.add_location(
        category="counter",
        parent="bathroom",
        pose=Pose(x=-2.45, y=2.5, z=0.0, q=[0.634411, 0.0, 0.0, 0.7729959]),
    )

    # Add robots
    robot0 = Robot(
        name="robot0",
        radius=0.1,
        max_linear_acceleration=1.0,
        max_angular_acceleration=1.0,
    )
    world.add_robot(robot0, loc="kitchen")

    robot1 = Robot(
        name="robot1", radius=0.08, color=(0.8, 0.8, 0), max_angular_acceleration=0.05
    )
    world.add_robot(robot1, loc="bathroom")

    robot2 = Robot(
        name="robot2", radius=0.06, color=(0, 0.8, 0.8), max_linear_acceleration=5.0
    )
    world.add_robot(robot2, loc="bedroom")

    return world


def command_robots(world):
    """Demonstrates robot dynamics by commanding robots."""
    dt = 0.1
    vel_commands = [
        np.array([0.1, 0.0, 0.5]),  # robot0
        np.array([0.0, 0.0, -1.0]),  # robot1
        np.array([0.2, 0.0, 0.0]),  # robot2
    ]
    backup_vel = np.array([-1.0, 0.0, 0.0])

    while True:
        t_start = time.time()
        for robot, cmd_vel in zip(world.robots, vel_commands):
            if robot.is_in_collision():
                cmd_vel = backup_vel

            robot.dynamics.step(cmd_vel, dt, world=world, check_collisions=True)

        t_elapsed = time.time() - t_start
        time.sleep(max(0, dt - t_elapsed))


if __name__ == "__main__":
    world = create_world()

    # Command robots on a separate thread.
    robot_commands_thread = Thread(target=lambda: command_robots(world))
    robot_commands_thread.start()

    # Start the program either as ROS node or standalone.
    start_gui(world)
