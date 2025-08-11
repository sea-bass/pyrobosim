#!/usr/bin/env python3

"""
Example showing how to build a world and use it with PyRoboSim,
additionally starting up a ROS interface.
"""
import os
import rclpy
import threading

from pyrobosim.core import Robot, World, WorldYamlLoader
from pyrobosim.gui import start_gui
from pyrobosim.navigation.execution import ConstantVelocityExecutor
from pyrobosim.navigation.rrt import RRTPlanner
from pyrobosim.utils.general import get_data_folder
from pyrobosim.utils.pose import Pose
from pyrobosim_ros.ros_interface import WorldROSWrapper


data_folder = get_data_folder()


def create_world() -> World:
    """Create a test world"""
    world = World()

    # Set the location and object metadata
    world.add_metadata(
        locations=[
            os.path.join(data_folder, "example_location_data_furniture.yaml"),
            os.path.join(data_folder, "example_location_data_accessories.yaml"),
        ],
        objects=[
            os.path.join(data_folder, "example_object_data_food.yaml"),
            os.path.join(data_folder, "example_object_data_drink.yaml"),
        ],
    )

    # Add rooms
    r1coords = [(-1, -1), (1.5, -1), (1.5, 1.5), (0.5, 1.5)]
    world.add_room(
        name="kitchen",
        pose=Pose(x=0.0, y=0.0, z=0.0, yaw=0.0),
        footprint=r1coords,
        color="red",
        nav_poses=[Pose(x=0.75, y=0.75, z=0.0, yaw=0.0)],
    )
    r2coords = [(-0.875, -0.75), (0.875, -0.75), (0.875, 0.75), (-0.875, 0.75)]
    world.add_room(
        name="bedroom",
        pose=Pose(x=2.625, y=3.25, z=0.0, yaw=0.0),
        footprint=r2coords,
        color="#009900",
    )
    r3coords = [(-1, 1), (-1, 3.5), (-3.0, 3.5), (-2.5, 1)]
    world.add_room(
        name="bathroom",
        footprint=r3coords,
        color=[0.0, 0.0, 0.6],
    )

    # Add hallways between the rooms
    world.add_hallway(
        room_start="kitchen", room_end="bathroom", width=0.7, color="#666666"
    )
    world.add_hallway(
        room_start="bathroom",
        room_end="bedroom",
        width=0.5,
        conn_method="angle",
        conn_angle=0,
        offset=0.8,
        color="dimgray",
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
        pose=Pose(x=0.85, y=-0.5, z=0.0, yaw=-90.0, angle_units="degrees"),
    )
    desk_pose = world.get_pose_relative_to(
        Pose(x=0.525, y=0.4, z=0.0, yaw=0.0), "bedroom"
    )
    desk = world.add_location(category="desk", parent="bedroom", pose=desk_pose)
    counter = world.add_location(
        category="counter",
        parent="bathroom",
        pose=Pose(x=-2.45, y=2.5, z=0.0, q=[0.634411, 0.0, 0.0, 0.7729959]),
    )

    # Add objects
    banana_pose = world.get_pose_relative_to(
        Pose(x=0.15, y=0.0, z=0.0, q=[0.9238811, 0.0, 0.0, -0.3826797]), table
    )
    world.add_object(category="banana", parent=table, pose=banana_pose)
    apple_pose = world.get_pose_relative_to(
        Pose(x=0.05, y=-0.15, z=0.0, q=[1.0, 0.0, 0.0, 0.0]), desk
    )
    world.add_object(category="apple", parent=desk, pose=apple_pose)
    world.add_object(category="apple", parent=table)
    world.add_object(category="apple", parent=table)
    world.add_object(category="water", parent=counter)
    world.add_object(category="banana", parent=counter)
    world.add_object(category="water", parent="desk")

    # Add a robot
    # Create path planner
    planner_config = {
        "bidirectional": True,
        "rrt_connect": False,
        "rrt_star": True,
        "collision_check_step_dist": 0.025,
        "max_connection_dist": 0.5,
        "rewire_radius": 1.5,
        "compress_path": False,
    }
    path_planner = RRTPlanner(**planner_config)
    robot = Robot(
        name="robot",
        radius=0.1,
        path_executor=ConstantVelocityExecutor(),
        path_planner=path_planner,
    )
    world.add_robot(robot, loc="kitchen")

    return world


def create_world_from_yaml(world_file: str) -> World:
    return WorldYamlLoader().from_file(os.path.join(data_folder, world_file))


def create_ros_node() -> WorldROSWrapper:
    """Initializes ROS node"""
    rclpy.init()
    node = WorldROSWrapper(state_pub_rate=0.1, dynamics_rate=0.01)
    node.declare_parameter("world_file", value="")

    # Set the world
    world_file = node.get_parameter("world_file").get_parameter_value().string_value
    if world_file == "":
        node.get_logger().info("Creating demo world programmatically.")
        world = create_world()
    else:
        node.get_logger().info(f"Using world file {world_file}.")
        world = create_world_from_yaml(world_file)

    node.set_world(world)

    return node


if __name__ == "__main__":
    node = create_ros_node()

    # Start ROS node in separate thread
    ros_thread = threading.Thread(target=lambda: node.start(wait_for_gui=True))
    ros_thread.start()

    # Start GUI in main thread
    start_gui(node.world)
