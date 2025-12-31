#!/usr/bin/env python3

"""
Test script showing how to build a world and use it with PyRoboSim.
"""
import argparse

from pyrobosim.core.robot import Robot
from pyrobosim.core.world import World
from pyrobosim.core.yaml_utils import WorldYamlLoader
from pyrobosim.gui import start_gui
from pyrobosim.manipulation import GraspGenerator, ParallelGraspProperties
from pyrobosim.navigation.execution import ConstantVelocityExecutor
from pyrobosim.navigation.a_star import AStarPlanner
from pyrobosim.navigation.prm import PRMPlanner
from pyrobosim.navigation.rrt import RRTPlanner
from pyrobosim.sensors.lidar import Lidar2D
from pyrobosim.utils.general import get_data_folder
from pyrobosim.utils.pose import Pose


data_folder = get_data_folder()


def create_world(multirobot: bool = False) -> World:
    """Create a test world"""
    world = World()

    # Set the location and object metadata
    world.add_metadata(
        locations=[
            data_folder / "example_location_data_furniture.yaml",
            data_folder / "example_location_data_accessories.yaml",
        ],
        objects=[
            data_folder / "example_object_data_food.yaml",
            data_folder / "example_object_data_drink.yaml",
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

    # Add robots
    grasp_props = ParallelGraspProperties(
        max_width=0.175,
        depth=0.1,
        height=0.04,
        width_clearance=0.01,
        depth_clearance=0.01,
    )
    lidar = Lidar2D(
        update_rate_s=0.1,
        angle_units="degrees",
        min_angle=-120.0,
        max_angle=120.0,
        angular_resolution=5.0,
        max_range_m=2.0,
    )

    robot0 = Robot(
        name="robot0",
        radius=0.1,
        path_executor=ConstantVelocityExecutor(
            linear_velocity=1.0,
            dt=0.1,
            max_angular_velocity=4.0,
            validate_during_execution=True,
        ),
        sensors={"lidar": lidar} if args.lidar else None,
        grasp_generator=GraspGenerator(grasp_props),
        partial_obs_objects=args.partial_obs_objects,
        color="#CC00CC",
    )
    world.add_robot(robot0, loc="kitchen")
    planner_config_rrt = {
        "bidirectional": True,
        "rrt_connect": False,
        "rrt_star": True,
        "collision_check_step_dist": 0.025,
        "max_connection_dist": 0.5,
        "rewire_radius": 1.5,
        "compress_path": False,
    }
    rrt_planner = RRTPlanner(**planner_config_rrt)
    robot0.set_path_planner(rrt_planner)

    if multirobot:
        robot1 = Robot(
            name="robot1",
            radius=0.08,
            color=(0.8, 0.8, 0),
            path_executor=ConstantVelocityExecutor(),
            grasp_generator=GraspGenerator(grasp_props),
            partial_obs_objects=args.partial_obs_objects,
        )
        world.add_robot(robot1, loc=["bathroom", "counter"])
        planner_config_prm = {
            "collision_check_step_dist": 0.025,
            "max_connection_dist": 1.5,
            "max_nodes": 100,
            "compress_path": False,
        }
        prm_planner = PRMPlanner(**planner_config_prm)
        robot1.set_path_planner(prm_planner)

        robot2 = Robot(
            name="robot2",
            radius=0.06,
            color=(0, 0.8, 0.8),
            path_executor=ConstantVelocityExecutor(),
            grasp_generator=GraspGenerator(grasp_props),
            partial_obs_objects=args.partial_obs_objects,
        )
        world.add_robot(robot2, loc=["bedroom", "desk"])
        planner_config_astar = {
            "grid_resolution": 0.05,
            "grid_inflation_radius": 0.15,
            "diagonal_motion": True,
            "heuristic": "euclidean",
        }
        astar_planner = AStarPlanner(**planner_config_astar)
        robot2.set_path_planner(astar_planner)

    return world


def create_world_from_yaml(world_file: str) -> World:
    return WorldYamlLoader().from_file(data_folder / world_file)


def parse_args() -> argparse.Namespace:
    """Parse command-line arguments"""
    parser = argparse.ArgumentParser(description="Main pyrobosim demo.")
    parser.add_argument(
        "--multirobot",
        action="store_true",
        help="If no YAML file is specified, this option will add "
        "multiple robots to the world defined in this file.",
    )
    parser.add_argument(
        "--world-file",
        default="",
        help="YAML file name (should be in the pyrobosim/data folder). "
        + "If not specified, a world will be created programmatically.",
    )
    parser.add_argument(
        "--partial-obs-objects",
        action="store_true",
        help="If True, robots have partial observability of objects and must detect them.",
    )
    parser.add_argument(
        "--lidar",
        action="store_true",
        help="If True, adds a lidar sensor to the first robot.",
    )
    return parser.parse_args()


if __name__ == "__main__":
    args = parse_args()

    # Create a world or load it from file.
    if args.world_file == "":
        world = create_world(args.multirobot)
    else:
        world = create_world_from_yaml(args.world_file)

    # Start the program either as ROS node or standalone.
    start_gui(world)
