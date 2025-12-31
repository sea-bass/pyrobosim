#!/usr/bin/env python3

"""
Test script showing PyRoboSim partial hallway observability feature.

This refers to the robot not knowing the true state (open/closed) of hallways
until it senses them with sensors.
"""
import argparse

from pyrobosim.core.robot import Robot
from pyrobosim.core.world import World
from pyrobosim.gui import start_gui
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

    # Set the location metadata
    world.add_metadata(
        locations=[
            data_folder / "example_location_data_furniture.yaml",
            data_folder / "example_location_data_accessories.yaml",
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
        room_start="kitchen",
        room_end="bathroom",
        width=0.7,
        color="#666666",
        is_open=False,
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
        is_open=False,
    )

    # Add locations
    world.add_location(
        category="table",
        parent="kitchen",
        pose=Pose(x=0.85, y=-0.5, z=0.0, yaw=-90.0, angle_units="degrees"),
    )
    desk_pose = world.get_pose_relative_to(
        Pose(x=0.525, y=0.4, z=0.0, yaw=0.0), "bedroom"
    )
    world.add_location(category="desk", parent="bedroom", pose=desk_pose)
    world.add_location(
        category="counter",
        parent="bathroom",
        pose=Pose(x=-2.45, y=2.5, z=0.0, q=[0.634411, 0.0, 0.0, 0.7729959]),
    )

    # Add robots
    lidar0 = Lidar2D(
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
            lidar_sensor_name="lidar",
        ),
        sensors={"lidar": lidar0},
        color="#CC00CC",
        partial_obs_hallways=True,
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
    rrt_planner0 = RRTPlanner(**planner_config_rrt)
    robot0.set_path_planner(rrt_planner0)

    if multirobot:
        lidar1 = Lidar2D(
            update_rate_s=0.1,
            angle_units="degrees",
            min_angle=-120.0,
            max_angle=120.0,
            angular_resolution=5.0,
            max_range_m=2.0,
        )
        robot1 = Robot(
            name="robot1",
            radius=0.08,
            color=(0.8, 0.8, 0),
            path_executor=ConstantVelocityExecutor(
                linear_velocity=1.0,
                dt=0.1,
                max_angular_velocity=4.0,
                validate_during_execution=True,
                lidar_sensor_name="lidar",
            ),
            sensors={"lidar": lidar1},
            partial_obs_hallways=True,
        )
        world.add_robot(robot1, loc="bathroom")
        planner_config_prm = {
            "collision_check_step_dist": 0.025,
            "max_connection_dist": 1.5,
            "max_nodes": 100,
            "compress_path": False,
        }
        prm_planner = PRMPlanner(**planner_config_prm)
        robot1.set_path_planner(prm_planner)

        lidar2 = Lidar2D(
            update_rate_s=0.1,
            angle_units="degrees",
            min_angle=-120.0,
            max_angle=120.0,
            angular_resolution=5.0,
            max_range_m=2.0,
        )
        robot2 = Robot(
            name="robot2",
            radius=0.06,
            color=(0, 0.8, 0.8),
            path_executor=ConstantVelocityExecutor(
                linear_velocity=1.0,
                dt=0.1,
                max_angular_velocity=4.0,
                validate_during_execution=True,
                lidar_sensor_name="lidar",
            ),
            sensors={"lidar": lidar2},
            partial_obs_hallways=True,
        )
        world.add_robot(robot2, loc="bedroom")
        planner_config_astar = {
            "grid_resolution": 0.05,
            "grid_inflation_radius": 0.15,
            "diagonal_motion": True,
            "heuristic": "euclidean",
        }
        astar_planner = AStarPlanner(**planner_config_astar)
        robot2.set_path_planner(astar_planner)

    return world


def parse_args() -> argparse.Namespace:
    """Parse command-line arguments"""
    parser = argparse.ArgumentParser(
        description="PyRoboSim demo for hallway partial observability feature."
    )
    parser.add_argument(
        "--multirobot",
        action="store_true",
        help="This option will add multiple robots to the world.",
    )
    return parser.parse_args()


if __name__ == "__main__":
    args = parse_args()

    # Create a world.
    world = create_world(args.multirobot)

    # Start the GUI.
    start_gui(world)
