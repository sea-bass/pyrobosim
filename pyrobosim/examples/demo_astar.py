import os

from pyrobosim.core import WorldYamlLoader
from pyrobosim.gui import start_gui
from pyrobosim.navigation import PathPlanner
from pyrobosim.utils.general import get_data_folder
from pyrobosim.utils.pose import Pose
from pyrobosim.navigation import OccupancyGrid

# Load a test world.
world_file = os.path.join(get_data_folder(), "test_world.yaml")
world = WorldYamlLoader().from_yaml(world_file)


def demo_astar():
    """Creates an occupancy grid based A* planner and plans a path."""
    robot = world.robots[0]
    planner_config = {
        "grid": OccupancyGrid.from_world(
            world, resolution=0.05, inflation_radius=1.5 * robot.radius
        ),
        "diagonal_motion": True,
        "heuristic": "euclidean",
        "compress_path": False,
    }

    planner = PathPlanner("astar", **planner_config)
    start = Pose(x=-0.5, y=-0.5)
    goal = Pose(x=3.0, y=3.0)
    robot.set_pose(start)
    robot.set_path_planner(planner)
    result = robot.plan_path(start, goal)
    planner.info()


if __name__ == "__main__":
    demo_astar()
    start_gui(world)
