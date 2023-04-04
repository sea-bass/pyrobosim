import os

from pyrobosim.core import WorldYamlLoader
from pyrobosim.gui import start_gui
from pyrobosim.navigation import PathPlanner
from pyrobosim.utils.general import get_data_folder
from pyrobosim.utils.pose import Pose
from pyrobosim.navigation import occupancy_grid_from_world

# Load a test world.
data_folder = get_data_folder()
loader = WorldYamlLoader()
world = loader.from_yaml(os.path.join(data_folder, "test_world.yaml"))


def demo_astar():
    """Creates an occupancy grid based A* planner and plans a path."""
    robot = world.robots[0]
    planner_config = {
        "grid": occupancy_grid_from_world(
            world, resolution=0.05, inflation_radius=1.5 * robot.radius
        ),
        "diagonal_motion": True,
        "heuristic": "euclidean",
    }

    planner = PathPlanner("astar", **planner_config)
    start = Pose(x=-0.5, y=-0.5)
    goal = Pose(x=3.0, y=3.0)
    robot.set_pose(start)
    robot.set_path_planner(planner)
    robot.current_path = robot.plan_path(start, goal)
    planner.info()


if __name__ == "__main__":
    demo_astar()
    start_gui(world)
