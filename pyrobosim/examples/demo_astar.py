import os
from pyrobosim.gui.main import start_gui
from pyrobosim.utils.pose import Pose
from pyrobosim.core.yaml import WorldYamlLoader
from pyrobosim.utils.general import get_data_folder
from pyrobosim.navigation.path_planner import PathPlanner
from pyrobosim.navigation.occupancy_grid import occupancy_grid_from_world


# Load a test world.
data_folder = get_data_folder()
loader = WorldYamlLoader()
world = loader.from_yaml(os.path.join(data_folder, "test_world.yaml"))


def demo_astar():
    """Creates an A* planner and plans a path using grid."""
    robot = world.robots[0]
    planner_config = {
        "grid": occupancy_grid_from_world(
            world, resolution=0.05, inflation_radius=1.5 * robot.radius
        ),
        "diagonal_motion": True,
        "heuristic": "euclidean",
    }

    planner = PathPlanner("astar", planner_config)
    start = Pose(x=-0.5, y=-0.5)
    goal = Pose(x=3.0, y=3.0)
    robot.set_pose(start)
    robot.set_path_planner(planner)
    robot.current_path = robot.plan_path(start, goal)


if __name__ == "__main__":
    demo_astar()
    start_gui(world)
