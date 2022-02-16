"""
Test script showing how to build a world and use it with pyrobosim
"""

import sys

from gui.main import PyRoboSim
from world.utils import Pose
from world.robot import Robot
from world.room import Room
from world.world import World


def create_world():
    # Create a robot and a world
    r = Robot(pose=Pose(x=1.0, y=1.5), radius=0.15)
    w = World(robot=r)
    w.set_metadata(locations="data/example_location_data.yaml")

    # Add rooms
    r1coords = [(0, 0), (1.5, 1.5), (1.5, 0)]
    w.add_room(Room(r1coords, name="kitchen", color=[1, 0, 0]))
    r2coords = [(1.75, 2.5), (3, 2.5), (3, 4), (1.75, 4)]
    w.add_room(Room(r2coords, name="bedroom", color=[0, 0.6, 0]))
    r3coords = [(-1, 1), (-1, 3.5), (-2.5, 3.5), (-2, 1)]
    w.add_room(Room(r3coords, name="bathroom", color=[0, 0, 0.6]))

    # Add hallways between the rooms
    w.add_hallway("kitchen", "bathroom", width=0.7)
    w.add_hallway("bathroom", "bedroom", width=0.5,
                  conn_method="angle", conn_angle=0, offset=0.75)
    w.add_hallway("kitchen", "bedroom", width=0.6,
                  conn_method="points",
                  conn_points=[(1.0, 0.5), (2.5, 0.5), (2.5, 2.5)])

    # Add locations
    w.add_location("table", "bathroom", Pose(x=-2.0, y=2.0))
    w.add_location("desk", "bedroom", Pose(x=3.0, y=3.0, yaw=0.2))

    return w


if __name__ == "__main__":
    w = create_world()
    app = PyRoboSim(w, sys.argv)
    sys.exit(app.exec_())
