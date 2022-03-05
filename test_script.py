"""
Test script showing how to build a world and use it with pyrobosim
"""

import sys
import numpy as np

from gui.main import PyRoboSim
from world.utils import Pose
from world.robot import Robot
from world.room import Room
from world.world import World


def create_world():
    # Create a robot and a world
    r = Robot(pose=Pose(x=1.0, y=1.5), radius=0.1)
    w = World(robot=r)
    w.set_metadata(locations="data/example_location_data.yaml", 
                   objects="data/example_object_data.yaml")

    # Add rooms
    r1coords = [(-1, -1), (1.5, -1), (1.5, 1.5), (0.5, 1.5)]
    w.add_room(Room(r1coords, name="kitchen", color=[1, 0, 0]))
    r2coords = [(1.75, 2.5), (3.5, 2.5), (3.5, 4), (1.75, 4)]
    w.add_room(Room(r2coords, name="bedroom", color=[0, 0.6, 0]))
    r3coords = [(-1, 1), (-1, 3.5), (-3.0, 3.5), (-2.5, 1)]
    w.add_room(Room(r3coords, name="bathroom", color=[0, 0, 0.6]))

    # Add hallways between the rooms
    w.add_hallway("kitchen", "bathroom", width=0.7)
    w.add_hallway("bathroom", "bedroom", width=0.5,
                  conn_method="angle", conn_angle=0, offset=0.8)
    w.add_hallway("kitchen", "bedroom", width=0.6,
                  conn_method="points",
                  conn_points=[(1.0, 0.5), (2.5, 0.5), (2.5, 3.0)])

    # Add locations
    table = w.add_location("table", "kitchen", Pose(x=0.85, y=-0.5, yaw=-np.pi/2))
    desk = w.add_location("desk", "bedroom", Pose(x=3.15, y=3.65))
    counter = w.add_location("counter", "bathroom", Pose(x=-2.45, y=2.5, yaw=np.pi/2 + np.pi/16))

    # Add objects
    w.add_object("banana", table, pose=Pose(x=1.0, y=-0.5, yaw=np.pi/4))
    w.add_object("apple", desk, pose=Pose(x=3.2, y=3.5))
    w.add_object("apple", table)
    w.add_object("apple", table)
    w.add_object("water", counter)
    w.add_object("banana", counter)
    w.add_object("water", desk)

    # Create a search graph
    w.create_search_graph(max_edge_dist=3.0, collision_check_dist=0.05, autoconnect=True)

    return w


if __name__ == "__main__":
    w = create_world()
    app = PyRoboSim(w, sys.argv)
    sys.exit(app.exec_())
