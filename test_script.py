import sys

from gui.main import PyRoboSim
from world.utils import Pose
from world.robot import Robot
from world.room import Room
from world.world import World

r = Robot(pose=Pose(x=1.0, y=1.5), radius=0.2)

w = World(robot=r)

r1coords = [(0, 0), (1.5, 1.5), (1.5, 0)]
w.add_room(Room(r1coords, name="kitchen", color=[1, 0, 0]))
r2coords = [(1.75, 3), (3, 3), (3, 4), (1.75, 4)]
w.add_room(Room(r2coords, name="bedroom", color=[0, 0.6, 0]))
r3coords = [(-1, 1), (-1, 3), (-2.5, 3), (-2, 1)]
w.add_room(Room(r3coords, name="bathroom", color=[0, 0, 0.6]))

app = PyRoboSim(w, sys.argv)
sys.exit(app.exec_())
