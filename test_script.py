import sys

from gui.main import PyRoboSim
from world.utils import Pose
from world.robot import Robot
from world.world import World

r = Robot(pose=Pose(x=1.0, y=1.5))
w = World(robot=r)
app = PyRoboSim(w, sys.argv)
sys.exit(app.exec_())
