import numpy as np
from matplotlib.figure import Figure
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg

class WorldGUI(FigureCanvasQTAgg):
    def __init__(self, world, width=5, height=4, dpi=100):
        self.fig = Figure(figsize=(width, height), dpi=dpi)
        self.axes = self.fig.add_subplot(111)
        self.axes.set_title("World Model")

        self.world = world
        self.createRobot()

        super(WorldGUI, self).__init__(self.fig)

    def createRobot(self):
        L = 0.2
        p = self.world.robot.pose
        self.robot_body, = self.axes.plot(
            p.x, p.y, 
            "mo", markersize=10, markeredgewidth=2, 
            markerfacecolor="None")
        self.robot_dir, = self.axes.plot(
            p.x + np.array([0, L*np.cos(p.yaw)]),
            p.y + np.array([0, L*np.sin(p.yaw)]),
            "m-", linewidth=2)
