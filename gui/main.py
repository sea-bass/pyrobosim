import numpy as np
from PyQt5 import QtWidgets

from .world import WorldGUI

class PyRoboSim(QtWidgets.QApplication):
    def __init__(self, world, args):
        super(PyRoboSim, self).__init__(args)
        self.set_world(world)

    def set_world(self, world):
        self.ww = WorldWidget(world)
        self.ww.show()

class WorldWidget(QtWidgets.QMainWindow):
    def __init__(self, world, *args, **kwargs):
        super(WorldWidget, self).__init__(*args, **kwargs)
        self.setWindowTitle("pyrobosim")

        # Matplotlib stuff
        self.wg = WorldGUI(world, width=5, height=4, dpi=100)

        # Push button
        self.push_button = QtWidgets.QPushButton("Randomize position")
        self.push_button.clicked.connect(self.on_click)

        layout = QtWidgets.QVBoxLayout()
        layout.addWidget(self.push_button)
        layout.addWidget(self.wg)
        widget = QtWidgets.QWidget()
        widget.setLayout(layout)
        self.setCentralWidget(widget)

        self.wg.show()

    def on_click(self):
        xmin, xmax = self.wg.world.x_bounds
        ymin, ymax = self.wg.world.y_bounds
        r = self.wg.world.inflation_radius
        L = 0.1 * min(xmax-xmin, ymax-ymin)

        valid_pose = False
        while not valid_pose:
            x = (xmax - xmin - 2*r) * np.random.random() + xmin + r
            y = (ymax - ymin - 2*r) * np.random.random() + ymin + r
            yaw = 2.0 * np.pi * np.random.random()
            valid_pose = not self.wg.world.check_occupancy([x, y])

        self.wg.robot_body.set_xdata(x)
        self.wg.robot_body.set_ydata(y)
        self.wg.robot_dir.set_xdata(x + np.array([0, L*np.cos(yaw)]))
        self.wg.robot_dir.set_ydata(y + np.array([0, L*np.sin(yaw)]))
        self.wg.axes.autoscale()
        self.wg.draw()
