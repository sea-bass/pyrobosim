import numpy as np
from PyQt5 import QtWidgets

from utils.pose import Pose
from gui.world import WorldGUI

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

        widget = QtWidgets.QWidget()

        # Push button
        self.push_button = QtWidgets.QPushButton("Randomize position")
        self.push_button.clicked.connect(self.on_click)

        # Animate button
        self.goal_layout = QtWidgets.QHBoxLayout()
        self.goal_layout.addWidget(QtWidgets.QLabel("Goal:"))
        self.goal_textbox = QtWidgets.QLineEdit()
        self.goal_layout.addWidget(self.goal_textbox)
        self.animate_button = QtWidgets.QPushButton("Animate path")
        self.animate_button.clicked.connect(self.on_animate_click)
        self.goal_layout.addWidget(self.animate_button)

        layout = QtWidgets.QVBoxLayout(widget)
        layout.addWidget(self.push_button)
        layout.addLayout(self.goal_layout)
        layout.addWidget(self.wg)
        
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

        self.wg.world.robot.set_pose(Pose(x=x, y=y, yaw=yaw))
        self.wg.update_robot_plot()
        self.wg.draw()

    def on_animate_click(self):
        print(f"Planning to {self.goal_textbox.text()}")
        p = self.wg.world.find_path(goal=self.goal_textbox.text())
        if p is not None:
            self.wg.animate_path(linear_velocity=1.0, dt=0.05)
