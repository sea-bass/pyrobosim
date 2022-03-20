import numpy as np
from PyQt5 import QtWidgets

from ..utils.pose import Pose
from ..gui.world import WorldGUI

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

        # Push buttons
        self.buttons_layout = QtWidgets.QHBoxLayout()
        self.rand_pose_button = QtWidgets.QPushButton("Randomize position")
        self.rand_pose_button.clicked.connect(self.rand_pose_cb)
        self.buttons_layout.addWidget(self.rand_pose_button)
        self.rand_goal_button = QtWidgets.QPushButton("Randomize goal")
        self.rand_goal_button.clicked.connect(self.rand_goal_cb)
        self.buttons_layout.addWidget(self.rand_goal_button)

        # Navigation buttons
        self.nav_layout = QtWidgets.QHBoxLayout()
        self.nav_layout.addWidget(QtWidgets.QLabel("Navigation Goal:"))
        self.nav_goal_textbox = QtWidgets.QLineEdit()
        self.nav_layout.addWidget(self.nav_goal_textbox)
        self.nav_button = QtWidgets.QPushButton("Navigate")
        self.nav_button.clicked.connect(self.on_navigate_click)
        self.nav_layout.addWidget(self.nav_button)

        # Manipulation buttons
        self.manip_layout = QtWidgets.QHBoxLayout()
        self.manip_layout.addWidget(QtWidgets.QLabel("Manipulation Goal:"))
        self.manip_obj_textbox = QtWidgets.QLineEdit()
        self.manip_layout.addWidget(self.manip_obj_textbox)
        self.pick_button = QtWidgets.QPushButton("Pick")
        self.pick_button.clicked.connect(self.on_pick_click)
        self.manip_layout.addWidget(self.pick_button)
        self.place_button = QtWidgets.QPushButton("Place")
        self.place_button.clicked.connect(self.on_place_click)
        self.manip_layout.addWidget(self.place_button)

        layout = QtWidgets.QVBoxLayout(widget)
        layout.addLayout(self.buttons_layout)
        layout.addLayout(self.nav_layout)
        layout.addLayout(self.manip_layout)
        layout.addWidget(self.wg)
        
        widget.setLayout(layout)
        self.setCentralWidget(widget)

        self.wg.show()

    def rand_pose_cb(self):
        """ Callback to randomize robot pose """
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

    def rand_goal_cb(self):
        """ Callback to randomize robot goal """
        all_entities = self.wg.world.get_object_names() + \
            self.wg.world.get_location_names() + \
            self.wg.world.get_room_names()
        obj_name = np.random.choice(all_entities)
        self.nav_goal_textbox.setText(obj_name)

    def on_navigate_click(self):
        print(f"Planning to {self.nav_goal_textbox.text()}")
        p = self.wg.world.find_path(goal=self.nav_goal_textbox.text())
        if p is not None:
            self.wg.animate_path(linear_velocity=1.0, dt=0.05)

    # TODO: Keep track of state and gray out the buttons
    def on_pick_click(self):
        obj_name = self.manip_obj_textbox.text()
        print(f"Picking {obj_name}")
        self.wg.pick_object(obj_name)

    def on_place_click(self):
        obj_name = self.manip_obj_textbox.text()
        # TODO: Get location name from state of world
        loc_name = self.nav_goal_textbox.text()
        print(f"Placing {obj_name} in {loc_name}")
        self.wg.place_object(obj_name, loc_name)
