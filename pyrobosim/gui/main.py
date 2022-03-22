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
        self.set_window_dims()

        self.wg = WorldGUI(world, dpi=100)
        self.create_layout()
        self.wg.show()


    def set_window_dims(self):
        """ Set window dimensions """
        screen_percent = 0.8
        screen = QtWidgets.QDesktopWidget().availableGeometry()
        window_width = screen.width() * screen_percent
        window_height = screen.height() * screen_percent
        window_x = screen.left() + 0.5 * (screen.width() - window_width)
        window_y = screen.top() + 0.5 * (screen.height() - window_height)
        self.setGeometry(window_x, window_y, window_width, window_height)


    def create_layout(self):
        """ Creates the GUI layout """
        self.main_widget = QtWidgets.QWidget()

        # Push buttons
        self.buttons_layout = QtWidgets.QHBoxLayout()
        self.rand_pose_button = QtWidgets.QPushButton("Randomize position")
        self.rand_pose_button.clicked.connect(self.rand_pose_cb)
        self.buttons_layout.addWidget(self.rand_pose_button)
        self.rand_goal_button = QtWidgets.QPushButton("Randomize nav goal")
        self.rand_goal_button.clicked.connect(self.rand_goal_cb)
        self.buttons_layout.addWidget(self.rand_goal_button)
        self.rand_obj_button = QtWidgets.QPushButton("Randomize target object")
        self.rand_obj_button.clicked.connect(self.rand_obj_cb)
        self.buttons_layout.addWidget(self.rand_obj_button)

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

        # Main layout
        self.main_layout = QtWidgets.QVBoxLayout(self.main_widget)
        self.main_layout.addLayout(self.buttons_layout)
        self.main_layout.addLayout(self.nav_layout)
        self.main_layout.addLayout(self.manip_layout)
        self.main_layout.addWidget(self.wg)
        
        self.main_widget.setLayout(self.main_layout)
        self.setCentralWidget(self.main_widget)

    ####################
    # Button Callbacks #
    ####################
    def rand_pose_cb(self):
        """ Callback to randomize robot pose """
        xmin, xmax = self.wg.world.x_bounds
        ymin, ymax = self.wg.world.y_bounds
        r = self.wg.world.inflation_radius

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
        entity_name = np.random.choice(all_entities)
        self.nav_goal_textbox.setText(entity_name)

    def rand_obj_cb(self):
        """ Callback to randomize manipulation object goal """
        obj_name = np.random.choice(self.wg.world.get_object_names())
        self.manip_obj_textbox.setText(obj_name)

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
