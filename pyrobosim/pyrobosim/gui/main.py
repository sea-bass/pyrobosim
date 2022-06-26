""" Main utilities for pyrobosim GUI. """

import numpy as np
from PyQt5 import QtWidgets
from matplotlib.backends.qt_compat import QtCore
from matplotlib.backends.backend_qt5agg import NavigationToolbar2QT

from .world_canvas import WorldCanvas
from ..utils.knowledge import query_to_entity


class PyRoboSimGUI(QtWidgets.QApplication):
    """Main pyrobosim GUI class."""

    def __init__(self, world, args):
        """
        Creates an instance of the pyrobosim GUI.

        :param world: World object to attach to the GUI.
        :type world: :class:`pyrobosim.core.world.World`
        """
        super(PyRoboSimGUI, self).__init__(args)
        self.world = world
        self.main_window = PyRoboSimMainWindow(world)
        self.main_window.show()


class PyRoboSimMainWindow(QtWidgets.QMainWindow):
    """Main application window for the pyrobosim GUI."""

    def __init__(self, world, *args, **kwargs):
        """
        Creates an instance of the pyrobosim application main window.

        :param world: World object to attach
        :type world: :class:`pyrobosim.core.world.World`
        """
        super(PyRoboSimMainWindow, self).__init__(*args, **kwargs)
        self.setWindowTitle("pyrobosim")
        self.set_window_dims()

        # Connect the GUI to the world
        self.world = world
        self.world.gui = self
        self.world.has_gui = True

        self.canvas = WorldCanvas(world)
        self.create_layout()
        self.update_manip_state()
        self.canvas.show()

    def set_window_dims(self, screen_fraction=0.8):
        """
        Set window dimensions.

        :param screen_fraction: Fraction of screen (0.0 to 1.0) used by window.
        :type screen_fraction: float
        """
        screen = QtWidgets.QDesktopWidget().availableGeometry()
        window_width = int(screen.width() * screen_fraction)
        window_height = int(screen.height() * screen_fraction)
        window_x = int(screen.left() + 0.5 * (screen.width() - window_width))
        window_y = int(screen.top() + 0.5 * (screen.height() - window_height))
        self.setGeometry(window_x, window_y, window_width, window_height)

    def create_layout(self):
        """Creates the main GUI layout."""
        self.main_widget = QtWidgets.QWidget()

        # Push buttons
        self.buttons_layout = QtWidgets.QHBoxLayout()
        self.rand_pose_button = QtWidgets.QPushButton("Randomize robot pose")
        self.rand_pose_button.clicked.connect(self.rand_pose_cb)
        self.buttons_layout.addWidget(self.rand_pose_button)
        self.rand_goal_button = QtWidgets.QPushButton("Randomize nav goal")
        self.rand_goal_button.clicked.connect(self.rand_goal_cb)
        self.buttons_layout.addWidget(self.rand_goal_button)
        self.rand_obj_button = QtWidgets.QPushButton("Randomize target object")
        self.rand_obj_button.clicked.connect(self.rand_obj_cb)
        self.buttons_layout.addWidget(self.rand_obj_button)

        # Navigation buttons
        self.goal_layout = QtWidgets.QHBoxLayout()
        self.goal_layout.addWidget(QtWidgets.QLabel("Goal query:"))
        self.goal_textbox = QtWidgets.QLineEdit()
        self.goal_layout.addWidget(self.goal_textbox)

        # Action buttons
        self.action_layout = QtWidgets.QHBoxLayout()
        self.nav_button = QtWidgets.QPushButton("Navigate")
        self.nav_button.clicked.connect(self.on_navigate_click)
        self.action_layout.addWidget(self.nav_button)
        self.pick_button = QtWidgets.QPushButton("Pick")
        self.pick_button.clicked.connect(self.on_pick_click)
        self.action_layout.addWidget(self.pick_button)
        self.place_button = QtWidgets.QPushButton("Place")
        self.place_button.clicked.connect(self.on_place_click)
        self.action_layout.addWidget(self.place_button)

        # World layout (Matplotlib affordances)
        self.world_layout = QtWidgets.QVBoxLayout()
        self.nav_toolbar = NavigationToolbar2QT(self.canvas, self)
        self.addToolBar(QtCore.Qt.BottomToolBarArea, self.nav_toolbar)
        self.world_layout.addWidget(self.canvas)

        # Main layout
        self.main_layout = QtWidgets.QVBoxLayout(self.main_widget)
        self.main_layout.addLayout(self.buttons_layout)
        self.main_layout.addLayout(self.goal_layout)
        self.main_layout.addLayout(self.action_layout)
        self.main_layout.addLayout(self.world_layout)

        self.main_widget.setLayout(self.main_layout)
        self.setCentralWidget(self.main_widget)

    ####################
    # State Management #
    ####################
    def update_manip_state(self):
        """Update the manipulation state to enable/disable buttons."""
        can_pick = self.world.has_robot and not self.world.robot.manipulated_object
        self.pick_button.setEnabled(can_pick)
        self.place_button.setEnabled(not can_pick)

    def set_buttons_during_action(self, state):
        """
        Enables or disables buttons that should not be pressed while
        the robot is executing an action.

        :param state: Desired button state (True to enable, False to disable)
        :type state: bool
        """
        self.nav_button.setEnabled(state)
        self.pick_button.setEnabled(state)
        self.place_button.setEnabled(state)
        self.rand_pose_button.setEnabled(state)

    ####################
    # Button Callbacks #
    ####################
    def rand_pose_cb(self):
        """Callback to randomize robot pose."""
        sampled_pose = self.world.sample_free_robot_pose_uniform()
        if sampled_pose is not None:
            self.world.robot.set_pose(sampled_pose)
            if self.world.robot.manipulated_object is not None:
                self.world.robot.manipulated_object.pose = sampled_pose
        self.canvas.update_robot_plot()
        self.canvas.show_world_state(navigating=True)
        self.canvas.draw()

    def rand_goal_cb(self):
        """Callback to randomize robot goal."""
        all_entities = self.world.get_location_names() + self.world.get_room_names()
        entity_name = np.random.choice(all_entities)
        self.goal_textbox.setText(entity_name)

    def rand_obj_cb(self):
        """Callback to randomize manipulation object goal."""
        obj_name = np.random.choice(self.world.get_object_names())
        self.goal_textbox.setText(obj_name)

    def on_navigate_click(self):
        """Callback to navigate to a goal location."""
        if self.world.robot and self.world.robot.executing_action:
            return

        query_list = self.goal_textbox.text().split(" ")
        loc = query_to_entity(
            self.world, query_list, mode="location", resolution_strategy="nearest"
        )
        if not loc:
            return

        print(f"Navigating to {loc.name}")
        self.set_buttons_during_action(False)
        self.canvas.navigate(loc)
        self.set_buttons_during_action(True)

    def on_pick_click(self):
        """Callback to pick an object."""
        if self.world.robot:
            loc = self.world.robot.location
            query_list = [loc] + self.goal_textbox.text().split(" ")
            obj = query_to_entity(
                self.world, query_list, mode="object", resolution_strategy="nearest"
            )
            print(f"Picking {obj.name}")
            self.canvas.pick_object(obj)
            self.update_manip_state()

    def on_place_click(self):
        """Callback to place an object."""
        if self.world.robot:
            if self.world.robot.manipulated_object is not None:
                print(f"Placing {self.world.robot.manipulated_object.name}")
            self.canvas.place_object()
            self.update_manip_state()
