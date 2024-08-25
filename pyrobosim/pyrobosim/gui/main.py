""" Main utilities for pyrobosim GUI. """

import numpy as np
import signal
import sys

from PySide6 import QtWidgets
from PySide6.QtGui import QFont, QScreen
from matplotlib.backends.qt_compat import QtCore
from matplotlib.backends.backend_qt5agg import NavigationToolbar2QT

from .world_canvas import WorldCanvas


def start_gui(world):
    """
    Helper function to start a pyrobosim GUI for a world model.

    :param world: World object to attach to the GUI.
    :type world: :class:`pyrobosim.core.world.World`
    """
    app = PyRoboSimGUI(world, sys.argv)
    signal.signal(signal.SIGINT, signal.SIG_DFL)
    sys.exit(app.exec_())


class PyRoboSimGUI(QtWidgets.QApplication):
    """Main pyrobosim GUI class."""

    def __init__(self, world, args, show=True):
        """
        Creates an instance of the pyrobosim GUI.

        :param world: World object to attach to the GUI.
        :type world: :class:`pyrobosim.core.world.World`
        :param args: System arguments, needed by the QApplication constructor.
        :type args: list[str]
        :param show: If true (default), shows the GUI. Otherwise runs headless for testing.
        :type show: bool, optional
        """
        super(PyRoboSimGUI, self).__init__(args)
        self.world = world
        self.main_window = PyRoboSimMainWindow(world, show)
        if show:
            self.main_window.show()


class PyRoboSimMainWindow(QtWidgets.QMainWindow):
    """Main application window for the pyrobosim GUI."""

    def __init__(self, world, show=True, *args, **kwargs):
        """
        Creates an instance of the pyrobosim application main window.

        :param world: World object to attach.
        :type world: :class:`pyrobosim.core.world.World`
        :param show: If true (default), shows the GUI. Otherwise runs headless for testing.
        :type show: bool, optional
        """
        super(PyRoboSimMainWindow, self).__init__(*args, **kwargs)
        self.setWindowTitle("pyrobosim")
        self.set_window_dims()

        # Connect the GUI to the world
        self.world = world
        self.world.gui = self
        self.world.has_gui = True

        self.layout_created = False
        self.canvas = WorldCanvas(self, world, show)
        self.create_layout()
        self.canvas.show()
        self.on_robot_changed()

    def closeEvent(self, _):
        """Cleans up running threads on closing the window."""
        self.canvas.nav_animator.stop()
        self.canvas.thread_pool.waitForDone()

    def set_window_dims(self, screen_fraction=0.8):
        """
        Set window dimensions.

        :param screen_fraction: Fraction of screen (0.0 to 1.0) used by window.
        :type screen_fraction: float
        """
        screen = QScreen.availableGeometry(QtWidgets.QApplication.primaryScreen())
        window_width = int(screen.width() * screen_fraction)
        window_height = int(screen.height() * screen_fraction)
        window_x = int(screen.left() + 0.5 * (screen.width() - window_width))
        window_y = int(screen.top() + 0.5 * (screen.height() - window_height))
        self.setGeometry(window_x, window_y, window_width, window_height)

    def create_layout(self):
        """Creates the main GUI layout."""
        self.main_widget = QtWidgets.QWidget()

        bold_font = QFont()
        bold_font.setBold(True)

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

        # Robot and goal query edit boxes
        self.queries_layout = QtWidgets.QHBoxLayout()

        robot_name_label = QtWidgets.QLabel("Robot name:")
        robot_name_label.setFont(bold_font)
        self.queries_layout.addWidget(robot_name_label)
        self.robot_textbox = QtWidgets.QComboBox()
        robot_names = [r.name for r in self.world.robots] + ["world"]
        self.robot_textbox.addItems(robot_names)
        self.robot_textbox.setEditable(True)
        self.robot_textbox.currentTextChanged.connect(self.on_robot_changed)
        self.queries_layout.addWidget(self.robot_textbox, 3)

        self.queries_layout.addWidget(QtWidgets.QWidget(), 1)  # Adds spacing

        goal_query_label = QtWidgets.QLabel("Goal query:")
        goal_query_label.setFont(bold_font)
        self.queries_layout.addWidget(goal_query_label)
        self.goal_textbox = QtWidgets.QLineEdit()
        self.queries_layout.addWidget(self.goal_textbox, 6)

        # Action buttons
        self.action_layout = QtWidgets.QGridLayout()
        self.nav_button = QtWidgets.QPushButton("Navigate")
        self.nav_button.clicked.connect(self.on_navigate_click)
        self.action_layout.addWidget(self.nav_button, 0, 0)
        self.pick_button = QtWidgets.QPushButton("Pick")
        self.pick_button.clicked.connect(self.on_pick_click)
        self.action_layout.addWidget(self.pick_button, 0, 1)
        self.place_button = QtWidgets.QPushButton("Place")
        self.place_button.clicked.connect(self.on_place_click)
        self.action_layout.addWidget(self.place_button, 0, 2)
        self.detect_button = QtWidgets.QPushButton("Detect")
        self.detect_button.clicked.connect(self.on_detect_click)
        self.action_layout.addWidget(self.detect_button, 1, 0)
        self.open_button = QtWidgets.QPushButton("Open")
        self.open_button.clicked.connect(self.on_open_click)
        self.action_layout.addWidget(self.open_button, 1, 1)
        self.close_button = QtWidgets.QPushButton("Close")
        self.close_button.clicked.connect(self.on_close_click)
        self.action_layout.addWidget(self.close_button, 1, 2)

        # World layout (Matplotlib affordances)
        self.world_layout = QtWidgets.QVBoxLayout()
        self.nav_toolbar = NavigationToolbar2QT(self.canvas, self)
        self.addToolBar(QtCore.Qt.BottomToolBarArea, self.nav_toolbar)
        self.world_layout.addWidget(self.canvas)

        # Other options
        self.other_options_layout = QtWidgets.QGridLayout()
        self.toggle_collision_polygons_checkbox = QtWidgets.QCheckBox(
            "Show collision polygons"
        )
        self.toggle_collision_polygons_checkbox.clicked.connect(
            self.on_collision_polygon_toggle_click
        )
        self.other_options_layout.addWidget(
            self.toggle_collision_polygons_checkbox, 0, 0
        )
        self.reset_path_planner_button = QtWidgets.QPushButton("Reset path planner")
        self.reset_path_planner_button.clicked.connect(self.on_reset_path_planner_click)
        self.other_options_layout.addWidget(self.reset_path_planner_button, 0, 1)
        self.cancel_action_button = QtWidgets.QPushButton("Cancel action")
        self.cancel_action_button.clicked.connect(self.on_cancel_action_click)
        self.other_options_layout.addWidget(self.cancel_action_button, 0, 2)

        # Main layout
        self.main_layout = QtWidgets.QVBoxLayout(self.main_widget)
        self.main_layout.addLayout(self.buttons_layout)
        self.main_layout.addLayout(self.queries_layout)
        self.main_layout.addLayout(self.action_layout)
        self.main_layout.addLayout(self.world_layout)
        self.main_layout.addLayout(self.other_options_layout)

        self.main_widget.setLayout(self.main_layout)
        self.setCentralWidget(self.main_widget)
        self.layout_created = True

    def get_current_robot(self):
        robot_name = self.robot_textbox.currentText()
        return self.world.get_robot_by_name(robot_name)

    ####################
    # State Management #
    ####################
    def update_button_state(self):
        """Update the state of buttons based on the state of the robot."""
        robot = self.get_current_robot()
        if robot:
            is_moving = robot.is_moving()
            at_open_object_spawn = robot.at_object_spawn() and robot.location.is_open
            can_pick = robot.manipulated_object is None
            can_open_close = robot.at_openable_location() and can_pick

            self.nav_button.setEnabled(not is_moving)
            self.pick_button.setEnabled(can_pick and at_open_object_spawn)
            self.place_button.setEnabled((not can_pick) and at_open_object_spawn)
            self.detect_button.setEnabled(at_open_object_spawn)
            self.open_button.setEnabled(can_open_close and not robot.location.is_open)
            self.close_button.setEnabled(can_open_close and robot.location.is_open)
            self.cancel_action_button.setEnabled(is_moving)
            self.reset_path_planner_button.setEnabled(not is_moving)

            self.canvas.show_world_state(robot, navigating=is_moving)
        else:
            self.nav_button.setEnabled(False)
            self.cancel_action_button.setEnabled(False)
            self.reset_path_planner_button.setEnabled(False)

        self.canvas.draw_signal.emit()

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
        self.detect_button.setEnabled(state)
        self.open_button.setEnabled(state)
        self.close_button.setEnabled(state)
        self.rand_pose_button.setEnabled(state)
        self.cancel_action_button.setEnabled(not state)
        self.reset_path_planner_button.setEnabled(state)

    ####################
    # Button Callbacks #
    ####################
    def rand_pose_cb(self):
        """Callback to randomize robot pose."""
        robot = self.get_current_robot()
        if not robot:
            print("No robot available.")
            return None

        sampled_pose = self.world.sample_free_robot_pose_uniform(
            robot, ignore_robots=False
        )
        if sampled_pose is not None:
            robot.set_pose(sampled_pose)
            if robot.manipulated_object is not None:
                robot.manipulated_object.pose = sampled_pose
        self.canvas.update_robots_plot()
        self.canvas.show_world_state(navigating=True)
        self.canvas.draw()

    def rand_goal_cb(self):
        """Callback to randomize robot goal."""
        all_entities = (
            self.world.get_location_names()
            + self.world.get_hallway_names()
            + self.world.get_room_names()
        )
        entity_name = np.random.choice(all_entities)
        self.goal_textbox.setText(entity_name)

    def rand_obj_cb(self):
        """Callback to randomize manipulation object goal."""
        obj_name = np.random.choice(self.world.get_object_names())
        self.goal_textbox.setText(obj_name)

    def on_robot_changed(self):
        """Callback when the currently selected robot changes."""
        self.canvas.show_objects_signal.emit()
        self.update_button_state()

    def on_navigate_click(self):
        """Callback to navigate to a goal location."""
        robot = self.get_current_robot()
        if robot and robot.executing_action:
            return

        loc = self.goal_textbox.text()
        if loc:
            print(f"[{robot.name}] Navigating to {loc}")
            self.canvas.navigate_signal.emit(robot, loc, None)

    def on_pick_click(self):
        """Callback to pick an object."""
        robot = self.get_current_robot()
        if robot:
            obj = self.goal_textbox.text()
            print(f"[{robot.name}] Picking {obj}")
            self.canvas.pick_object(robot, obj)
            self.update_button_state()

    def on_place_click(self):
        """Callback to place an object."""
        robot = self.get_current_robot()
        if robot and robot.manipulated_object is not None:
            print(f"[{robot.name}] Placing {robot.manipulated_object.name}")
            self.canvas.place_object(robot)
            self.update_button_state()

    def on_detect_click(self):
        """Callback to detect objects."""
        robot = self.get_current_robot()
        if robot:
            print(f"[{robot.name}] Detecting objects")
            obj_query = self.goal_textbox.text() or None
            self.canvas.detect_objects(robot, obj_query)
            self.update_button_state()

    def on_open_click(self):
        """Callback to open a location."""
        robot = self.get_current_robot()
        if robot and robot.location:
            print(f"[{robot.name}] Opening {robot.location}")
            self.canvas.open_location(robot)
            self.update_button_state()

    def on_close_click(self):
        """Callback to close a location."""
        robot = self.get_current_robot()
        if robot and robot.location:
            print(f"[{robot.name}] Closing {robot.location}")
            self.canvas.close_location(robot)
            self.update_button_state()

    def on_collision_polygon_toggle_click(self):
        """Callback to toggle collision polygons."""
        self.canvas.toggle_collision_polygons()
        self.canvas.draw_signal.emit()

    def on_cancel_action_click(self):
        """Callback to cancel any running action for the current robot."""
        robot = self.get_current_robot()
        if robot:
            robot.cancel_actions()
            self.canvas.draw_signal.emit()

    def on_reset_path_planner_click(self):
        """Callback to reset the path planner for the current robot."""
        robot = self.get_current_robot()
        if robot:
            robot.reset_path_planner()
            self.canvas.draw_signal.emit()
