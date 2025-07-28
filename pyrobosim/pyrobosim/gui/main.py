"""Main utilities for PyRoboSim GUI."""

import numpy as np
import signal
import sys
from typing import Any

from typing import Callable
from PySide6 import QtWidgets
from PySide6.QtCore import Signal, QEvent, Qt
from PySide6.QtGui import QFont, QScreen
from matplotlib.backends.qt_compat import QtCore
from matplotlib.backends.backend_qt5agg import NavigationToolbar2QT

from ..core.robot import Robot
from ..core.world import World


def start_gui(world: World) -> None:
    """
    Helper function to start a PyRoboSim GUI for a world model.

    :param world: World object to attach to the GUI.
    """
    app = PyRoboSimGUI(world, sys.argv)
    signal.signal(signal.SIGINT, signal.SIG_DFL)
    sys.exit(app.exec_())


class PyRoboSimGUI(QtWidgets.QApplication):  # type: ignore [misc]
    """Main PyRoboSim GUI class."""

    def __init__(self, world: World, args: Any, show: bool = True) -> None:
        """
        Creates an instance of the PyRoboSim GUI.

        :param world: World object to attach to the GUI.
        :param args: System arguments, needed by the QApplication constructor.
        :param show: If true (default), shows the GUI. Otherwise runs headless for testing.
        """
        super(PyRoboSimGUI, self).__init__(args)
        self.world = world
        self.main_window = PyRoboSimMainWindow(self.world, show)
        if show:
            self.main_window.show()


class PyRoboSimMainWindow(QtWidgets.QMainWindow):  # type: ignore [misc]
    """Main application window for the PyRoboSim GUI."""

    update_buttons_signal = Signal()
    """Signal for updating UI button state."""

    def __init__(
        self, world: World, show: bool = True, *args: Any, **kwargs: Any
    ) -> None:
        """
        Creates an instance of the PyRoboSim application main window.

        :param world: World object to attach.
        :param show: If true (default), shows the GUI. Otherwise runs headless for testing.
        """
        from .world_canvas import WorldCanvas

        super(PyRoboSimMainWindow, self).__init__(*args, **kwargs)
        self.layout_created = False

        self.setWindowTitle("PyRoboSim")
        self.set_window_dims()

        self.canvas = WorldCanvas(self, world, show)
        self.set_world(world)

        self.create_layout()
        self.canvas.show()
        self.on_robot_changed()
        self.world.logger.info(f"Initialized PyRoboSim GUI.")

    def set_world(self, world: World) -> None:
        """
        Sets the world for the GUI.

        :param world: World object to attach.
        """
        self.world = world
        self.world.gui = self
        self.canvas.world = world

    def closeEvent(self, _: QEvent) -> None:
        """Cleans up running threads on closing the window."""
        self.canvas.nav_animator.stop()
        self.canvas.thread_pool.waitForDone()
        if self.world and (self.world.ros_node is not None):
            self.world.ros_node.shutdown()

    def set_window_dims(self, screen_fraction: float = 0.8) -> None:
        """
        Set window dimensions.

        :param screen_fraction: Fraction of screen (0.0 to 1.0) used by window.
        """
        screen = QScreen.availableGeometry(QtWidgets.QApplication.primaryScreen())
        window_width = int(screen.width() * screen_fraction)
        window_height = int(screen.height() * screen_fraction)
        window_x = int(screen.left() + 0.5 * (screen.width() - window_width))
        window_y = int(screen.top() + 0.5 * (screen.height() - window_height))
        self.setGeometry(window_x, window_y, window_width, window_height)

    def _add_checkbox(
        self, label: str, default_state: bool, slot: Callable[[int], None]
    ) -> QtWidgets.QCheckBox:
        """
        Helper function to add visibility toggles of room/location/object/robot names

        :param label: Text label to display in the UI.
        :param default_state: default state of the checkbox (True/False).
        :param slot: Callback function to connect to the checkbox's stateChanged signal.

        :return: The created QCheckBox instance for access elsewhere if required.
        """
        checkbox = QtWidgets.QCheckBox(label)
        checkbox.setChecked(default_state)
        checkbox.stateChanged.connect(slot)

        action = QtWidgets.QWidgetAction(self.visibility_menu)
        action.setDefaultWidget(checkbox)
        self.visibility_menu.addAction(action)
        return checkbox

    def create_layout(self) -> None:
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
        self.visibility_layout = QtWidgets.QHBoxLayout()
        self.toggle_menu_button = QtWidgets.QToolButton()
        self.toggle_menu_button.setText("Visibility Controls")
        self.toggle_menu_button.setPopupMode(
            QtWidgets.QToolButton.ToolButtonPopupMode.InstantPopup
        )
        self.visibility_menu = QtWidgets.QMenu()

        self.visibility_dropdown = QtWidgets.QFrame(self)
        self.visibility_dropdown.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.visibility_dropdown.setWindowFlags(Qt.Popup)
        self.visibility_dropdown.setLayout(QtWidgets.QVBoxLayout())

        self.show_collision_action = self._add_checkbox(
            "Show collision polygons", False, self.on_toggle_collision_polygons
        )
        self.show_room_names_checkbox = self._add_checkbox(
            "Show room names", True, self.on_toggle_room_names
        )
        self.show_object_names_checkbox = self._add_checkbox(
            "Show object names", True, self.on_toggle_object_names
        )
        self.show_location_names_checkbox = self._add_checkbox(
            "Show location names", True, self.on_toggle_location_names
        )
        self.show_robot_names_checkbox = self._add_checkbox(
            "Show robot names", True, self.on_toggle_robot_names
        )

        self.toggle_menu_button.setMenu(self.visibility_menu)
        self.visibility_layout.addWidget(
            self.toggle_menu_button, alignment=Qt.AlignmentFlag.AlignLeft
        )
        visibility_widget = QtWidgets.QWidget()
        visibility_widget.setLayout(self.visibility_layout)
        self.other_options_layout.addWidget(visibility_widget, 0, 0)

        self.reset_world_button = QtWidgets.QPushButton("Reset world")
        self.reset_world_button.clicked.connect(self.on_reset_world_click)
        self.other_options_layout.addWidget(self.reset_world_button, 0, 1)
        self.reset_path_planner_button = QtWidgets.QPushButton("Reset path planner")
        self.reset_path_planner_button.clicked.connect(self.on_reset_path_planner_click)
        self.other_options_layout.addWidget(self.reset_path_planner_button, 0, 2)
        self.cancel_action_button = QtWidgets.QPushButton("Cancel action")
        self.cancel_action_button.clicked.connect(self.on_cancel_action_click)
        self.other_options_layout.addWidget(self.cancel_action_button, 0, 3)

        # Main layout
        self.main_layout = QtWidgets.QVBoxLayout(self.main_widget)
        self.main_layout.addLayout(self.buttons_layout)
        self.main_layout.addLayout(self.queries_layout)
        self.main_layout.addLayout(self.action_layout)
        self.main_layout.addLayout(self.world_layout)
        self.main_layout.addLayout(self.other_options_layout)

        self.update_buttons_signal.connect(self.update_button_state)

        self.main_widget.setLayout(self.main_layout)
        self.setCentralWidget(self.main_widget)
        self.layout_created = True

    def get_current_robot(self) -> Robot | None:
        robot_name = self.robot_textbox.currentText()
        return self.world.get_robot_by_name(robot_name)

    ####################
    # State Management #
    ####################
    def update_button_state(self) -> None:
        """Update the state of buttons based on the state of the robot."""
        robot = self.get_current_robot()
        if robot is not None:
            is_moving = robot.is_moving()
            is_location_open = (robot.location is not None) and (robot.location.is_open)

            at_open_object_spawn = robot.at_object_spawn() and is_location_open
            can_pick = robot.manipulated_object is None
            can_open_close = robot.at_openable_location() and can_pick

            self.nav_button.setEnabled(not is_moving)
            self.pick_button.setEnabled(can_pick and at_open_object_spawn)
            self.place_button.setEnabled((not can_pick) and at_open_object_spawn)
            self.detect_button.setEnabled(at_open_object_spawn)
            self.open_button.setEnabled(can_open_close and not is_location_open)
            self.close_button.setEnabled(can_open_close and is_location_open)
            self.cancel_action_button.setEnabled(is_moving)
            self.reset_world_button.setEnabled(not is_moving)
            self.reset_path_planner_button.setEnabled(not is_moving)
            self.rand_pose_button.setEnabled(not is_moving)

            self.canvas.show_world_state(robot)
        else:
            self.nav_button.setEnabled(False)
            self.pick_button.setEnabled(False)
            self.place_button.setEnabled(False)
            self.detect_button.setEnabled(False)
            self.cancel_action_button.setEnabled(False)
            self.open_button.setEnabled(True)
            self.close_button.setEnabled(True)
            self.reset_world_button.setEnabled(True)
            self.reset_path_planner_button.setEnabled(False)
            self.rand_pose_button.setEnabled(False)

        self.canvas.draw_signal.emit()

    def set_buttons_during_action(self, state: bool) -> None:
        """
        Enables or disables buttons that should not be pressed while
        the robot is executing an action.

        :param state: Desired button state (True to enable, False to disable)
        """
        if self.get_current_robot() is not None:
            self.nav_button.setEnabled(state)
            self.pick_button.setEnabled(state)
            self.place_button.setEnabled(state)
            self.detect_button.setEnabled(state)
            self.open_button.setEnabled(state)
            self.close_button.setEnabled(state)
            self.rand_pose_button.setEnabled(state)
            self.cancel_action_button.setEnabled(not state)
            self.reset_world_button.setEnabled(state)
            self.reset_path_planner_button.setEnabled(state)

    ####################
    # Button Callbacks #
    ####################
    def rand_pose_cb(self) -> None:
        """Callback to randomize robot pose."""
        robot = self.get_current_robot()
        if robot is None:
            self.world.logger.warning("No robot available.")
            return None

        sampled_pose = self.world.sample_free_robot_pose_uniform(
            robot, ignore_robots=False
        )
        if sampled_pose is not None:
            robot.set_pose(sampled_pose)
            if robot.manipulated_object is not None:
                robot.manipulated_object.pose = sampled_pose

        self.canvas.update_robots_plot()
        self.canvas.show_world_state()
        self.canvas.draw()

    def rand_goal_cb(self) -> None:
        """Callback to randomize robot goal."""
        all_entities = (
            self.world.get_location_names()
            + self.world.get_hallway_names()
            + self.world.get_room_names()
        )
        entity_name = np.random.choice(all_entities)
        self.goal_textbox.setText(entity_name)

    def rand_obj_cb(self) -> None:
        """Callback to randomize manipulation object goal."""
        obj_name = np.random.choice(self.world.get_object_names())
        self.goal_textbox.setText(obj_name)

    def on_robot_changed(self) -> None:
        """Callback when the currently selected robot changes."""
        self.canvas.show_objects_signal.emit()
        self.canvas.show_hallways_signal.emit()
        self.update_buttons_signal.emit()

    def on_navigate_click(self) -> None:
        """Callback to navigate to a goal location."""
        robot = self.get_current_robot()
        if (robot is None) or robot.executing_action:
            return

        loc = self.goal_textbox.text()
        if loc is not None:
            robot.logger.info(f"Navigating to {loc}")
            self.canvas.navigate_signal.emit(robot, loc, None, 1.0)

    def on_pick_click(self) -> None:
        """Callback to pick an object."""
        robot = self.get_current_robot()
        if robot is not None:
            obj = self.goal_textbox.text()
            robot.logger.info(f"Picking {obj}")
            self.canvas.pick_object(robot, obj)
            self.update_buttons_signal.emit()

    def on_place_click(self) -> None:
        """Callback to place an object."""
        robot = self.get_current_robot()
        if (robot is not None) and (robot.manipulated_object is not None):
            robot.logger.info(f"Placing {robot.manipulated_object.name}")
            self.canvas.place_object(robot)
            self.update_buttons_signal.emit()

    def on_detect_click(self) -> None:
        """Callback to detect objects."""
        robot = self.get_current_robot()
        if robot is not None:
            obj_query = self.goal_textbox.text() or None
            log_message = "Detecting objects"
            if obj_query is not None:
                log_message += f" with query: {obj_query}"
            robot.logger.info(log_message)
            self.canvas.detect_objects(robot, obj_query)
            self.update_buttons_signal.emit()

    def on_open_click(self) -> None:
        """Callback to open a location."""
        robot = self.get_current_robot()
        if (robot is not None) and (robot.location is not None):
            robot.logger.info(f"Opening {robot.location}")
            self.canvas.open_location(robot)
            self.update_buttons_signal.emit()
        elif (robot is None) and self.goal_textbox.text():
            self.world.open_location(self.goal_textbox.text())

    def on_close_click(self) -> None:
        """Callback to close a location."""
        robot = self.get_current_robot()
        if (robot is not None) and (robot.location is not None):
            robot.logger.info(f"Closing {robot.location}")
            self.canvas.close_location(robot)
            self.update_buttons_signal.emit()
        elif (robot is None) and self.goal_textbox.text():
            self.world.close_location(self.goal_textbox.text())

    def on_toggle_collision_polygons(self, state: int) -> None:
        """
        Callback to toggle collision polygons.

        :param state: Integer state of the checkbox (0=unchecked, 2=checked)
        """
        self.canvas.toggle_collision_polygons()
        self.canvas.draw_signal.emit()

    def on_toggle_room_names(self, state: int) -> None:
        """
        Callback to toggle room name visibility.

        :param state: Integer state of the checkbox (0=unchecked, 2=checked)
        """
        self.canvas.toggle_room_names()
        self.canvas.draw_signal.emit()

    def on_toggle_object_names(self, state: int) -> None:
        """
        Callback to toggle object name visibility.

        :param state: Integer state of the checkbox (0=unchecked, 2=checked)
        """
        self.canvas.toggle_object_names()
        self.canvas.draw_signal.emit()

    def on_toggle_location_names(self, state: int) -> None:
        """
        Callback to toggle location name visibility.

        :param state: Integer state of the checkbox (0=unchecked, 2=checked)
        """
        self.canvas.toggle_location_names()
        self.canvas.draw_signal.emit()

    def on_toggle_robot_names(self, state: int) -> None:
        """
        Callback to toggle robot name visibility.

        :param state: Integer state of the checkbox (0=unchecked, 2=checked)
        """
        self.canvas.toggle_robot_names()
        self.canvas.draw_signal.emit()

    def on_cancel_action_click(self) -> None:
        """Callback to cancel any running action for the current robot."""
        robot = self.get_current_robot()
        if robot is not None:
            robot.cancel_actions()
            self.update_buttons_signal.emit()
            self.canvas.draw_signal.emit()

    def on_reset_world_click(self) -> None:
        """Callback to reset the entire world."""
        self.world.reset()

    def on_reset_path_planner_click(self) -> None:
        """Callback to reset the path planner for the current robot."""
        robot = self.get_current_robot()
        if robot is not None:
            robot.reset_path_planner()
            self.canvas.draw_signal.emit()
