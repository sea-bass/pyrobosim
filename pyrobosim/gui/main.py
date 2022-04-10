import numpy as np
from PyQt5 import QtWidgets
from matplotlib.backends.qt_compat import QtCore
from matplotlib.backends.backend_qt5agg import NavigationToolbar2QT

from ..gui.world import WorldGUI
from ..utils.knowledge import query_to_entity


class PyRoboSim(QtWidgets.QApplication):
    """ Main pyrobosim GUI class. """
    def __init__(self, world, args):
        super(PyRoboSim, self).__init__(args)
        self.set_world(world)
        self.ww.show()

    def set_world(self, world):
        """
        Assigns a world model to the GUI.

        :param world: World object
        """
        self.ww = WorldWidget(world)


class WorldWidget(QtWidgets.QMainWindow):
    def __init__(self, world, *args, **kwargs):
        super(WorldWidget, self).__init__(*args, **kwargs)
        self.setWindowTitle("pyrobosim")
        self.set_window_dims()

        # Connect the GUI to the world
        self.world = world
        self.world.gui = self
        self.world.has_gui = True

        self.wg = WorldGUI(world, dpi=100)
        self.create_layout()
        self.update_manip_state()
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
        self.nav_toolbar = NavigationToolbar2QT(self.wg, self)
        self.addToolBar(QtCore.Qt.BottomToolBarArea, self.nav_toolbar)
        self.world_layout.addWidget(self.wg)

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
        """ Update the manipulation state to enable/disable buttons """
        can_pick = self.wg.world.has_robot and \
            self.wg.world.robot.manipulated_object is None
        self.pick_button.setEnabled(can_pick)
        self.place_button.setEnabled(not can_pick)

    def set_buttons_during_action(self, state):
        """ 
        Enables or disables buttons that should not be pressed while
        the robot is executing an action
        """
        self.nav_button.setEnabled(state)
        self.pick_button.setEnabled(state)
        self.place_button.setEnabled(state)
        self.rand_pose_button.setEnabled(state)

    ####################
    # Button Callbacks #
    ####################
    def rand_pose_cb(self):
        """ Callback to randomize robot pose """
        sampled_pose = self.wg.world.sample_free_robot_pose_uniform()
        if sampled_pose is not None:
            self.wg.world.robot.set_pose(sampled_pose)
        self.wg.update_robot_plot()
        self.wg.world.current_path = None
        self.wg.show_path()
        self.wg.show_world_state(navigating=True)
        self.wg.draw()

    def rand_goal_cb(self):
        """ Callback to randomize robot goal """
        all_entities = self.wg.world.get_location_names() + \
            self.wg.world.get_room_names()
        entity_name = np.random.choice(all_entities)
        self.goal_textbox.setText(entity_name)

    def rand_obj_cb(self):
        """ Callback to randomize manipulation object goal """
        obj_name = np.random.choice(self.wg.world.get_object_names())
        self.goal_textbox.setText(obj_name)

    def on_navigate_click(self):
        """ Callback to navigate to a goal location """
        if self.wg.world.robot and self.wg.world.robot.executing_action:
            return

        query_list = self.goal_textbox.text().split(" ")
        loc = query_to_entity(self.wg.world, query_list, mode="location",
                              resolution_strategy="nearest")
        if not loc:
            return
        
        print(f"Navigating to {loc.name}")
        self.set_buttons_during_action(False)
        self.wg.navigate(loc)
        self.set_buttons_during_action(True)

    def on_pick_click(self):
        """ Callback to pick an object """
        if self.wg.world.robot:
            loc = self.wg.world.robot.location
            query_list = [loc] + self.goal_textbox.text().split(" ")
            obj = query_to_entity(self.wg.world, query_list, mode="object",
                                  resolution_strategy="nearest")
            print(f"Picking {obj.name}")
            self.wg.pick_object(obj)
            self.update_manip_state()

    def on_place_click(self):
        """ Callback to place an object """
        if self.wg.world.robot:
            loc = self.wg.world.robot.location
            if self.wg.world.robot.manipulated_object is not None:
                print(f"Placing {self.wg.world.robot.manipulated_object.name}")
            self.wg.place_object(loc)
            self.update_manip_state()
