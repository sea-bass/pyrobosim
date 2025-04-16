"""Utilities for displaying a PyRoboSim world in a figure canvas."""

import adjustText
import numpy as np
import time
import threading
import matplotlib.pyplot as plt
from matplotlib.artist import Artist
from matplotlib.figure import Figure
from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg
from matplotlib.lines import Line2D
from matplotlib.patches import PathPatch
from matplotlib.pyplot import Circle
from matplotlib.text import Text
from matplotlib.transforms import Affine2D
from PySide6.QtCore import QThreadPool, QTimer, Signal

from .action_runners import (
    NavRunner,
    PickRunner,
    PlaceRunner,
    DetectRunner,
    OpenRunner,
    CloseRunner,
)
from .main import PyRoboSimMainWindow
from ..core.objects import Object
from ..core.robot import Robot
from ..core.world import World
from ..navigation.visualization import plot_path_planner
from ..utils.path import Path
from ..utils.pose import Pose


class WorldCanvas(FigureCanvasQTAgg):  # type: ignore [misc]
    """
    Canvas for rendering a PyRoboSim world as a MatPlotLib figure in an
    application.
    """

    # Visualization constants
    object_zorder = 3
    """ zorder for object visualization. """
    robot_zorder = 3
    """ zorder for robot visualization. """
    robot_dir_line_factor = 3.0
    """ Multiplier of robot radius for plotting robot orientation lines. """

    draw_lock = threading.RLock()
    """ Lock for drawing on the canvas in a thread-safe manner. """

    draw_signal = Signal()
    """ Signal for drawing without threading errors. """

    navigate_signal = Signal(Robot, str, Path)
    """ Signal for starting a navigation task without threading errors. """

    show_hallways_signal = Signal()
    """ Signal for showing hallways without threading errors. """

    show_locations_signal = Signal()
    """ Signal for showing locations without threading errors. """

    show_objects_signal = Signal()
    """ Signal for showing objects without threading errors. """

    show_robots_signal = Signal()
    """ Signal for showing robots without threading errors. """

    show_planner_and_path_signal = Signal(Robot, bool, Path)
    """ Signal for showing planners and paths without threading errors. """

    def __init__(
        self,
        main_window: PyRoboSimMainWindow,
        world: World,
        show: bool = True,
        dpi: int = 100,
        animation_dt: float = 0.1,
        realtime_factor: float = 1.0,
    ) -> None:
        """
        Creates an instance of a PyRoboSim figure canvas.

        :param main_window: The main window object, needed for bookkeeping.
        :param world: World object to attach.
        :param show: If true (default), shows the GUI. Otherwise runs headless for testing.
        :param dpi: DPI for the figure.
        :param animation_dt: Time step for animations (seconds).
        :param realtime_factor: Real-time multiplication factor for animation (1.0 is real-time).
        """
        self.fig = Figure(dpi=dpi, tight_layout=True)
        self.axes = self.fig.add_subplot(111)
        super(WorldCanvas, self).__init__(self.fig)
        plt.ion()

        self.main_window = main_window
        self.world = world

        self.displayed_path = None
        self.displayed_path_start = None
        self.displayed_path_goal = None

        # Display/animation properties
        self.animation_dt = animation_dt
        self.realtime_factor = realtime_factor

        self.robot_bodies: list[Circle] = []
        self.robot_dirs: list[Line2D] = []
        self.robot_lengths: list[float] = []
        self.robot_texts: list[Text] = []
        self.robot_sensor_artists: list[Artist] = []
        self.obj_patches: list[PathPatch] = []
        self.obj_texts: list[Text] = []
        self.hallway_patches: list[PathPatch] = []
        self.room_patches: list[PathPatch] = []
        self.room_texts: list[Text] = []
        self.location_patches: list[PathPatch] = []
        self.location_texts: list[Text] = []
        self.path_planner_artists: dict[str, Artist] = {"graph": [], "path": []}
        self.show_collision_polygons = False

        # Connect signals
        self.draw_signal.connect(self.draw_and_sleep)
        self.navigate_signal.connect(self.navigate)
        self.show_hallways_signal.connect(self.show_hallways)
        self.show_locations_signal.connect(self.show_locations)
        self.show_objects_signal.connect(self.show_objects)
        self.show_robots_signal.connect(self.show_robots)
        self.show_planner_and_path_signal.connect(self.show_planner_and_path)

        # Thread pool for managing long-running tasks in separate threads.
        self.thread_pool = QThreadPool()

        # Start timer for animating robot navigation state.
        if show:
            sleep_time_msec = int(1000.0 * self.animation_dt / self.realtime_factor)
            self.nav_animator = QTimer()
            self.nav_animator.timeout.connect(self.nav_animation_callback)
            self.nav_animator.start(sleep_time_msec)

    def toggle_collision_polygons(self) -> None:
        """Shows/hides collision polygons."""
        self.show_collision_polygons = not self.show_collision_polygons
        self.world.logger.info(
            "Enabling collision polygons"
            if self.show_collision_polygons
            else "Disabling collision polygons"
        )
        self.show_hallways()
        self.show_rooms()

    def show_robots(self) -> None:
        """Draws robots, along with any associated sensors."""
        with self.draw_lock:
            len(self.world.robots)
            for body in self.robot_bodies:
                body.remove()
            for dir in self.robot_dirs:
                dir.remove()
            for text in self.robot_texts:
                text.remove()
            for artist in self.robot_sensor_artists:
                artist.remove()
            self.robot_bodies = []
            self.robot_dirs = []
            self.robot_lengths = []
            self.robot_sensor_artists = []

            for i, robot in enumerate(self.world.robots):
                p = robot.get_pose()
                self.robot_bodies.append(
                    Circle(
                        (p.x, p.y),
                        radius=robot.radius,
                        edgecolor=robot.color,
                        facecolor=[1.0, 1.0, 1.0],
                        fill=True,
                        linewidth=2,
                        zorder=self.robot_zorder,
                    )
                )
                self.axes.add_patch(self.robot_bodies[i])

                robot_length = self.robot_dir_line_factor * robot.radius
                (robot_dir,) = self.axes.plot(
                    p.x + np.array([0, robot_length * np.cos(p.get_yaw())]),
                    p.y + np.array([0, robot_length * np.sin(p.get_yaw())]),
                    linestyle="-",
                    color=robot.color,
                    linewidth=2,
                    zorder=self.robot_zorder,
                )
                self.robot_dirs.append(robot_dir)
                self.robot_lengths.append(robot_length)

                x = p.x
                y = p.y - 2.0 * robot.radius
                robot.viz_text = self.axes.text(
                    x,
                    y,
                    robot.name,
                    clip_on=True,
                    color=robot.color,
                    horizontalalignment="center",
                    verticalalignment="top",
                    fontsize=10,
                )

                for sensor in robot.sensors.values():
                    if sensor.is_active:
                        sensor_artists = sensor.setup_artists()
                        self.robot_sensor_artists.extend(sensor_artists)
                        for artist in sensor_artists:
                            self.axes.add_artist(artist)

            self.robot_texts = [robot.viz_text for robot in self.world.robots]

    def show_hallways(self) -> None:
        """Draws hallways in the world."""
        with self.draw_lock:
            for hallway in self.hallway_patches:
                hallway.remove()

            self.hallway_patches = [h.viz_patch for h in self.world.hallways]

            for h in self.world.hallways:
                self.axes.add_patch(h.viz_patch)
                if not h.is_open:
                    closed_patch = h.get_closed_patch()
                    self.axes.add_patch(closed_patch)
                    self.hallway_patches.append(closed_patch)
                elif self.show_collision_polygons:
                    coll_patch = h.get_collision_patch()
                    self.axes.add_patch(coll_patch)
                    self.hallway_patches.append(coll_patch)

    def show_rooms(self) -> None:
        """Draws rooms in the world."""
        with self.draw_lock:
            for room in self.room_patches:
                room.remove()
            for text in self.room_texts:
                text.remove()

            self.room_patches = [room.viz_patch for room in self.world.rooms]
            self.room_texts = []

            for room in self.world.rooms:
                self.axes.add_patch(room.viz_patch)
                viz_text = self.axes.text(
                    room.centroid[0],
                    room.centroid[1],
                    room.name,
                    color=room.viz_color,
                    fontsize=12,
                    ha="center",
                    va="top",
                    clip_on=True,
                )
                self.room_texts.append(viz_text)

                if self.show_collision_polygons:
                    coll_patch = room.get_collision_patch()
                    self.axes.add_patch(coll_patch)
                    self.room_patches.append(coll_patch)

    def show_locations(self) -> None:
        """Draws locations and object spawns in the world."""
        with self.draw_lock:
            for location in self.location_patches:
                location.remove()
            for text in self.location_texts:
                text.remove()

            self.location_patches = [loc.viz_patch for loc in self.world.locations]
            self.location_texts = []

            for loc in self.world.locations:
                self.axes.add_patch(loc.viz_patch)
                loc_text = self.axes.text(
                    loc.pose.x,
                    loc.pose.y,
                    loc.name,
                    color=loc.viz_color,
                    fontsize=10,
                    ha="center",
                    va="top",
                    clip_on=True,
                )
                for spawn in loc.children:
                    self.axes.add_patch(spawn.viz_patch)

                self.location_texts.append(loc_text)
                self.location_patches.extend(
                    [spawn.viz_patch for spawn in loc.children]
                )

    def show_objects(self) -> None:
        """Draws objects and their associated texts."""
        with self.draw_lock:
            for obj_patch in self.obj_patches:
                obj_patch.remove()
            for obj_text in self.obj_texts:
                obj_text.remove()

            robot = self.main_window.get_current_robot()
            if robot:
                known_objects = robot.get_known_objects()
            else:
                known_objects = self.world.objects

            for obj in known_objects:
                self.axes.add_patch(obj.viz_patch)
                xmin, ymin, xmax, ymax = obj.polygon.bounds
                x = obj.pose.x + 1.0 * (xmax - xmin)
                y = obj.pose.y + 1.0 * (ymax - ymin)
                obj.viz_text = self.axes.text(
                    x, y, obj.name, clip_on=True, color=obj.viz_color, fontsize=8
                )
            self.obj_patches = [o.viz_patch for o in known_objects]
            self.obj_texts = [o.viz_text for o in known_objects]

            # Adjust the text to try avoid collisions
            adjustText.adjust_text(
                self.obj_texts, iter_lim=100, objects=self.obj_patches, ax=self.axes
            )

    def show(self) -> None:
        """
        Displays all entities in the world (robots, rooms, objects, etc.).
        """
        # Entities in the world.
        self.show_rooms()
        self.show_hallways()
        self.show_locations()
        self.show_objects()

        # Robots, along with their paths and planner graphs
        self.show_robots()
        if len(self.world.robots) > 0:
            self.show_planner_and_path(robot=self.world.robots[0])

        self.axes.autoscale()
        self.axes.axis("equal")

    def draw_and_sleep(self) -> None:
        """Redraws the figure and waits a small amount of time."""
        with self.draw_lock:
            self.fig.canvas.flush_events()
            self.fig.canvas.draw()
            time.sleep(0.005)

    def show_planner_and_path(
        self,
        robot: Robot | None = None,
        show_graphs: bool = True,
        path: Path | None = None,
    ) -> None:
        """
        Plot the path planner and latest path, if specified.
        This planner could be global (property of the world)
        or local (property of the robot).

        :param robot: If set to a Robot instance, uses that robot for display.
        :param show_graphs: If True, shows the path planner's latest graph(s).
        :param path: An optional path to display.
        """
        if not robot:
            self.world.logger.warning("No robot found")
            return

        # Since removing artists while drawing can cause issues,
        # this function should also lock drawing.
        with self.draw_lock:
            color = robot.color if robot is not None else "m"
            if robot.path_planner:
                graphs = robot.path_planner.get_graphs() if show_graphs else []
                path = path or robot.path_planner.get_latest_path()
                path_planner_artists = plot_path_planner(
                    self.axes, graphs=graphs, path=path, path_color=color
                )

                for artist in self.path_planner_artists["graph"]:
                    artist.remove()
                self.path_planner_artists["graph"] = path_planner_artists.get(
                    "graph", []
                )

                for artist in self.path_planner_artists["path"]:
                    artist.remove()
                self.path_planner_artists["path"] = path_planner_artists.get("path", [])

        self.draw_and_sleep()

    def nav_animation_callback(self) -> None:
        """Timer callback function to animate navigating robots."""
        if not self.main_window.isVisible():
            return

        world = self.world
        if world.gui is None:
            return

        # Check if any robot is currently navigating.
        nav_status = [robot.is_moving() for robot in world.robots]
        if any(nav_status):
            # Show the state of the currently selected robot
            cur_robot = world.gui.get_current_robot()
            if cur_robot is not None and cur_robot.is_moving():
                self.show_world_state(cur_robot)
                world.gui.set_buttons_during_action(False)

        self.update_robots_plot()
        self.draw_and_sleep()

    def update_robots_plot(self) -> None:
        """Updates the robot visualization graphics objects."""
        with self.draw_lock:
            if len(self.world.robots) != len(self.robot_bodies):
                self.show_robots()
            for i, robot in enumerate(self.world.robots):
                p = robot.get_pose()
                self.robot_bodies[i].center = p.x, p.y
                self.robot_dirs[i].set_xdata(
                    p.x + np.array([0, self.robot_lengths[i] * np.cos(p.get_yaw())])
                )
                self.robot_dirs[i].set_ydata(
                    p.y + np.array([0, self.robot_lengths[i] * np.sin(p.get_yaw())])
                )
                if robot.viz_text is not None:
                    robot.viz_text.set_position((p.x, p.y - 2.0 * robot.radius))
                if robot.manipulated_object is not None:
                    self.update_object_plot(robot.manipulated_object)

                for sensor in robot.sensors.values():
                    if sensor.is_active:
                        sensor.update_artists()

    def show_world_state(self, robot: Robot | None = None) -> None:
        """
        Shows the world state in the figure title.

        :param robot: If set to a Robot instance, uses that robot for showing state.
        """
        if robot is not None:
            title_bits = []
            if robot.location is not None:
                if isinstance(robot.location, str):
                    robot_loc: str = robot.location
                else:
                    robot_loc = robot.location.name
                title_bits.append(f"Location: {robot_loc}")
            if robot.manipulated_object is not None:
                title_bits.append(f"Holding: {robot.manipulated_object.name}")

            battery_str = f"Battery: {robot.battery_level:.2f}%"
            title_str = f"[{robot.name}] " + battery_str + "\n" + ", ".join(title_bits)
            self.axes.set_title(title_str)

    def update_object_plot(self, obj: Object | None) -> None:
        """
        Updates an object visualization based on its pose.

        :param obj: Object instance to update.
        """
        if obj is None:
            return

        tf = (
            Affine2D()
            .translate(-obj.centroid[0], -obj.centroid[1])
            .rotate(obj.pose.get_yaw())
            .translate(obj.pose.x, obj.pose.y)
        )
        if obj.viz_patch is not None:
            obj.viz_patch.set_transform(tf + self.axes.transData)

        xmin, ymin, xmax, ymax = obj.polygon.bounds
        x = obj.pose.x + 1.0 * (xmax - xmin)
        y = obj.pose.y + 1.0 * (ymax - ymin)
        if obj.viz_text is not None:
            obj.viz_text.set_position((x, y))

    def navigate(self, robot: Robot, goal: str, path: Path | None = None) -> None:
        """
        Starts a thread to navigate a robot to a goal.

        :param robot: Robot instance or name to execute action.
        :param goal: Name of goal location (resolved by the world model).
        :param path: Path to goal location, defaults to None.
        """
        nav_thread = NavRunner(self.world, robot, goal, path, self.realtime_factor)
        self.thread_pool.start(nav_thread)

    def pick_object(
        self, robot: Robot, obj_name: str, grasp_pose: Pose | None = None
    ) -> None:
        """
        Picks an object with a specified robot.

        :param robot: Robot instance to execute action.
        :param obj_name: The name of the object.
        :param grasp_pose: A pose describing how to manipulate the object.
        """
        pick_thread = PickRunner(self.world, robot, obj_name, grasp_pose)
        self.thread_pool.start(pick_thread)

    def place_object(self, robot: Robot, pose: Pose | None = None) -> None:
        """
        Places an object at a specified robot's current location.

        :param robot: Robot instance to execute action.
        :param pose: Optional pose describing how to place the object.
        """
        place_thread = PlaceRunner(self.world, robot, pose)
        self.thread_pool.start(place_thread)

    def detect_objects(self, robot: Robot, query: str | None = None) -> None:
        """
        Detects objects at the robot's current location.

        :param robot: Robot instance to execute action.
        :param query: Query for object detection.
        """
        detect_thread = DetectRunner(self.world, robot, query)
        self.thread_pool.start(detect_thread)

    def open_location(self, robot: Robot) -> None:
        """
        Opens the robot's current location, if available.

        :param robot: Robot instance to execute action.
        """
        open_thread = OpenRunner(self.world, robot)
        self.thread_pool.start(open_thread)

    def close_location(self, robot: Robot) -> None:
        """
        Closes the robot's current location, if available.

        :param robot: Robot instance to execute action.
        """
        close_thread = CloseRunner(self.world, robot)
        self.thread_pool.start(close_thread)
