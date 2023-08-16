""" Utilities for displaying a pyrobosim world in a figure canvas. """

import adjustText
import numpy as np
import time
import threading
import warnings
from matplotlib.figure import Figure
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg
from matplotlib.pyplot import Circle
from matplotlib.transforms import Affine2D
from PyQt5.QtCore import pyqtSignal, QThread

from pyrobosim.utils.motion import Path


class NavAnimator(QThread):
    """
    Helper class that wraps navigation animation in a QThread.
    """

    def __init__(self, canvas):
        """
        Creates a navigation monitor thread.

        :param canvas: A world canvas object linked to this thread.
        :type canvas: :class:`pyrobosim.gui.world_canvas.WorldCanvas`
        """
        super(NavAnimator, self).__init__()
        self.canvas = canvas

    def run(self):
        """Runs the navigation monitor thread."""
        self.canvas.monitor_nav_animation()


class WorldCanvas(FigureCanvasQTAgg):
    """
    Canvas for rendering a pyrobosim world as a matplotlib figure in an
    application.
    """

    # Animation constants
    animation_dt = 0.1
    """ Time step for animations (seconds). """
    realtime_factor = 1.0
    """ Real-time multiplier for animations. """

    # Visualization constants
    object_zorder = 3
    """ zorder for object visualization. """
    robot_zorder = 3
    """ zorder for robot visualization. """

    nav_trigger = pyqtSignal(str, str, Path)
    """ Signal to trigger navigation method in a thread-safe manner. """

    draw_lock = threading.Lock()
    """ Lock for drawing on the canvas in a thread-safe manner. """

    def __init__(self, world, dpi=100):
        """
        Creates an instance of a pyrobosim figure canvas.

        :param world: World object to attach.
        :type world: :class:`pyrobosim.core.world.World`
        :param dpi: DPI for the figure.
        :type dpi: int
        """
        self.fig = Figure(dpi=dpi, tight_layout=True)
        self.axes = self.fig.add_subplot(111)
        super(WorldCanvas, self).__init__(self.fig)

        self.world = world

        self.displayed_path = None
        self.displayed_path_start = None
        self.displayed_path_goal = None

        # Multiplier of robot radius for plotting robot orientation lines.
        self.robot_dir_line_factor = 3.0

        self.robot_bodies = []
        self.robot_dirs = []
        self.robot_lengths = []
        self.path_planner_artists = {"graph": [], "path": []}

        # Debug displays (TODO: Should be available from GUI).
        self.show_collision_polygons = False

        # Connect triggers for thread-safe execution.
        self.nav_trigger.connect(self.navigate_in_thread)

        # Start thread for animating robot navigation state.
        self.nav_animator = NavAnimator(self)
        self.nav_animator.start()

    def show_robots(self):
        """Draws robots as circles with heading lines for visualization."""
        n_robots = len(self.world.robots)
        for b in self.robot_bodies:
            b.remove()
        for d in self.robot_dirs:
            b.remove()
        for l in self.robot_lengths:
            l.remove()
        self.robot_bodies = n_robots * [None]
        self.robot_dirs = n_robots * [None]
        self.robot_lengths = n_robots * [None]

        for i, robot in enumerate(self.world.robots):
            p = robot.pose
            self.robot_bodies[i] = Circle(
                (p.x, p.y),
                radius=robot.radius,
                edgecolor=robot.color,
                fill=False,
                linewidth=2,
                zorder=self.robot_zorder,
            )
            self.axes.add_patch(self.robot_bodies[i])

            robot_length = self.robot_dir_line_factor * robot.radius
            (self.robot_dirs[i],) = self.axes.plot(
                p.x + np.array([0, robot_length * np.cos(p.get_yaw())]),
                p.y + np.array([0, robot_length * np.sin(p.get_yaw())]),
                linestyle="-",
                color=robot.color,
                linewidth=2,
                zorder=self.robot_zorder,
            )
            self.robot_lengths[i] = robot_length

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
        self.robot_texts = [r.viz_text for r in (self.world.robots)]

    def show(self):
        """
        Displays all entities in the world (robots, rooms, objects, etc.).
        """
        # Robots
        self.show_robots()

        # Rooms and hallways
        for r in self.world.rooms:
            self.axes.add_patch(r.viz_patch)
            t = self.axes.text(
                r.centroid[0],
                r.centroid[1],
                r.name,
                color=r.viz_color,
                fontsize=12,
                ha="center",
                va="top",
                clip_on=True,
            )
            if self.show_collision_polygons:
                self.axes.add_patch(r.get_collision_patch())
        for h in self.world.hallways:
            self.axes.add_patch(h.viz_patch)
            if self.show_collision_polygons:
                self.axes.add_patch(h.get_collision_patch())

        # Locations
        for loc in self.world.locations:
            self.axes.add_patch(loc.viz_patch)
            t = self.axes.text(
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

        # Objects
        for obj in self.world.objects:
            self.axes.add_patch(obj.viz_patch)
            xmin, ymin, xmax, ymax = obj.polygon.bounds
            x = obj.pose.x + 1.0 * (xmax - xmin)
            y = obj.pose.y + 1.0 * (ymax - ymin)
            obj.viz_text = self.axes.text(
                x, y, obj.name, clip_on=True, color=obj.viz_color, fontsize=8
            )
        self.obj_patches = [o.viz_patch for o in (self.world.objects)]
        self.obj_texts = [o.viz_text for o in (self.world.objects)]

        # Show paths and planner graphs
        if len(self.world.robots) > 0:
            self.show_planner_and_path(robot=self.world.robots[0])

        self.axes.autoscale()
        self.axes.axis("equal")
        self.adjust_text(self.obj_texts)

    def draw_and_sleep(self):
        """Redraws the figure and waits a small amount of time."""
        if self.draw_lock.locked():
            return
        self.draw_lock.acquire()
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()
        time.sleep(0.001)
        self.draw_lock.release()

    def get_animated_artists(self):
        """Returns a list of artists to animate when blitting."""
        animated_artists = self.robot_bodies + self.robot_dirs + [self.axes.title]
        for robot in self.world.robots:
            held_object = robot.manipulated_object
            if held_object is not None:
                animated_artists.extend([held_object.viz_patch, held_object.viz_text])
        return animated_artists

    def monitor_nav_animation(self):
        """
        Monitors the navigation animation (to be started in a separate thread).
        """
        sleep_time = self.animation_dt / self.realtime_factor
        while True:
            # Check if any robot is currently navigating.
            nav_status = [robot.executing_nav for robot in self.world.robots]
            if any(nav_status):
                active_robot_indices = [
                    i for i, status in enumerate(nav_status) if status
                ]

                # Update the animation.
                self.update_robots_plot()
                self.show_world_state(
                    self.world.robots[active_robot_indices[0]], navigating=True
                )
                self.draw_and_sleep()

                # Check if GUI buttons should be disabled
                if self.world.has_gui and self.world.gui.layout_created:
                    cur_robot = self.world.gui.get_current_robot()
                    is_cur_robot_moving = (
                        cur_robot in self.world.robots and cur_robot.executing_nav
                    )
                    self.world.gui.set_buttons_during_action(not is_cur_robot_moving)

            else:
                # If the GUI button states did not toggle correctly, force them
                # to be active once no robots are moving.
                if self.world.has_gui and self.world.gui.layout_created:
                    self.world.gui.set_buttons_during_action(True)

            time.sleep(sleep_time)

    def adjust_text(self, objs):
        """
        Adjust text in a figure.

        :param objs: List of objects to consider for text adjustment
        :type objs: list
        """
        adjustText.adjust_text(objs, lim=100, add_objects=self.obj_patches)

    def show_planner_and_path(self, robot=None, path=None):
        """
        Plot the path planner and latest path, if specified.
        This planner could be global (property of the world)
        or local (property of the robot).

        :param robot: If set to a Robot instance, uses that robot for display.
        :type robot: :class:`pyrobosim.core.robot.Robot`, optional
        :param path: Path to goal location, defaults to None.
        :type path: :class:`pyrobosim.utils.motion.Path`, optional
        """
        # Since removing artists while drawing can cause issues,
        # this function should also lock drawing.
        self.draw_lock.acquire()

        color = robot.color if robot is not None else "m"
        if robot and robot.path_planner:
            path_planner_artists = robot.path_planner.plot(
                self.axes, path=path, path_color=color
            )

            for artist in self.path_planner_artists["graph"]:
                artist.remove()
            self.path_planner_artists["graph"] = path_planner_artists.get("graph", [])

            for artist in self.path_planner_artists["path"]:
                artist.remove()
            self.path_planner_artists["path"] = path_planner_artists.get("path", [])

        else:
            if not robot:
                warnings.warn("No robot found")
            elif not robot.path_planner:
                warnings.warn("Robot does not have a planner")

        self.draw_lock.release()

    def update_robots_plot(self):
        """Updates the robot visualization graphics objects."""
        if len(self.world.robots) != len(self.robot_bodies):
            self.show_robots()
        for i, robot in enumerate(self.world.robots):
            p = robot.pose
            self.robot_bodies[i].center = p.x, p.y
            self.robot_dirs[i].set_xdata(
                p.x + np.array([0, self.robot_lengths[i] * np.cos(p.get_yaw())])
            )
            self.robot_dirs[i].set_ydata(
                p.y + np.array([0, self.robot_lengths[i] * np.sin(p.get_yaw())])
            )
            robot.viz_text.set_position((p.x, p.y - 2.0 * robot.radius))
            self.update_object_plot(robot.manipulated_object)

    def show_world_state(self, robot=None, navigating=False):
        """
        Shows the world state in the figure title.

        :param robot: If set to a Robot instance, uses that robot for showing state.
        :type robot: :class:`pyrobosim.core.robot.Robot`, optional
        :param navigating: Flag that indicates that the robot is moving so we
            should continuously update the title containing the robot location.
        :type navigating: bool, optional
        """
        if robot is not None:
            title_bits = []
            if navigating:
                robot_loc = self.world.get_location_from_pose(robot.pose)
                if robot_loc is not None:
                    title_bits.append(f"Location: {robot_loc.name}")
            elif robot.location is not None:
                if isinstance(robot.location, str):
                    robot_loc = robot.location
                else:
                    robot_loc = robot.location.name
                title_bits.append(f"Location: {robot_loc}")
            if robot.manipulated_object is not None:
                title_bits.append(f"Holding: {robot.manipulated_object.name}")
            title_str = f"[{robot.name}] " + ", ".join(title_bits)
            self.axes.set_title(title_str)

    def update_object_plot(self, obj):
        """
        Updates an object visualization based on its pose.

        :param obj: pyrobosim object to update.
        :type obj: class:`pyrobosim.objects.Object`
        """
        if obj is None:
            return

        tf = (
            Affine2D()
            .translate(-obj.centroid[0], -obj.centroid[1])
            .rotate(obj.pose.get_yaw())
            .translate(obj.pose.x, obj.pose.y)
        )
        obj.viz_patch.set_transform(tf + self.axes.transData)

        xmin, ymin, xmax, ymax = obj.polygon.bounds
        x = obj.pose.x + 1.0 * (xmax - xmin)
        y = obj.pose.y + 1.0 * (ymax - ymin)
        obj.viz_text.set_position((x, y))

    def navigate_in_thread(self, robot, goal, path=None):
        """
        Starts a thread to navigate a robot to a goal.

        :param robot: Robot instance or name to execute action.
        :type robot: :class:`pyrobosim.core.robot.Robot` or str
        :param goal: Name of goal location (resolved by the world model).
        :type goal: str
        :param path: Path to goal location, defaults to None.
        :type path: :class:`pyrobosim.utils.motion.Path`, optional
        :return: True if navigation succeeds, else False
        :rtype: bool
        """
        if isinstance(robot, str):
            robot = self.world.get_robot_by_name(robot)
        nav_thread = threading.Thread(target=self.navigate, args=(robot, goal, path))
        nav_thread.start()

    def navigate(self, robot, goal, path=None):
        """
        Animates a path to a goal location using a robot's path executor.

        :param robot: Robot instance to execute action.
        :type robot: :class:`pyrobosim.core.robot.Robot`
        :param goal: Name of goal location (resolved by the world model).
        :type goal: str
        :param path: Path to goal location, defaults to None.
        :type path: :class:`pyrobosim.utils.motion.Path`, optional
        :return: True if navigation succeeds, else False
        :rtype: bool
        """

        # Find a path, or use an existing one, and start the navigation thread.
        if robot and robot.path_planner:
            goal_node = self.world.graph_node_from_entity(goal, robot=robot)
            if not path or path.num_poses < 2:
                path = robot.plan_path(robot.pose, goal_node.pose)
            self.show_planner_and_path(robot=robot, path=path)
            robot.follow_path(
                path,
                target_location=goal_node.parent,
                realtime_factor=self.realtime_factor,
                blocking=False,
            )

            # Sleep while the robot is executing the action.
            while robot.executing_nav:
                time.sleep(0.1)

            self.show_world_state(robot=robot)
            self.draw_and_sleep()
            return True

        return False

    def pick_object(self, robot, obj_name, grasp_pose=None):
        """
        Picks an object with a specified robot.

        :param robot: Robot instance to execute action.
        :type robot: :class:`pyrobosim.core.robot.Robot`
        :param obj_name: The name of the object.
        :type obj_name: str
        :param grasp_pose: A pose describing how to manipulate the object.
        :type grasp_pose: :class:`pyrobosim.utils.pose.Pose`, optional
        :return: True if picking succeeds, else False
        :rtype: bool
        """
        if robot is not None:
            success = robot.pick_object(obj_name, grasp_pose)
            if success:
                self.update_object_plot(robot.manipulated_object)
                self.show_world_state(robot)
                self.draw_and_sleep()
            return success
        return False

    def place_object(self, robot, pose=None):
        """
        Places an object at a specified robot's current location.

        :param robot: Robot instance to execute action.
        :type robot: :class:`pyrobosim.core.robot.Robot`
        :param pose: Optional placement pose, defaults to None.
        :type pose: :class:`pyrobosim.utils.pose.Pose`, optional
        :return: True if placing succeeds, else False
        :rtype: bool
        """
        if robot is not None:
            obj = robot.manipulated_object
            if obj is None:
                return
            obj.viz_patch.remove()
            success = robot.place_object(pose=pose)
            self.axes.add_patch(obj.viz_patch)
            self.show_world_state(robot)
            self.draw_and_sleep()
            return success
        return False
