""" Utilities for displaying a pyrobosim world in a figure canvas. """

import adjustText
import numpy as np
import time
import threading
from matplotlib.figure import Figure
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg
from matplotlib.pyplot import Circle
from matplotlib.transforms import Affine2D
from PyQt5.QtCore import pyqtSignal


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

    nav_trigger = pyqtSignal(str, str)
    """ Signal to trigger navigation method in a thread-safe way. """

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
        self.path_planner_artists = []

        # Debug displays (TODO: Should be available from GUI).
        self.show_collision_polygons = False

        # Connect triggers for thread-safe execution.
        self.nav_trigger.connect(self.navigate_in_thread)

        self.draw_lock = False

        # Start thread for monitoring robot navigation state.
        self.nav_animator = threading.Thread(target=self.monitor_nav_animation)
        self.nav_animator.start()

    def show_robots(self):
        """Draws robots as circles with heading lines for visualization."""
        n_robots = len(self.world.robots)
        self.robot_bodies = n_robots*[None]
        self.robot_dirs = n_robots*[None]
        self.robot_lengths = n_robots*[None]

        for i, robot in enumerate(self.world.robots):
            p = robot.pose
            self.robot_bodies[i] = Circle(
                (p.x, p.y),
                radius=robot.radius,
                edgecolor=robot.color,
                fill=False,
                linewidth=2,
                zorder=self.robot_zorder)
            self.axes.add_patch(self.robot_bodies[i])

            robot_length = self.robot_dir_line_factor * robot.radius
            (self.robot_dirs[i],) = self.axes.plot(
                p.x + np.array([0, robot_length * np.cos(p.yaw)]),
                p.y + np.array([0, robot_length * np.sin(p.yaw)]),
                linestyle="-",
                color=robot.color,
                linewidth=2,
                zorder=self.robot_zorder,
            )
            self.robot_lengths[i] = robot_length

            x = p.x
            y = p.y - 2.0 * robot.radius
            robot.viz_text = self.axes.text(
                x, y, robot.name, clip_on=True, color=robot.color,
                horizontalalignment="center", verticalalignment="top",
                fontsize=10
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

        # Path planner and path
        if len(self.world.robots) > 0:
            robot_to_show = self.world.robots[0]
        else:
            robot_to_show = None
        self.show_planner_and_path(robot_to_show)

        self.axes.autoscale()
        self.axes.axis("equal")
        self.adjust_text(self.obj_texts)

    def draw_and_sleep(self):
        """Redraws the figure and waits a small amount of time."""
        if self.draw_lock:
            return

        self.draw_lock = True
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()
        time.sleep(0.001)
        self.draw_lock = False

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
                active_robot_indices = [i for i, status in enumerate(nav_status) if status]
                
                # Update the animation.
                self.update_robots_plot()
                for idx in active_robot_indices:    
                    self.update_object_plot(
                        self.world.robots[idx].manipulated_object)
                self.show_world_state(
                    self.world.robots[active_robot_indices[0]],
                    navigating=True)
                self.draw_and_sleep()

                # Check if GUI buttons should be disabled
                if self.world.has_gui and self.world.gui.layout_created:
                    cur_robot = self.world.gui.get_current_robot()
                    is_cur_robot_moving = cur_robot in self.world.robots and \
                                          cur_robot.executing_nav
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

    def show_path(self, path, robot=None):
        """
        Plots a standalone path.

        :param path: The path to display.
        :type path: :class:`pyrobosim.utils.motion.Path`
        """
        for e in self.path_planner_artists:
            self.axes.lines.remove(e)
        self.path_planner_artists = []
        x = [p.x for p in path.poses]
        y = [p.y for p in path.poses]
        color = robot.color if robot is not None else "m"
        (path,) = self.axes.plot(x, y, linestyle="-", color=color,
                                 linewidth=3, zorder=1)
        (start,) = self.axes.plot(x[0], y[0], "go", zorder=2)
        (goal,) = self.axes.plot(x[-1], y[-1], "rx", zorder=2)
        self.path_planner_artists.extend((path, start, goal))

    def show_planner_and_path(self, robot=None):
        """
        Plot the path planner and latest path, if specified.
        This planner could be global (property of the world)
        or local (property of the robot).

        :param robot: If set to a Robot instance, uses that robot for display.
        :type robot: :class:`pyrobosim.core.robot.Robot`, optional
        """
        # Since removing artists while drawing can cause issues,
        # this function should also lock drawing.
        while self.draw_lock:
            time.sleep(0.001)
        self.draw_lock = True
        for e in self.path_planner_artists:
            self.axes.lines.remove(e)

        color = robot.color if robot is not None else "m"

        if robot and robot.path_planner:
            self.path_planner_artists = robot.path_planner.plot(
                self.axes, path_color=color)
        elif self.world.path_planner:
            self.path_planner_artists = self.world.path_planner.plot(
                self.axes, path_color=color)
        self.draw_lock = False

    def update_robots_plot(self):
        """Updates the robot visualization graphics objects."""
        for i, robot in enumerate(self.world.robots):
            p = robot.pose
            self.robot_bodies[i].center = p.x, p.y
            self.robot_dirs[i].set_xdata(p.x + np.array([0, self.robot_lengths[i] * np.cos(p.yaw)]))
            self.robot_dirs[i].set_ydata(p.y + np.array([0, self.robot_lengths[i] * np.sin(p.yaw)]))
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
                title_bits.append(f"Location: {robot.location.name}")
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
            .rotate(obj.pose.yaw)
            .translate(obj.pose.x, obj.pose.y)
        )
        obj.viz_patch.set_transform(tf + self.axes.transData)

        xmin, ymin, xmax, ymax = obj.polygon.bounds
        x = obj.pose.x + 1.0 * (xmax - xmin)
        y = obj.pose.y + 1.0 * (ymax - ymin)
        obj.viz_text.set_position((x, y))

    def navigate_in_thread(self, robot, goal):
        """
        Starts a thread to navigate a robot to a goal.

        :param robot: Robot instance or name to execute action.
        :type robot: :class:`pyrobosim.core.robot.Robot` or str
        :param goal: Name of goal location (resolved by the world model).
        :type goal: str
        :return: True if navigation succeeds, else False
        :rtype: bool
        """
        if isinstance(robot, str):
            robot = self.world.get_robot_by_name(robot)
        t = threading.Thread(target=self.navigate, args=(robot, goal))
        t.start()

    def navigate(self, robot, goal):
        """
        Animates a path to a goal location using a robot's path executor.

        :param robot: Robot instance to execute action.
        :type robot: :class:`pyrobosim.core.robot.Robot`
        :param goal: Name of goal location (resolved by the world model).
        :type goal: str
        :return: True if navigation succeeds, else False
        :rtype: bool
        """

        # Find a path, or use an existing one, and start the navigation thread.
        if not robot.current_path or robot.current_path.num_poses < 1:
            path = self.world.find_path(goal, robot=robot)
            self.show_planner_and_path(robot)
        else:
            path = robot.current_path
            robot.current_goal = self.world.get_entity_by_name(goal)
            self.show_path(path, robot=robot)
        robot.follow_path(
            path,
            target_location=robot.current_goal,
            realtime_factor=self.realtime_factor,
        )

        # Sleep while the robot is executing the action.
        while robot.executing_nav:
            time.sleep(0.1)
        return True

    def pick_object(self, robot, obj_name):
        """
        Picks an object with a specified robot.

        :param robot: Robot instance to execute action.
        :type robot: :class:`pyrobosim.core.robot.Robot`
        :param obj_name: The name of the object.
        :type obj_name: str
        :return: True if picking succeeds, else False
        :rtype: bool
        """
        if robot is not None:
            success = robot.pick_object(obj_name)
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
            self.update_object_plot(obj)
            self.show_world_state(robot)
            self.draw_and_sleep()
            return success
        return False
