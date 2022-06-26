""" Utilities for displaying a pyrobosim world in a figure canvas. """

import adjustText
import numpy as np
import time
from matplotlib.figure import Figure
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg
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

    nav_trigger = pyqtSignal(str)
    """ Signal to trigger navigation method in a thread-safe way. """

    def __init__(self, world, dpi=100):
        """
        Creates an instance of a pyrobosim figure canvas.

        :param dpi: DPI for the figure.
        :type dpi: int
        """
        self.fig = Figure(dpi=dpi, tight_layout=True)
        self.axes = self.fig.add_subplot(111)
        super(WorldCanvas, self).__init__(self.fig)

        self.world = world

        self.robot_normalized_length = 0.1
        self.robot_length = None

        self.displayed_path = None
        self.displayed_path_start = None
        self.displayed_path_goal = None

        self.robot_body = None
        self.robot_dir = None
        self.animated_artists = [self.robot_body, self.robot_dir]
        self.path_planner_artists = []

        # Debug displays (TODO: Should be available from GUI)
        self.show_collision_polygons = False

        # Connect triggers for thread-safe execution
        self.nav_trigger.connect(self.navigate)

    def show_robot(self):
        """Draws a robot as a circle with a heading line for visualization."""
        if self.world.has_robot:
            self.robot_length = self.robot_normalized_length * max(
                (self.world.x_bounds[1] - self.world.x_bounds[0]),
                (self.world.y_bounds[1] - self.world.y_bounds[0]),
            )
            p = self.world.robot.pose
            (self.robot_body,) = self.axes.plot(
                p.x,
                p.y,
                "mo",
                markersize=10,
                markeredgewidth=2,
                markerfacecolor="None",
                zorder=self.robot_zorder,
            )
            (self.robot_dir,) = self.axes.plot(
                p.x + np.array([0, self.robot_length * np.cos(p.yaw)]),
                p.y + np.array([0, self.robot_length * np.sin(p.yaw)]),
                "m-",
                linewidth=2,
                zorder=self.robot_zorder,
            )

    def show(self):
        """
        Displays all entities in the world (rooms, locations, objects, etc.).
        """
        # Robot
        self.show_robot()
        self.show_world_state()

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
        self.show_planner_and_path()

        # Update the robot length
        self.robot_length = self.robot_normalized_length * max(
            (self.world.x_bounds[1] - self.world.x_bounds[0]),
            (self.world.y_bounds[1] - self.world.y_bounds[0]),
        )

        self.axes.autoscale()
        self.axes.axis("equal")
        self.adjust_text(self.obj_texts)

    def draw_and_sleep(self):
        """Redraws the figure and waits a small amount of time."""
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()
        time.sleep(0.001)

    def get_animated_artists(self):
        """Returns a list of artists to animate when blitting."""
        animated_artists = [self.robot_body, self.robot_dir, self.axes.title]
        held_object = self.world.robot.manipulated_object
        if held_object is not None:
            animated_artists.extend([held_object.viz_patch, held_object.viz_text])
        return animated_artists

    def adjust_text(self, objs):
        """
        Adjust text in a figure.

        :param objs: List of objects to consider for text adjustment
        :type objs: list
        """
        adjustText.adjust_text(objs, lim=100, add_objects=self.obj_patches)

    def show_path(self, path):
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
        (path,) = self.axes.plot(x, y, "m-", linewidth=3, zorder=1)
        (start,) = self.axes.plot(x[0], y[0], "go", zorder=2)
        (goal,) = self.axes.plot(x[-1], y[-1], "rx", zorder=2)
        self.path_planner_artists.extend((path, start, goal))

    def show_planner_and_path(self):
        """
        Plot the path planner and latest path, if specified.
        This planner could be global (property of the world)
        or local (property of the robot).
        """
        for e in self.path_planner_artists:
            self.axes.lines.remove(e)

        if self.world.robot.path_planner:
            self.path_planner_artists = self.world.robot.path_planner.plot(self.axes)
        elif self.world.path_planner:
            self.path_planner_artists = self.world.path_planner.plot(self.axes)

    def update_robot_plot(self):
        """Updates the robot visualization graphics objects."""
        p = self.world.robot.pose
        self.robot_body.set_xdata(p.x)
        self.robot_body.set_ydata(p.y)
        self.robot_dir.set_xdata(p.x + np.array([0, self.robot_length * np.cos(p.yaw)]))
        self.robot_dir.set_ydata(p.y + np.array([0, self.robot_length * np.sin(p.yaw)]))
        self.update_object_plot(self.world.robot.manipulated_object)

    def show_world_state(self, navigating=False):
        """
        Shows the world state in the figure title.

        :param navigating: Flag that indicates that the robot is moving so we
            should continuously update the title containing the robot location.
        :type navigating: bool, optional
        """
        r = self.world.robot
        if r is not None:
            title_bits = []
            if navigating:
                robot_loc = self.world.get_location_from_pose(self.world.robot.pose)
                if robot_loc is not None:
                    title_bits.append(f"Location: {robot_loc.name}")
            elif r.location is not None:
                title_bits.append(f"Location: {r.location.name}")
            if r.manipulated_object is not None:
                title_bits.append(f"Holding: {r.manipulated_object.name}")
            title_str = ", ".join(title_bits)
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

    def navigate(self, goal):
        """
        Animates a path to a goal location using the robot's path executor.

        :param goal: Name of goal location (resolved by the world model).
        :type goal: str
        :return: True if navigation succeeds, else False
        :rtype: bool
        """
        # Find a path, or use an existing one, and start the navigation thread.
        if not self.world.current_path or self.world.current_path.num_poses < 1:
            path = self.world.find_path(goal)
            self.show_planner_and_path()
        else:
            path = self.world.current_path
            self.world.current_goal = self.world.get_entity_by_name(goal)
            self.show_path(path)
        self.world.robot.follow_path(
            path,
            target_location=self.world.current_goal,
            realtime_factor=self.realtime_factor,
        )

        # Animate while navigation is active
        do_blit = True  # Keeping this around to disable if needed
        sleep_time = self.animation_dt / self.realtime_factor
        if do_blit:
            animated_artists = self.get_animated_artists()
            for a in animated_artists:
                a.set_animated(True)
            self.draw_and_sleep()
            bg = self.fig.canvas.copy_from_bbox(self.fig.bbox)
            while self.world.robot.executing_nav:
                # Needs to happen before blitting to avoid race condition
                time.sleep(sleep_time)
                self.fig.canvas.restore_region(bg)
                self.update_robot_plot()
                self.update_object_plot(self.world.robot.manipulated_object)
                self.show_world_state(navigating=True)
                for a in animated_artists:
                    self.axes.draw_artist(a)
                self.fig.canvas.blit(self.fig.bbox)
                self.fig.canvas.flush_events()
            for a in animated_artists:
                a.set_animated(False)
        else:
            while self.world.robot.executing_nav:
                self.update_robot_plot()
                self.update_object_plot(self.world.robot.manipulated_object)
                self.show_world_state(navigating=True)
                self.draw_and_sleep()
                time.sleep(sleep_time)

        self.show_world_state()
        self.draw_and_sleep()
        return True

    def pick_object(self, obj_name):
        """
        Picks an object.

        :param obj_name: The name of the object.
        :type obj_name: str
        :return: True if picking succeeds, else False
        :rtype: bool
        """
        robot = self.world.robot
        if robot is not None:
            success = robot.pick_object(obj_name)
            if success:
                self.update_object_plot(robot.manipulated_object)
                self.show_world_state()
                self.draw_and_sleep()
            return success
        return False

    def place_object(self, pose=None):
        """
        Places an object at the robot's current location.

        :param pose: Optional placement pose, defaults to None.
        :type pose: :class:`pyrobosim.utils.pose.Pose`
        :return: True if placing succeeds, else False
        :rtype: bool
        """
        robot = self.world.robot
        if robot is not None:
            obj = robot.manipulated_object
            if obj is None:
                return
            obj.viz_patch.remove()
            success = robot.place_object(pose=pose)
            self.axes.add_patch(obj.viz_patch)
            self.update_object_plot(obj)
            self.show_world_state()
            self.draw_and_sleep()
            return success
        return False
