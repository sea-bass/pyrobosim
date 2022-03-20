import adjustText
import numpy as np
import time
from matplotlib.figure import Figure
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg
from matplotlib.transforms import Affine2D

from ..utils.pose import Pose
from ..utils.trajectory import get_constant_speed_trajectory, interpolate_trajectory


class WorldGUI(FigureCanvasQTAgg):
    object_zorder = 3

    def __init__(self, world, width=5, height=4, dpi=100):
        self.fig = Figure(figsize=(width, height), dpi=dpi)
        self.axes = self.fig.add_subplot(111)
        self.axes.set_title("World Model")

        self.robot_normalized_length = 0.1
        self.robot_length = None

        self.displayed_path = None
        self.displayed_path_start = None
        self.displayed_path_goal = None

        self.world = world
        self.createRobot()

        super(WorldGUI, self).__init__(self.fig)

    def createRobot(self):
        """ Creates the robot for visualization """
        self.robot_length = self.robot_normalized_length * max(
            (self.world.x_bounds[1] - self.world.x_bounds[0]),
            (self.world.y_bounds[1] - self.world.y_bounds[0]))
        p = self.world.robot.pose
        self.robot_body, = self.axes.plot(
            p.x, p.y,
            "mo", markersize=10, markeredgewidth=2,
            markerfacecolor="None")
        self.robot_dir, = self.axes.plot(
            p.x + np.array([0, self.robot_length*np.cos(p.yaw)]),
            p.y + np.array([0, self.robot_length*np.sin(p.yaw)]),
            "m-", linewidth=2)

    def show(self):
        # Rooms and hallways
        for r in self.world.rooms:
            self.axes.add_patch(r.viz_patch)
            t = self.axes.text(r.centroid[0], r.centroid[1], r.name,
                               color=r.viz_color, fontsize=12,
                               ha="center", va="top")
        for h in self.world.hallways:
            self.axes.add_patch(h.viz_patch)

        # Locations
        for loc in self.world.locations:
            self.axes.add_patch(loc.viz_patch)
            t = self.axes.text(loc.pose.x, loc.pose.y, loc.name,
                               color=loc.viz_color, fontsize=10,
                               ha="center", va="top")
            for spawn in loc.children:
                self.axes.add_patch(spawn.viz_patch)

        # Objects
        self.obj_texts = []
        for obj in self.world.objects:
            self.axes.add_patch(obj.viz_patch)
            xmin, ymin, xmax, ymax = obj.polygon.bounds
            x = obj.pose.x + 1.5*(xmax - obj.pose.x)
            y = obj.pose.y + 1.5*(ymin - obj.pose.y)
            obj.viz_text = self.axes.text(x, y, obj.name,
                                          color=obj.viz_color, fontsize=8)
            self.obj_texts.append(obj.viz_text)

        # Search graph
        if self.world.search_graph is not None:
            x = [n.pose.x for n in self.world.search_graph.nodes]
            y = [n.pose.y for n in self.world.search_graph.nodes]
            graph_nodes = self.axes.scatter(x, y, 15, "k")

            graph_edges = []
            for e in self.world.search_graph.edges:
                x = (e.n0.pose.x, e.n1.pose.x)
                y = (e.n0.pose.y, e.n1.pose.y)
                graph_edges.append(self.axes.plot(x, y, "k:", linewidth=1))

            # Plot the path if specified
            self.show_path()

        # Update the robot length
        self.robot_length = self.robot_normalized_length * max(
            (self.world.x_bounds[1] - self.world.x_bounds[0]),
            (self.world.y_bounds[1] - self.world.y_bounds[0]))

        self.axes.autoscale()
        self.axes.axis("equal")
        self.adjust_text(self.obj_texts)


    def adjust_text(self, objs):
        """ Adjust text in a figure """
        obj_patches = [o.viz_patch for o in (self.world.objects)]
        adjustText.adjust_text(objs, lim=100,
                               add_objects=obj_patches + self.obj_texts)


    def show_path(self, path=None):
        """ Displays a path """
        if path is not None:
            x = [p.x for p in self.world.current_path]
            y = [p.y for p in self.world.current_path]

            if self.displayed_path is None:
                self.displayed_path, = self.axes.plot(x, y, "m-", linewidth=3,
                                                      zorder=1)
                self.displayed_path_start = self.axes.scatter(
                    x[0], y[0], 60, "g", "o", zorder=2)
                self.displayed_path_goal = self.axes.scatter(
                    x[-1], y[-1], 60, "r", "x", zorder=2)
            else:
                self.displayed_path.set_xdata(x)
                self.displayed_path.set_ydata(y)
                self.displayed_path_start.set_offsets((x[0], y[0]))
                self.displayed_path_goal.set_offsets((x[-1], y[-1]))

    def update_robot_plot(self):
        """ Updates the robot visualization graphics objects """
        p = self.world.robot.pose
        self.robot_body.set_xdata(p.x)
        self.robot_body.set_ydata(p.y)
        self.robot_dir.set_xdata(
            p.x + np.array([0, self.robot_length*np.cos(p.yaw)]))
        self.robot_dir.set_ydata(
            p.y + np.array([0, self.robot_length*np.sin(p.yaw)]))

    def update_object_plot(self, obj):
        """ Updates an object visualization based on its pose """
        tf = Affine2D().translate(-obj.centroid[0], -obj.centroid[1]).rotate(
            obj.pose.yaw).translate(obj.pose.x, obj.pose.y)
        obj.viz_patch.set_transform(tf + self.axes.transData)
        obj.viz_text.set_position((obj.pose.x, obj.pose.y))
        self.adjust_text([obj.viz_text])

    def animate_path(self, path=None, linear_velocity=0.2, max_angular_velocity=None,
                     dt=0.1, realtime_factor=1.0):
        """ 
        Animates a path (found using `find_path()`) given a 
        velocity, time step, and real-time scale factor
        """
        if path is None:
            path = self.world.current_path

        # Convert the path to an interpolated trajectory
        traj = get_constant_speed_trajectory(path, linear_velocity=linear_velocity,
                                             max_angular_velocity=max_angular_velocity)
        (traj_t, traj_x, traj_y, traj_yaw) = interpolate_trajectory(traj, dt)

        # Loop through and animate
        self.show_path(path)
        dt = dt/realtime_factor
        has_manip_object = self.world.robot.manipulated_object is not None
        for i in range(len(traj_t)):
            pose = Pose(x=traj_x[i], y=traj_y[i], yaw=traj_yaw[i])
            self.world.robot.set_pose(pose)
            self.update_robot_plot()
            if has_manip_object:
                self.world.robot.manipulated_object.pose = pose
                self.update_object_plot(self.world.robot.manipulated_object)

            self.fig.canvas.draw()
            self.fig.canvas.flush_events()
            time.sleep(dt)

    def pick_object(self, obj_name):
        """ Picks an object """
        if self.world.pick_object(obj_name):
            self.update_object_plot(self.world.robot.manipulated_object)
            self.fig.canvas.draw()
            self.fig.canvas.flush_events()
            time.sleep(0.01)

    def place_object(self, obj_name, loc_name):
        """ Places an object """
        obj = self.world.get_object_by_name(self.world.robot.manipulated_object.name)
        obj.viz_patch.remove()
        if self.world.place_object(loc_name):
            self.axes.add_patch(obj.viz_patch)
            self.update_object_plot(obj)
            self.fig.canvas.draw()
            self.fig.canvas.flush_events()
            time.sleep(0.01)
