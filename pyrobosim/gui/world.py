import adjustText
import numpy as np
import time
from matplotlib.figure import Figure
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg
from matplotlib.transforms import Affine2D


class WorldGUI(FigureCanvasQTAgg):
    object_zorder = 3
    robot_zorder = 3

    def __init__(self, world, width=5, height=4, dpi=100):
        self.fig = Figure(figsize=(width, height), dpi=dpi)
        self.axes = self.fig.add_subplot(111)

        # Connect the GUI to the world
        self.world = world
        self.world.gui = self
        self.world.has_gui = True

        self.robot_normalized_length = 0.1
        self.robot_length = None

        self.displayed_path = None
        self.displayed_path_start = None
        self.displayed_path_goal = None

        self.robot_body = None
        self.robot_dir = None
        self.animated_artists = [self.robot_body, self.robot_dir]

        # Debug displays (TODO: Should be available from GUI)
        self.show_collision_polygons = False

        super(WorldGUI, self).__init__(self.fig)

    def show_robot(self):
        """ Creates the robot for visualization """
        if self.world.has_robot:
            self.robot_length = self.robot_normalized_length * max(
                (self.world.x_bounds[1] - self.world.x_bounds[0]),
                (self.world.y_bounds[1] - self.world.y_bounds[0]))
            p = self.world.robot.pose
            self.robot_body, = self.axes.plot(
                p.x, p.y,
                "mo", markersize=10, markeredgewidth=2,
                markerfacecolor="None", zorder=self.robot_zorder)
            self.robot_dir, = self.axes.plot(
                p.x + np.array([0, self.robot_length*np.cos(p.yaw)]),
                p.y + np.array([0, self.robot_length*np.sin(p.yaw)]),
                "m-", linewidth=2, zorder=self.robot_zorder)

    def show(self):
        # Robot
        self.show_robot()
        self.show_world_state()

        # Rooms and hallways
        for r in self.world.rooms:
            self.axes.add_patch(r.viz_patch)
            t = self.axes.text(r.centroid[0], r.centroid[1], r.name,
                               color=r.viz_color, fontsize=12,
                               ha="center", va="top")
            if self.show_collision_polygons:
                self.axes.add_patch(r.get_collision_patch())
        for h in self.world.hallways:
            self.axes.add_patch(h.viz_patch)
            if self.show_collision_polygons:
                self.axes.add_patch(h.get_collision_patch())

        # Locations
        for loc in self.world.locations:
            self.axes.add_patch(loc.viz_patch)
            t = self.axes.text(loc.pose.x, loc.pose.y, loc.name,
                               color=loc.viz_color, fontsize=10,
                               ha="center", va="top")
            for spawn in loc.children:
                self.axes.add_patch(spawn.viz_patch)

        # Objects
        for obj in self.world.objects:
            self.axes.add_patch(obj.viz_patch)
            xmin, ymin, xmax, ymax = obj.polygon.bounds
            x = obj.pose.x + 1.0*(xmax - xmin)
            y = obj.pose.y + 1.0*(ymax - ymin)
            obj.viz_text = self.axes.text(x, y, obj.name,
                                          color=obj.viz_color, fontsize=8)
        self.obj_patches = [o.viz_patch for o in (self.world.objects)]
        self.obj_texts = [o.viz_text for o in (self.world.objects)]

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

    def draw_and_sleep(self):
        """ Redraws the figure and waits a small amount of time """
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()
        time.sleep(0.001)

    def adjust_text(self, objs):
        """ Adjust text in a figure """
        adjustText.adjust_text(objs, lim=100,
                               add_objects=self.obj_patches)

    def show_path(self, path=None):
        """ Displays a path """
        if path is None:
            path = self.world.current_path

        if path is not None:
            x = [p.pose.x for p in self.world.current_path]
            y = [p.pose.y for p in self.world.current_path]

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

    def show_world_state(self, navigating=False):
        """ Shows the world state in the figure title """
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
        """ Updates an object visualization based on its pose """
        tf = Affine2D().translate(-obj.centroid[0], -obj.centroid[1]).rotate(
            obj.pose.yaw).translate(obj.pose.x, obj.pose.y)
        obj.viz_patch.set_transform(tf + self.axes.transData)
        
        xmin, ymin, xmax, ymax = obj.polygon.bounds
        x = obj.pose.x + 1.0*(xmax - xmin)
        y = obj.pose.y + 1.0*(ymax - ymin)
        obj.viz_text.set_position((x, y))

    def navigate(self, goal):
        """ 
        Animates a path (found using `find_path()`) given a 
        velocity, time step, and real-time scale factor
        """
        # Find a path and kick off the navigation thread
        path = self.world.find_path(goal)
        self.show_path(path)

        dt = 0.1
        rt_factor = 1.0
        self.world.execute_path(path, dt=dt, realtime_factor=rt_factor,
                                linear_velocity=1.0, max_angular_velocity=None)

        # Animate while navigation is active
        # NOTE: Right now blitting is working to make things faster, but we run into
        # a race condition when the ROS wrapper is also in the same thread
        held_obj = self.world.robot.manipulated_object
        is_holding_object = held_obj is not None   
        do_blit = True
        sleep_time = dt / rt_factor
        if do_blit:
            animated_artists = [self.robot_body, self.robot_dir]
            if is_holding_object:
                animated_artists.extend([held_obj.viz_patch, held_obj.viz_text])     
            for a in animated_artists:
                a.set_animated(True)
            self.draw_and_sleep()
            bg = self.fig.canvas.copy_from_bbox(self.fig.bbox)
            while self.world.robot.executing_action:
                time.sleep(sleep_time) # Needs to happen before blitting to avoid race condition
                self.fig.canvas.restore_region(bg)
                self.update_robot_plot()
                if is_holding_object:
                    self.update_object_plot(held_obj)
                self.show_world_state(navigating=True)
                for a in animated_artists:
                    self.axes.draw_artist(a)
                self.fig.canvas.blit(self.fig.bbox)
                self.fig.canvas.flush_events()  
            for a in animated_artists:
                a.set_animated(False)
        else:
            while self.world.robot.executing_action:
                self.update_robot_plot()
                if is_holding_object:
                    self.update_object_plot(held_obj)
                self.show_world_state(navigating=True)
                self.draw_and_sleep()
                time.sleep(sleep_time)

        self.show_world_state()
        self.draw_and_sleep()
        return True

    def pick_object(self, obj_name):
        """ Picks an object """
        success = self.world.pick_object(obj_name)
        if success:
            self.update_object_plot(self.world.robot.manipulated_object)
            self.show_world_state()
            self.draw_and_sleep()
        return success

    def place_object(self, loc_name):
        """ Places an object """
        obj = self.world.robot.manipulated_object
        if obj is None:
            return
        obj.viz_patch.remove()
        success = self.world.place_object(loc_name)
        self.axes.add_patch(obj.viz_patch)
        self.update_object_plot(obj)
        self.show_world_state()
        self.draw_and_sleep()
        return success
