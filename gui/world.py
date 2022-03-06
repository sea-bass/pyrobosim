import adjustText
import numpy as np
from matplotlib.figure import Figure
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg

class WorldGUI(FigureCanvasQTAgg):
    def __init__(self, world, width=5, height=4, dpi=100):
        self.fig = Figure(figsize=(width, height), dpi=dpi)
        self.axes = self.fig.add_subplot(111)
        self.axes.set_title("World Model")

        self.world = world
        self.createRobot()

        super(WorldGUI, self).__init__(self.fig)

    def createRobot(self):
        normalized_length = 0.1
        plot_length = normalized_length * max(
            (self.world.x_bounds[1] - self.world.x_bounds[0]),
            (self.world.y_bounds[1] - self.world.y_bounds[0]))

        p = self.world.robot.pose
        self.robot_body, = self.axes.plot(
            p.x, p.y, 
            "mo", markersize=10, markeredgewidth=2, 
            markerfacecolor="None")
        self.robot_dir, = self.axes.plot(
            p.x + np.array([0, plot_length*np.cos(p.yaw)]),
            p.y + np.array([0, plot_length*np.sin(p.yaw)]),
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
            t = self.axes.text(x, y, obj.name,
                           color=obj.viz_color, fontsize=8)
            self.obj_texts.append(t)
        obj_patches = [o.viz_patch for o in (self.world.objects)]

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
            if self.world.current_path is not None:
                x = [p.x for p in self.world.current_path]
                y = [p.y for p in self.world.current_path]
                self.axes.plot(x, y, "m-", linewidth=3, zorder=1)       # Entire path
                self.axes.scatter(x[0], y[0], 60, "g", "o", zorder=2)   # Start
                self.axes.scatter(x[-1], y[-1], 60, "r", "x", zorder=2) # Goal

        self.axes.autoscale()
        self.axes.axis("equal")
        adjustText.adjust_text(self.obj_texts, lim=100, add_objects=obj_patches)
