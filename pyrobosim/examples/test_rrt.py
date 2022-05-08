#!/usr/bin/env python
import os

from pyrobosim.core.yaml import WorldYamlLoader
from pyrobosim.navigation.rrt import RRTPlanner
from pyrobosim.navigation.trajectory import fill_path_yaws
from pyrobosim.utils.general import get_data_folder
from pyrobosim.utils.pose import Pose

if __name__=="__main__":

    # Load a world
    data_folder = get_data_folder()
    loader = WorldYamlLoader()
    w = loader.from_yaml(os.path.join(data_folder, "test_world.yaml"))
    w.search_graph = None

    # Create an RRT planner and plan
    rrt = RRTPlanner(w, bidirectional=True, rrt_star=True)
    start = Pose(x=-0.5, y=-0.5)
    goal = Pose(x=3.0, y=3.0)
    w.robot.set_pose(start)
    w.current_path = rrt.plan(start, goal)
    w.current_path = fill_path_yaws(w.current_path)
    print([n.pose for n in w.current_path])

    # Plot hacks: Make this work with the GUI
    from pyrobosim.gui.world_canvas import WorldCanvas
    import matplotlib.pyplot as plt
    wc = WorldCanvas(w)

    managed_fig = plt.figure()
    canvas_manager = managed_fig.canvas.manager
    canvas_manager.canvas.figure = wc.fig
    wc.fig.set_canvas(canvas_manager.canvas)

    wc.show()

    # Plot the RRT
    for e in rrt.graph.edges:
        x = (e.n0.pose.x, e.n1.pose.x)
        y = (e.n0.pose.y, e.n1.pose.y)
        wc.axes.plot(x, y, "k:", linewidth=1)
    if rrt.bidirectional:
        for e in rrt.graph_goal.edges:
            x = (e.n0.pose.x, e.n1.pose.x)
            y = (e.n0.pose.y, e.n1.pose.y)
            wc.axes.plot(x, y, "b--", linewidth=1)
    

    plt.show()

    