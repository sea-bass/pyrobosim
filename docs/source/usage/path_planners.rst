.. _path_planners:

Path Planners
=============

This section explains how to use (and optionally extend) path planners in ``pyrobosim``.

Path Planner Definitions
------------------------

The ``pyrobosim/navigation`` module contains all path planner implementations.

In the ``__init__.py`` file available, you will see a list of implemented planner classes that looks like this:

.. code-block:: python

    # Planners
    from .a_star import AStarPlanner
    from .rrt import RRTPlanner
    from .prm import PRMPlanner
    from .world_graph import WorldGraphPlanner


    PATH_PLANNERS_MAP = {
        "astar": AStarPlanner,
        "rrt": RRTPlanner,
        "prm": PRMPlanner,
        "world_graph": WorldGraphPlanner,
    }

When loading path planners from YAML, the ``planner.type`` entry will correspond to one of these entries in the ``PATH_PLANNERS_MAP`` dictionary.
As such, if you would like to add your own planner, you can do so in this file.

.. code-block:: python

    # Planners
    from .a_star import AStarPlanner
    from .rrt import RRTPlanner
    from .prm import PRMPlanner
    from .world_graph import WorldGraphPlanner
    from my_module.my_file import MyNewPlanner


    PATH_PLANNERS_MAP = {
        "astar": AStarPlanner,
        "rrt": RRTPlanner,
        "prm": PRMPlanner,
        "world_graph": WorldGraphPlanner,
        "my_planner": MyNewPlanner,
    }


What to Implement in a Planner
------------------------------

The path planners implemented in ``pyrobosim`` provide general functionality to plan paths in a known world, as well as data for visualization in the UI.

First, you want to make a constructor that accepts a world in a ``world`` keyword argument, along with any other data you might expect.
For example, this planner accepts a world and a few parameters and builds an occupancy grid.

.. code-block:: python

    class MyNewPlanner:
        def __init__(self, *, world, grid_resolution, grid_inflation_radius):
            self.grid = OccupancyGrid.from_world(
                world,
                resolution=grid_resolution,
                inflation_radius=grid_inflation_radius,
            )

Next, you should implement a ``reset()`` method.
This is needed if, for example, the world changes and you want to generate new persistent data structures such as a new occupancy grid or roadmap.
If your planner does not have any such data, you still must implement this.

.. code-block:: python

    class MyNewPlanner:
        def __init__(self, *, world, grid_resolution, grid_inflation_radius):
            self.world = world
            self.grid_resolution = grid_resolution
            self.grid_inflation_radius = grid_inflation_radius
            self.reset()

        def reset(self):
            self.grid = OccupancyGrid.from_world(
                self.world,
                resolution=self.grid_resolution,
                inflation_radius=self.grid_inflation_radius,
            )

Then, you need to implement the actual path planning.
This is done using a ``plan()`` method that accepts a start and goal pose and returns a ``Path`` object.

.. code-block:: python

    import time
    from pyrobosim.utils.motion import Path

        def plan(self, start, goal):
            t_start = time.time()
            # Your planning logic goes here
            return Path(
                poses=[start, goal],
                planning_time=time.time() - t_start
            )

For visualization, you can provide ``get_graphs()`` and ``get_latest_paths()`` methods.

.. code-block:: python

    from pyrobosim.utils.search_graph.SearchGraph

        def plan(self, start, goal):
            t_start = time.time()
            self.search_graph = SearchGraph()

            # Your planning logic goes here

            self.latest_path = Path(
                poses=[start, goal],
                planning_time=time.time() - t_start
            )
            return self.latest_path

        def get_graphs(self):
            return [SearchGraph()]

        def get_latest_path(self):
            return self.latest_path

To serialize to file, which is needed to reset the world, you should also implement the ``to_dict()`` method.
Note the ``get_planner_string()`` helper function, which extracts the name of the planner you defined in ``PATH_PLANNERS_MAP`` earlier on.

.. code-block:: python

        def to_dict(self, start, goal):
            from pyrobosim.navigation import get_planner_string

            return {
                "type": get_planner_string(self),
                "grid_resolution": self.grid_resolution,
                "grid_inflation_radius": self.grid_inflation_radius,
            }

If you would like to implement your own path planner, it is highly recommended to look at the existing planner implementations as a reference.
You can also always ask the maintainers through a Git issue!
