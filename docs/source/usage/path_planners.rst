.. _path_planners:

Path Planners
=============

This section explains how to use (and optionally extend) path planners in PyRoboSim.

Path Planner Definitions
------------------------

The ``pyrobosim/navigation`` module contains all path planner implementations.

What to Implement in a Planner
------------------------------

The path planners implemented in PyRoboSim provide general functionality to plan paths in a known world, as well as data for visualization in the UI.

You should implement a ``reset()`` method.
This is needed if, for example, the world changes and you want to generate new persistent data structures such as a new occupancy grid or roadmap.
Note that the parent class' method can be called using ``super().reset()``, which automatically resets common attributes such as ``latest_path``, ``robot``, and ``world``.

.. code-block:: python

    from pyrobosim.navigation.types import PathPlanner
    from pyrobosim.utils.path import Path

    class MyNewPlanner(PathPlanner):

        plugin_name = "my_planner"  # Needed to register the plugin

        def __init__(
            self,
            *,
            grid_resolution: float,
            grid_inflation_radius: float,
        ) -> None:
            super().__init__()
            self.reset()

        def reset(self) -> None:
            super().reset()
            if self.world is not None:
                self.grid = OccupancyGrid.from_world(
                    world,
                    resolution=grid_resolution,
                    inflation_radius=grid_inflation_radius,
                )

Then, you need to implement the actual path planning.
This is done using a ``plan()`` method that accepts a start and goal pose and returns a ``Path`` object.

.. code-block:: python

    import time
    from pyrobosim.utils.pose import Pose

        def plan(self, start: Pose, goal: Pose) -> Path:
            t_start = time.time()

            # Your planning logic goes here

            return Path(
                poses=[start, goal],
                planning_time=time.time() - t_start
            )

For visualization, you can provide ``get_graphs()`` and ``get_latest_paths()`` methods.

.. code-block:: python

    from pyrobosim.utils.search_graph.SearchGraph

        def plan(self, start: Pose, goal: Pose) -> Path:
            t_start = time.time()
            self.search_graph = SearchGraph()

            # Your planning logic goes here

            self.latest_path = Path(
                poses=[start, goal],
                planning_time=time.time() - t_start
            )
            return self.latest_path

        def get_graphs(self) -> list[SearchGraph]:
            return [SearchGraph()]

        def get_latest_path(self) -> Path:
            return self.latest_path

To serialize to file, which is needed to reset the world, you should also implement the ``to_dict()`` method.
Note the ``plugin_name`` attribute, which contains the name of the planner you defined earlier on.

.. code-block:: python

        def to_dict(self) -> dict[str, Any]:
            return {
                "type": self.plugin_name,
                "grid_resolution": self.grid_resolution,
                "grid_inflation_radius": self.grid_inflation_radius,
            }

At this point, you can import your own path planner in code and load it dynamically using the ``PathPlanner`` parent class.

.. code-block:: python

    from pyrobosim.navigation import PathPlanner
    from my_module import MyNewPlanner  # Still need to import this!

    planner_class = PathPlanner.registered_plugins["my_planner"]
    planner = planner_class(grid_resolution=0.01, grid_inflation_radius=0.1)

... or from YAML world files.

.. code-block:: yaml

    robots:
      name: robot
      path_planner:
        type: my_planner
        grid_resolution: 0.01
        grid_inflation_radius: 0.1

If you would like to implement your own path planner, it is highly recommended to look at the existing planner implementations as a reference.
You can also always ask the maintainers through a Git issue!
