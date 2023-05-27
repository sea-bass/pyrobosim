Path Planners
=======================
This section explains how to use and optionally extend the path planners in ``pyrobosim``.

PathPlanner Interface
---------------------

``pyrobosim`` uses an interface based approach to path planners.

::

    from pyrobosim.navigation import PathPlanner
    path_planner = PathPlanner(planner_type, **planner_config)


A specific planner can be selected through the planner interface, by providing the ``planner_type`` and a configuration given as ``**planner_config``.

Each planner can potentially have multiple implementations.
A specific implementation can be selected by providing the relevant parameters in ``**planner_config``.

Available planner types and their available implementations can be found below :

- "astar" : :py:mod:`pyrobosim.navigation.a_star`
- "prm" : :py:mod:`pyrobosim.navigation.prm`
- "rrt" : :py:mod:`pyrobosim.navigation.rrt`


Adding New Planners
-------------------

The path planners in ``pyrobosim`` are designed to be extensible, so that you can add your own implementation for an existing planner type, or even a new planner type.

For example, to add a new planner type called ``NewPlanner``:

- Create the planner factory class ``NewPlanner`` which inherits :py:mod:`pyrobosim.navigation.planner_base`

::

    from pyrobosim.navigation import PlannerBase

    class NewPlanner(PlannerBase):
        pass

- Create concrete implementations of your planner similar to :py:class:`pyrobosim.navigation.rrt.RRTPlannerPolygon`

::

    class NewPlannerPolygon:
        pass


- Next, you should specify the mechanism to select the concrete implementation of your planner type in the ``__init__()``, and implement the ``plan()`` method.
  Refer to :py:class:`pyrobosim.navigation.rrt.RRTPlanner` for an example on how to do this.

::

    from pyrobosim.navigation import PlannerBase

    class NewPlanner(PlannerBase):

        def __init__():
            # do implementation selection here.

        def plan():
            # Call implementations to compute path , do any pre or post processing


- Add the planner type to the list of supported planners in :py:mod:`pyrobosim.navigation.path_planner`

::

    self.planners = {"astar": AstarPlanner, "rrt": RRTPlanner, "prm": PRMPlanner, "new_planner": NewPlanner}

- Use the ``PathPlanner`` interface to use your new planner.

::

    new_path_planner  = PathPlanner("new_planner", **planner_config)


.. note::

    Planner implementations that need to display graphs should provide a `get_graphs()` method and set the `graphs` attribute of `PlannerBase` like in
    :py:func:`pyrobosim.navigation.rrt.RRTPlannerPolygon.get_graphs` and :py:func:`pyrobosim.navigation.rrt.RRTPlanner.plan`.
