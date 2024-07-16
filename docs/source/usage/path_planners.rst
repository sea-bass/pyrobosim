.. _path_planners:

Path Planners
=============

This section explains how to use and optionally extend the path planners in ``pyrobosim``.

``PathPlanner`` Interface
-------------------------

``pyrobosim`` uses an interface based approach to path planners.
A specific planner can be selected through the planner interface, by providing the planner type and a set of planner-specific keyword arguments.

.. code-block:: python

    from pyrobosim.navigation import PathPlanner

    path_planner = PathPlanner("planner_type", **planner_config)

Available planner types and their available implementations can be found below :

- ``"astar"`` : :py:mod:`pyrobosim.navigation.a_star`
- ``"prm"`` : :py:mod:`pyrobosim.navigation.prm`
- ``"rrt"`` : :py:mod:`pyrobosim.navigation.rrt`

Each planner can potentially have multiple implementations.
A specific implementation can be selected by providing the relevant parameters in the keyword arguments contained in ``**planner_config``.


Adding New Planners
-------------------

The path planners in ``pyrobosim`` are designed to be extensible, so that you can add your own implementation for an existing planner type, or even a new planner type.

For example, to add a new planner type called ``NewPlanner``:

- Create the planner factory class ``NewPlanner`` which inherits :py:mod:`pyrobosim.navigation.planner_base`.

    .. code-block:: python

        from pyrobosim.navigation import PathPlannerBase

        class NewPlanner(PathPlannerBase):
            pass

- Create concrete implementations of your planner similar to :py:class:`pyrobosim.navigation.rrt.RRTPlannerPolygon`.

    .. code-block:: python

        class NewPlannerPolygon:
            pass

- Next, you should specify the mechanism to select the concrete implementation of your planner type in the ``__init__()`` method, and implement the ``plan()`` method.
  Refer to :py:class:`pyrobosim.navigation.rrt.RRTPlanner` for an example on how to do this.

    .. code-block:: python

        from pyrobosim.navigation import PathPlannerBase

        class NewPlanner(PathPlannerBase):

            def __init__(self, **planner_config):
                # Select the implementation here
                if planner_config.get("some_param", None):
                    self.impl = NewPlannerPolygon(**planner_config)
                else:
                    raise NotImplementedError("This configuration is not valid!")

            def plan(self, start, goal):
                # Call implementations to compute path , do any pre- or post- processing.
                path = self.impl.plan(start, goal)
                return path

- Add the planner type to the list of supported planners in :py:mod:`pyrobosim.navigation.path_planner`.

    .. code-block:: python

        self.planners = {
            "astar": AstarPlanner,
            "rrt": RRTPlanner,
            "prm": PRMPlanner,
            "new_planner": NewPlanner,  # Here is our new planner!
        }

- Use the ``PathPlanner`` interface to use your new planner.

    .. code-block:: python

        planner_config = {"some_param": some_value, "some_other_param": some_other_value}
        new_path_planner  = PathPlanner("new_planner", **planner_config)

.. note::

    Planner implementations that need to display graphs and planned paths should set the ``graphs`` and ``latest_path`` attributes of ``PathPlannerBase``.
    Refer to :py:func:`pyrobosim.navigation.rrt.RRTPlanner.plan` and :py:func:`pyrobosim.navigation.prm.PRMPlanner.plan` for some example implementations.
