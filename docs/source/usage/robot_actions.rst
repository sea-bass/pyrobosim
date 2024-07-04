Robot Actions
=============
``pyrobosim`` enables you to command your robot to take high-level actions.

The actions currently supported are:

* **Move**: Uses a path planner to navigate to a specific location. Refer to :ref:`path_planners` for more details.
* **Pick**: Picks up an object at the robot's current location.
* **Place**: Places an object at the robot's current location.
* **Detect**: Detects objects at the robot's current location. Refer to :ref:`partial_observability` for more details.

Actions can be triggered in a variety of ways:

* Through the buttons on the GUI.
* Using a robot's ``execute_action()`` method.
* (If using ROS), sending a goal to the ``/execute_action`` ROS action server.

You can also command a robot with a *plan*, which is a sequences of actions:

* Using a robot's ``execute_plan()`` method.
* (If using ROS), sending a goal to the ``/execute_task_plan`` ROS action server.

Plans can also be automatically generated with :ref:`_task_and_motion_planning`.


.. _partial_observability:

Partial Observability
---------------------
By default, all robots have full knowledge of all the objects in the world.

A common use case for design robot behaviors is that a robot instead starts with limited or no knowledge of objects.
In these cases, the robot must explicitly go to a location and use an object detector to find new objects to add to their world model.

You can model this in ``pyrobosim`` by instantiating robot objects with the ``partial_observability`` option set to ``True``.
Then, you can use the **detect** action to find objects at the robot's current location.

To test this, you can run the following example.

::

    cd /path/to/pyrobosim/pyrobosim
    python3 examples/demo.py --multirobot --partial-observability

In the GUI, selecting a robot in the drop-down menu will only display the objects locally known to that robot.
Alternatively, you can select the ``world`` option to show all existing objects.

.. image:: ../media/pyrobosim_partial_observability.png
    :align: center
    :width: 720px
    :alt: Partial observability in the pyrobosim GUI.

|
