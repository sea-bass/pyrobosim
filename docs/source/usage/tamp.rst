Task and Motion Planning
========================
We use `PDDLStream <https://github.com/caelan/pddlstream>`_ to perform integrated task and motion planning (TAMP).
This tool expands task planning with purely discrete parameters using `Planning Domain Definition Language (PDDL) <https://planning.wiki/guide/whatis/pddl>`_ 
by adding the concept of *streams* for sampling continuous parameters in actions.

If you did not already install PDDLStream, ensure you do so with this script.

::

   ./setup/setup_pddlstream.bash


Examples
--------
Regardless of running PDDLStream standalone or using ROS2, we have included a set of examples
that gradually build up from simple, purely discrete planning, to a more complex integrated TAMP
demo with continuous action parameters.

The current example list is:

* ``01_simple`` - Simple domain with purely discrete actions
* ``02_derived`` - Purely discrete actions, but uses *derived predicates* for more complex goals
* ``03_nav_stream`` - Samples navigation poses and entire motion plan instances
* ``04_nav_manip_stream`` - Samples navigation poses, motion plans, and collision-free object placement instances

These PDDL domain and stream description files can be found in the ``pyrobosim/pyrobosim/data/pddlstream/domains`` folder.

Standalone
----------

You can try running a sample script as follows

::

    cd /path/to/pyrobosim/pyrobosim
    python examples/demo_pddl.py --example 01_simple verbose

.. image:: ../media/pddlstream_demo_standalone.png
    :align: center
    :width: 720px
    :alt: PDDLStream standalone demo.

|

With ROS2
---------

First, build and setup the ``colcon`` workspace (or use one of our provided Docker containers).

::

    cd /path/to/colcon/workspace
    colcon build
    . install/local_setup.bash


With ROS2, the idea is to separate out functionality into different *nodes*.

To start a world and then a planner with a hard-coded goal specification:

::

    ros2 run pyrobosim_ros demo_pddl_world.py --example 01_simple
    ros2 run pyrobosim_ros demo_pddl_planner.py --example 01_simple --verbose

To start a world, a planner, and a separate node that publishes a goal specification:

::

    ros2 run pyrobosim_ros demo_pddl_world.py --example 01_simple
    ros2 run pyrobosim_ros demo_pddl_planner.py --example 01_simple --verbose --subscribe
    ros2 run pyrobosim_ros demo_pddl_goal_publisher.py --example 01_simple

The output should look as follows:

.. image:: ../media/pddlstream_demo_ros.png
    :align: center
    :width: 720px
    :alt: PDDLStream demo with ROS2.

|
