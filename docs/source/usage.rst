Usage
=====
This package works with or without ROS2, and with or without a PyQt5 based GUI for visualization. 

The entry points are still in progress, but this is what works now.

Standalone
----------

First, install the module.

::

    cd /path/to/pyrobosim
    pip install .

Then, run the test script.

::

    python scripts/demo.py

With ROS2
---------

First, build and setup the colcon workspace.

::

    colcon build
    . install/local_setup.bash


The standalone (ROS-free) option is:

::

    ros2 run pyrobosim demo.py 


The ROS option is (in 2 separate Terminals):

::

    ros2 run pyrobosim demo.py --ros
    ros2 run pyrobosim demo_commands.py

The first command will start a world as a ROS2 node, and the second one will publish a plan (or set of actions) to the node.


Creating Worlds
---------------
Worlds can be created either with the pyrobosim API, or loaded from a YAML file using the :doc:`WorldYamlLoader </generated/pyrobosim.world.yaml.WorldYamlLoader>` utility:

By default, ``demo.py`` creates a world using the API, but you can alternative try a demo YAML file using the ``--from-file`` argument. For example:

::

    python scripts/demo.py --from-file
    ros2 run pyrobosim demo.py --ros --from-file

Refer to the World YAML Specification (TODO) for more information.

Also, all locations and objects must be specified as a category before spawning in the world.
Refer to the Location YAML Specification (TODO) and Object YAML Specification (TODO) for more information.


Exporting Worlds to Gazebo
--------------------------
To export words to Gazebo, there is a :doc:`WorldGazeboExporter </generated/pyrobosim.world.gazebo.WorldGazeboExporter>` utility:

Standalone:

::

    python scripts/demo_world_save.py

ROS:

::

    ros2 run pyrobosim demo_world_save.py

You can then follow the steps to see the generated world.
