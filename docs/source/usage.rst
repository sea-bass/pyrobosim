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

    python scripts/test_script.py

With ROS2
---------

First, build and setup the colcon workspace.

::

    colcon build
    . install/local_setup.bash


The ROS-free option is:

::

    ros2 run pyrobosim test_script.py 


The ROS option is (in 2 separate Terminals):

::

    ros2 run pyrobosim test_script.py --ros
    ros2 run pyrobosim test_command_publisher.py


Creating Worlds from YAML
-------------------------
TODO


Exporting Worlds to Gazebo
--------------------------
TODO
