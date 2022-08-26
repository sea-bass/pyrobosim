Setup
=====

This package is being tested with:

* Python 3.8 in Ubuntu 20.04, optionally with ROS2 Foxy
* Python 3.10 in Ubuntu 22.04, optionally with ROS2 Humble

Local Setup
-----------

If using ROS2, clone this repo in a valid `colcon workspace <https://docs.ros.org/en/foxy/Tutorials/Workspace/Creating-A-Workspace.html>`_.
Otherwise, if running standalone, clone it wherever you would like.

To set up your Python virtual environment, configure and run

::

  ./setup/create_python_env.bash

By default, this will create a Python virtual environment in ``~/python-virtualenvs/pyrobosim``.

If you want to use `PDDLStream <https://github.com/caelan/pddlstream>`_ for 
task and motion planning, you should also run:

::

   ./setup/setup_pddlstream.bash

To then source this virtual environment, run

::

    source ./setup/source_pyrobosim.bash

As documented in the above script, I recommend making a bash function in your ``~/.bashrc`` script so you can easily just call `pyrobosim` from your Terminal to get started.

::

    pyrobosim() {
       source /path/to/pyrobosim/setup/source_pyrobosim.bash
    }

If you plan to use ROS2, you can similarly create a bash function like this:

::

    pyrobosim_ros() {
       source /path/to/pyrobosim/setup/source_pyrobosim.bash humble
    }


Docker Setup
------------

We also provide Docker images compatible with ROS2 Foxy and Humble releases.

If you already have sourced ROS2 in your system (e.g., ``source /opt/ros/humble/setup.bash``),
then you should have a ``ROS_DISTRO`` environment variable set. Otherwise,

::

    export ROS_DISTRO=humble
    ./docker/build_docker.bash
    ./docker/run_docker.bash

Alternatively, you can directly run a command in a Docker container:

::

    ./docker/run_docker.bash "ros2 launch pyrobosim_ros demo.py"

Colcon workspace build artifacts are shared across containers by mounting volumes,
so you can run a different command in a new Terminal without rebuilding.
