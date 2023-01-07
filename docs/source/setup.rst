Setup
=====

This package is being tested with:

* Python 3.8 in Ubuntu 20.04, optionally with ROS 2 Foxy
* Python 3.10 in Ubuntu 22.04, optionally with ROS 2 Humble

Local Setup
-----------

If using ROS 2, clone this repo in a valid `colcon workspace <https://docs.ros.org/en/humble/Tutorials/Workspace/Creating-A-Workspace.html>`_.
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

If you plan to use ROS 2, you can similarly create a bash function like this:

::

    pyrobosim_ros() {
       source /path/to/pyrobosim/setup/source_pyrobosim.bash humble
    }


Docker Setup
------------

We also provide Docker images compatible with ROS 2 releases.

If you already have sourced ROS 2 in your system (e.g., ``source /opt/ros/humble/setup.bash``),
then you should have a ``ROS_DISTRO`` environment variable set. Otherwise,

::

    export ROS_DISTRO=humble
    docker compose build

Then, you can start any of the existing demos.
To see all the available services, refer to the ``docker-compose.yaml`` file or use Tab completion.

::

    docker compose up demo
    docker compose up demo_ros

Alternatively, you can start a Docker container.

::

    docker compose run base
    # python3 src/pyrobosim/examples/demo.py

You can also start a new Terminal and go into the same container as follows.

::

    docker exec -it <pyrobosim-base-1> bash
    # ros2 launch pyrobosim_ros demo.py

The source code on your host machine is mounted as a volume, 
so you can make modifications on your host and rebuild the Colcon workspace inside the container.
