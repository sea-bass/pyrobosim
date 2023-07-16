Setup
=====

This package is being tested with:

* Python 3.10 in Ubuntu 22.04
* Optionally with ROS 2 Humble and Iron

pip install (Limited)
---------------------

You can quickly install ``pyrobosim`` through PyPi.
However, note that this will not include any of the ROS 2 or Task and Motion Planning functionality.

::

  pip install pyrobosim

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

As documented in the above script, we recommend making a bash function in your ``~/.bashrc`` script so you can easily just call `pyrobosim` from your Terminal to get started.

::

    pyrobosim() {
       source /path/to/pyrobosim/setup/source_pyrobosim.bash
    }

Additional ROS 2 Setup
----------------------

After you have installed ``pyrobosim`` and activated your Python virtual environment,
you must build your colcon workspace to install the ``pyrobosim_msgs`` and ``pyrobosim_ros`` packages.
For example, if you have cloned this repo to ``~/pyrobosim_ws/src/pyrobosim``, you can do:

::

    cd ~/pyrobosim_ws
    colcon build
    . install/local_setup.bash

For ROS 2 workflows, you can also make bash function to get set up like this:

::

    pyrobosim_ros() {
       source /path/to/pyrobosim/setup/source_pyrobosim.bash humble
    }

The additional ``humble`` argument will make sure that ROS 2 Humble and your built colcon workspace are sourced in addition to activating the Python virtual environment.

Docker Setup
------------

We also provide Docker images compatible with ROS 2 releases.

If you already have sourced ROS 2 in your system (e.g., ``source /opt/ros/humble/setup.bash``),
then you should have a ``ROS_DISTRO`` environment variable set.
Otherwise,

::

    export ROS_DISTRO=humble
    docker compose build

Then, you can start any of the existing demos.
To see all the available services, refer to the ``docker-compose.yaml`` file or use Tab completion.

::

    docker compose run demo
    docker compose run demo_ros

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
