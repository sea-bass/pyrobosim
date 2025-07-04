Setup
=====

This package is being tested with:

* Python 3.10 / Ubuntu 22.04 (optionally with ROS 2 Humble)
* Python 3.12 / Ubuntu 24.04 (optionally with ROS 2 Jazzy, Kilted, or Rolling)

pip install (Limited)
---------------------

You can quickly install PyRoboSim through PyPi.
However, note that this will not include any of the ROS 2 or Task and Motion Planning functionality.

::

  pip install pyrobosim


Local Setup
-----------

If using ROS 2, clone this repo in a valid `ROS 2 workspace <https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html>`_.
Otherwise, if running standalone, clone it wherever you would like.

To set up your Python virtual environment, configure and run

::

  ./setup/setup_pyrobosim.bash

By default, this will create a Python virtual environment in ``~/python-virtualenvs/pyrobosim``.
You can change this location by modifying the script.

When you run this script, you will get prompts for optionally setting up ROS 2 and PDDLStream for task and motion planning.

To then setup the environment, run

::

    source ./setup/source_pyrobosim.bash

We recommend making a bash function in your ``~/.bashrc`` file (or equivalent) so you can easily just call ``pyrobosim`` from your shell to get started.

::

    pyrobosim() {
       source /path/to/pyrobosim/setup/source_pyrobosim.bash
    }

.. note::
    The ``setup_pyrobosim_bash`` script will create a configuration file named ``pyrobosim.env`` in the repo root folder.
    This file is then read by the ``source_pyrobosim.bash`` script to start up the environment.

    You can edit this file manually if you're comfortable with how these settings work.
    Otherwise, we recommend rerunning ``setup_pyrobosim.bash`` and following the prompts if you want to make changes to your setup.


Docker Setup
------------

We also provide Docker images compatible with supported ROS 2 distributions.

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

Alternatively, you can start a Docker container without running any examples.

::

    docker compose run base
    # python3 src/pyrobosim/examples/demo.py

From a new Terminal, you can then start a new interactive shell in the same container as follows.

::

    docker exec -it <pyrobosim-base-1> bash
    # ros2 launch pyrobosim_ros demo.py

The source code on your host machine is mounted as a volume.
This means you can make modifications on your host and rebuild the ROS 2 workspace inside the container.
