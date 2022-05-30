Setup
=====

This package is being tested with:

* Python 3.8 in Ubuntu 20.04, optionally with ROS2 Foxy
* Python 3.10 in Ubuntu 22.04, optionally with ROS2 Humble

If using ROS2, clone this repo in a valid `colcon workspace <https://docs.ros.org/en/foxy/Tutorials/Workspace/Creating-A-Workspace.html>`_.
Otherwise, if running standalone, clone it wherever you would like.

To set up your Python virtual environment, configure and run

::

  ./setup/create_python_env.bash

By default, this will create a Python virtual environment in ``~/python-virtualenvs/pyrobosim``.

To then source this virtual environment, run

::

    source ./setup/setup_pyrobosim.bash

As documented in the above script, I recommend making a bash function in your ``~/.bashrc`` script so you can easily just call `pyrobosim` from your Terminal to get started.

::

    pyrobosim() {
       source /path/to/pyrobosim/setup/setup_pyrobosim.bash
    }

If you plan to use ROS2, you can similarly create a bash function like this:

::

    pyrobosim_ros() {
       source /path/to/pyrobosim/setup/setup_pyrobosim.bash humble
    }
