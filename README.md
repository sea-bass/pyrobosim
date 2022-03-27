# pyrobosim
ROS2 enabled 2D mobile robot simulator for behavior prototyping.

![Example animation of the simulator](media/pyrobosim.gif)

By Sebastian Castro, 2022

---

## Setup
This package is being developed with Python 3.8 in Ubuntu 20.04, and if you use the optional ROS2 interface, with ROS2 Foxy.

If using ROS2, clone this repo in a valid colcon workspace. Otherwise, clone it wherever you would like.

To set up your Python virtual environment, configure and run:

```
./setup/create_python_env.bash
```

By default, this will create a Python virtual environment in `~/python-virtualenvs/pyrobosim`.

To then source this virtual environment, run:

```
source ./setup/setup_pyrobosim.bash
```

As documented in the above script, I recommend making a bash function in your `~/.bashrc` script so you can easily just call `pyrobosim_ros` from your Terminal to get started.

```
pyrobosim_ros() {
  source /path/to/pyrobosim/setup/setup_pyrobosim.bash
}
```

---

## Usage
This package works with or without ROS2, and with or without a PyQt5 based GUI for visualization. 

The entry points are still in progress, but this is what works now.

### With ROS

First, build and setup the colcon workspace.

```
colcon build
. install/local_setup.bash
```

The ROS-free option is:
```
ros2 run pyrobosim test_script.py 
```

The ROS option is (in 2 separate Terminals):
```
ros2 run pyrobosim test_script.py --ros
ros2 run pyrobosim test_command_publisher.py
```

### Without ROS

First, install the module.

```
cd /path/to/pyrobosim
pip install .
```

Then, run the test script.

```
python scripts/test_script.py
```
