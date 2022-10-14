# pyrobosim

[![Pyrobosim Tests](https://github.com/sea-bass/pyrobosim/actions/workflows/actions.yml/badge.svg?branch=main)](https://github.com/sea-bass/pyrobosim/actions/workflows/actions.yml)
[![Documentation Status](https://readthedocs.org/projects/pyrobosim/badge/?version=latest)](https://pyrobosim.readthedocs.io/en/latest/?badge=latest)

ROS2 enabled 2D mobile robot simulator for behavior prototyping.

By Sebastian Castro, 2022

Refer to the [full documentation](https://pyrobosim.readthedocs.io/) for setup, usage, and other concepts.

![Example animation of the simulator](docs/source/media/pyrobosim_demo.gif)

### Simulation of a Tiago mobile manipulator

With ROS2 Humble, a Tiago mobile manipulator can be simulated.

#### Tiago installation

These directions assume ROS2 Humble and pyrobosim have already been installed.

First clone the dependencies from `tiago_simulation.repos`:

```bash
cd your_ros_workspace/src
for repo in pyrobosim/tiago_simulation.repos $(f="pyrobosim/tiago_simulation.repos"; test -r $f && echo $f); do vcs import < "$repo"; done
rosdep install -r --from-paths . --ignore-src --rosdistro $ROS_DISTRO -y
colcon build
```

Then install MoveIt2 per the [official instructions](https://moveit.ros.org/install-moveit2/source/).

#### Tiago usage

This simple example navigates the Tiago robot to the entity "bedroom".

First launch Tiago in the Ignition simulator. Both Ignition and RViz windows should launch.

```bash
ros2 launch tiago_2dnav_gazebo tiago_navigation_gazebo.launch.py
```

As of now, there is no automated way to tuck Tiago's arm. So, open `rqt` and use the joint sliders to do it manually:

`rqt` --> Plugins --> Robot tools --> Joint trajectory controller

Move the joints to approximately [0 1 -3.46 1.24 -0.21 0 0]

Launch pyrobosim:

```bash
ros2 run pyrobosim_ros demo.py --ros-args -p world_file:=test
```

Finally, send an action request to navigate to the bedroom:

```bash
ros2 run pyrobosim_ros demo_commands.py --ros-args -p mode:=action
```
