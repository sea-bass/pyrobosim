.. _geometry_conventions:

Geometry Conventions
====================

3D orientations can be represented using Euler angles or quaternions.
For simplicity, if you are representing poses in 2D, you can use the yaw angle (Z Euler angle), but we recommend using quaternions for generic 3D pose calculations and for interfacing with ROS.

The Euler angle convention is extrinsic XYZ (roll = X, pitch = Y, yaw = Z) in radians.
Quaternions use the order convention ``[qw, qx, qy, qz]``.
There is more documentation in the :py:class:`pyrobosim.utils.pose.Pose` source code.

When ROS 2 is connected to PyRoboSim, the reference frame is ``map``.
