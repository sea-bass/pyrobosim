Robot Dynamics
==============

While the main focus of ``pyrobosim`` is to command robots with high-level actions, it is also possible to directly send velocity commands to your robots.

When you instantiate a :py:class:`pyrobosim.core.robot.Robot` object, it contains a :py:class:`pyrobosim.core.dynamics.RobotDynamics2D` object.

The linear and angular velocity limits can be specified here as well.
For example:

.. code-block:: python

    robot0 = Robot(
        name="my_robot",
        radius=0.1,
        max_linear_velocity=1.0,
        max_angular_velocity=3.0,
        max_linear_acceleration=2.0,
        max_angular_acceleration=6.0,
    )

Core API Usage
--------------

You can step each robot's dynamics in a loop of your choice.
For example, you can create a simulation loop as follows:

.. code-block:: python

    def command_robots(world):
        dt = 0.1
        vel_commands = [
            np.array([0.1, 0.0, 0.5]),  # robot0
            np.array([0.0, 0.0, -1.0]),  # robot1
            np.array([0.2, 0.0, 0.0]),  # robot2
        ]
        backup_vel = np.array([-1.0, 0.0, 0.0])

        while True:
            for robot, cmd_vel in zip(world.robots, vel_commands):
                robot.dynamics.step(cmd_vel, dt, world=world, check_collisions=True)
            time.sleep(dt)

Note that the ability to pass in a world and check collisions is optional.
If you want to optimize for speed and do not need collision checking, you can disable this.

Also, you can check whether a robot's previous velocity command caused a collision and the robot did not move.

.. code-block:: python

    # Robot collided, let's command it to back up!
    if robot.is_in_collision():
        cmd_vel = np.array([-0.5, 0.0, 0.0])

Finally, if you want to run with the GUI, you must ensure that the GUI is running on the main thread and the dynamics are on a separate thread.
You can do this as follows, using our ``command_robots()`` function above.

.. code-block:: python

    from threading import Thread

    # Command robots on a separate thread.
    robot_commands_thread = Thread(target=lambda: command_robots(world))
    robot_commands_thread.start()

    # Start the program either as ROS node or standalone.
    start_gui(world)

The full example is available here for you to run and modify.

::

    python3 examples/demo_dynamics.py


ROS 2 Interface Usage
---------------------

If you launch ``pyrobosim`` with the ROS 2 interface, you can command each robot using ROS topics.

For example, a robot named ``my_robot`` will have a topic ``my_robot/cmd_vel`` of type ``geometry_msgs/Twist``.
To command this robot from an existing ROS 2 node in Python, you can do something like this:

.. code-block:: python

    from geometry_msgs.msg import Twist

    vel_pub = node.create_publisher(Twist, "my_robot/cmd_vel", 10)

    vel_cmd = Twist()
    vel_cmd.linear.x = 0.25
    vel_cmd.angular.z = 1.0

    vel_pub.publish(vel_cmd)

To handle the nondeterminism of publishing velocity commands using ROS topics, the :py:class:`pyrobosim_ros.ros_interface.WorldROSWrapper` class provides arguments to latch velocity commands and then ramp them down to zero velocity.
While you can look at the documentation for a full list of arguments, the important ones to know are:

.. code-block:: python

    from pyrobosim_ros.ros_interface import WorldROSWrapper

    node = WorldROSWrapper(
        dynamics_rate=0.01,           # Dynamics update rate
        dynamics_latch_time=0.5,      # Velocity command latch time
        dynamics_ramp_down_time=0.5,  # Velocity command ramp down time
    )

You can try this out using the following example.

::

    # Launch pyrobosim
    ros2 launch pyrobosim_ros demo.launch.py

    # Launch a simple velocity publisher node
    ros2 run pyrobosim_ros demo_velocity_publisher.py

    # (Optional launch with parameters)
    ros2 run pyrobosim_ros demo_velocity_publisher.py --ros-args -p robot_name:=robot -p lin_vel:=-0.1 -p ang_vel:=0.5

    # (Optional) Modify the velocities at runtime
    ros2 param set demo_velocity_publisher lin_vel -0.1
    ros2 param set demo_velocity_publisher ang_vel 0.25
