Robot Dynamics
==============
While the main focus of ``pyrobosim`` is to command robots with high-level actions, it is also possible to directly send velocity commands to your robots for other applications.

When you instantiate a :py:class:`pyrobosim.core.robot.Robot` object, it also gets created with a :py:class:`pyrobosim.utils.dynamics.RobotDynamics2D` object.

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

Notice that the ability to pass in a world and check collisions is optional.
If you want to optimize for speed and do not need collision checking, you can disable this.

Also, you can check whether a robot's previous velocity command caused a collision and the robot did not move.

.. code-block:: python

    # Robot collided, let's command it to back up!
    if robot.is_in_collision():
        cmd_vel = np.array([-0.5, 0.0, 0.0])

Finally, if you want to run with the GUI up, you must ensure that the GUI is running on the main thread and the dynamics are on a separate thread.
You can do this as follows, using our ``command_robots()`` function.

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
