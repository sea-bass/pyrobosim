""" Path execution utilities. """

import time
import threading
import warnings

from ..planning.actions import ExecutionResult, ExecutionStatus
from ..utils.motion import Path
from ..utils.trajectory import get_constant_speed_trajectory, interpolate_trajectory


class ConstantVelocityExecutor:
    """
    Executes a path with a linear trajectory assuming constant
    linear and angular velocity, and that the robot can perfectly
    go to the next pose.
    """

    def __init__(
        self,
        dt=0.1,
        linear_velocity=1.0,
        max_angular_velocity=None,
        validate_during_execution=False,
        validation_dt=0.5,
        validation_step_dist=0.025,
    ):
        """
        Creates a constant velocity path executor.

        :param dt: Time step for creating a trajectory, in seconds.
        :type dt: float
        :param linear_velocity: Linear velocity, in m/s.
        :type linear_velocity: float
        :param max_angular_velocity: Maximum angular velocity, in rad/s.
        :type max_angular_velocity: float, optional
        :param validate_during_execution: If True, runs a separate thread that validates the remaining path at a regular rate.
        :type validate_during_execution: bool
        :param validation_dt: Time step for validating the remaining path, in seconds.
        :type validation_dt: float
        :param validation_step_dist: The step size for discretizing a straight line to check collisions.
        :type validation_step_dist: float
        """
        self.robot = None
        self.dt = dt
        self.linear_velocity = linear_velocity
        self.max_angular_velocity = max_angular_velocity

        self.validation_timer = None
        self.validate_during_execution = validate_during_execution
        self.validation_dt = validation_dt
        self.validation_step_dist = validation_step_dist

        # Execution state
        self.current_traj_time = 0.0
        self.current_distance_traveled = 0.0
        self.abort_execution = False  # Flag to abort internally
        self.cancel_execution = False  # Flag to cancel from user

    def execute(self, path, realtime_factor=1.0):
        """
        Generates and executes a trajectory on the robot.

        :param path: Path to execute on the robot.
        :type path: :class:`pyrobosim.utils.motion.Path`
        :param realtime_factor: A multiplier on the execution time relative to
            real time, defaults to 1.0.
        :type realtime_factor: float, optional
        :return: An object describing the execution result.
        :rtype: :class:`pyrobosim.planning.actions.ExecutionResult`
        """
        if self.robot is None:
            message = "No robot attached to execute the trajectory."
            warnings.warn(message)
            return ExecutionResult(
                status=ExecutionStatus.PRECONDITION_FAILURE,
                message=message,
            )
        elif path.num_poses < 2:
            message = "Not enough waypoints in path to execute."
            warnings.warn(message)
            return ExecutionResult(
                status=ExecutionStatus.PRECONDITION_FAILURE,
                message=message,
            )

        self.robot.executing_nav = True
        self.current_traj_time = 0.0
        self.current_distance_traveled = 0.0
        self.abort_execution = False

        # Convert the path to an interpolated trajectory.
        self.traj = get_constant_speed_trajectory(
            path,
            linear_velocity=self.linear_velocity,
            max_angular_velocity=self.max_angular_velocity,
        )
        traj_interp = interpolate_trajectory(self.traj, self.dt)

        # Optionally, kick off the path validation timer.
        if self.validate_during_execution and self.robot.world is not None:
            self.validation_timer = threading.Thread(
                target=self.validate_remaining_path
            )
            self.validation_timer.start()

        # Execute the trajectory.
        status = ExecutionStatus.SUCCESS
        message = ""
        sleep_time = self.dt / realtime_factor
        is_holding_object = self.robot.manipulated_object is not None
        prev_pose = traj_interp.poses[0]
        for i in range(traj_interp.num_points()):
            start_time = time.time()
            cur_pose = traj_interp.poses[i]
            self.current_traj_time = traj_interp.t_pts[i]
            self.robot.set_pose(cur_pose)
            if is_holding_object:
                self.robot.manipulated_object.set_pose(cur_pose)

            if self.abort_execution:
                if self.validate_during_execution:
                    self.validation_timer.join()
                message = "Trajectory execution aborted."
                warnings.warn(message)
                status = ExecutionStatus.EXECUTION_FAILURE
                break
            if self.cancel_execution:
                self.cancel_execution = False
                message = "Trajectory execution canceled by user."
                warnings.warn(message)
                status = ExecutionStatus.CANCELED
                break

            self.current_distance_traveled += cur_pose.get_linear_distance(prev_pose)
            prev_pose = cur_pose
            time.sleep(max(0, sleep_time - (time.time() - start_time)))

        # Finalize path execution.
        time.sleep(0.1)  # To ensure background threads get the end of the path.
        self.robot.last_nav_result = ExecutionResult(status=status, message=message)
        self.robot.executing_nav = False
        self.robot.executing_action = False
        self.robot.current_action = None
        return self.robot.last_nav_result

    def validate_remaining_path(self):
        """
        Validates the remaining path by checking collisions against the world.

        This function will set the `abort_execution` attribute to `True`,
        which cancels the main trajectory execution loop.
        """
        while self.robot.executing_nav and not self.abort_execution:
            start_time = time.time()
            cur_pose = self.robot.get_pose()
            cur_time = self.current_traj_time

            # Get the waypoint index of the remaining path.
            for idx, t in enumerate(self.traj.t_pts):
                if t >= cur_time:
                    break
            if idx == self.traj.num_points() - 1:
                return

            # Collision check the remaining path.
            poses = [cur_pose]
            poses.extend(self.traj.poses[idx:])
            if len(poses) > 2:
                remaining_path = Path(poses=poses)
                if not remaining_path.is_collision_free(
                    self.robot.world, step_dist=self.validation_step_dist
                ):
                    warnings.warn("Remaining path is in collision. Aborting execution.")
                    self.abort_execution = True

            time.sleep(max(0, self.validation_dt - (time.time() - start_time)))
