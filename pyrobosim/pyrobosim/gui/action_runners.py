""" Utilities for executing actions from the UI in separate QThreads. """

from PySide6.QtCore import QRunnable


class NavRunner(QRunnable):
    """
    Helper class that wraps navigation execution in a QThread.
    """

    def __init__(self, canvas, robot, goal, path):
        """
        Creates a navigation execution thread.

        :param canvas: A world canvas object linked to this thread.
        :type canvas: :class:`pyrobosim.gui.world_canvas.WorldCanvas`
        :param robot: Robot instance or name to execute action.
        :type robot: :class:`pyrobosim.core.robot.Robot` or str
        :param goal: Name of goal location (resolved by the world model).
        :type goal: str
        :param path: Path to goal location, defaults to None.
        :type path: :class:`pyrobosim.utils.motion.Path`, optional
        """
        super(NavRunner, self).__init__()
        self.canvas = canvas
        self.robot = robot
        self.goal = goal
        self.path = path

    def run(self):
        """Runs the navigation execution thread."""
        robot = self.robot
        world = self.canvas.world

        if isinstance(robot, str):
            robot = world.get_robot_by_name(robot)
        if robot is None:
            self.world.logger.warning("Robot is not specified. Cannot navigate.")
            return

        robot.navigate(
            goal=self.goal,
            path=self.path,
            realtime_factor=self.canvas.realtime_factor,
        )


class PickRunner(QRunnable):
    """
    Helper class that wraps object picking execution in a QThread.
    """

    def __init__(self, canvas, robot, obj_query, grasp_pose):
        """
        Creates an object picking execution thread.

        :param canvas: A world canvas object linked to this thread.
        :type canvas: :class:`pyrobosim.gui.world_canvas.WorldCanvas`
        :param robot: Robot instance or name to execute action.
        :type robot: :class:`pyrobosim.core.robot.Robot` or str
        :param obj_query: The object query (name, category, etc.).
        :type obj_query: str
        :param grasp_pose: A pose describing how to manipulate the object.
        :type grasp_pose: :class:`pyrobosim.utils.pose.Pose`, optional
        """
        super(PickRunner, self).__init__()
        self.canvas = canvas
        self.robot = robot
        self.obj_query = obj_query
        self.grasp_pose = grasp_pose

    def run(self):
        """Runs the object picking execution thread."""
        robot = self.robot
        world = self.canvas.world

        if isinstance(robot, str):
            robot = world.get_robot_by_name(robot)
        if robot is None:
            self.world.logger.warning("Robot is not specified. Cannot pick objects.")
            return

        robot.pick_object(self.obj_query, self.grasp_pose)


class PlaceRunner(QRunnable):
    """
    Helper class that wraps object placement execution in a QThread.
    """

    def __init__(self, canvas, robot, pose):
        """
        Creates an object placement execution thread.

        :param canvas: A world canvas object linked to this thread.
        :type canvas: :class:`pyrobosim.gui.world_canvas.WorldCanvas`
        :param robot: Robot instance or name to execute action.
        :type robot: :class:`pyrobosim.core.robot.Robot` or str
        :param pose: Optional pose describing how to place the object.
        :type pose: :class:`pyrobosim.utils.pose.Pose`, optional
        """
        super(PlaceRunner, self).__init__()
        self.canvas = canvas
        self.robot = robot
        self.pose = pose

    def run(self):
        """Runs the object picking execution thread."""
        robot = self.robot
        world = self.canvas.world

        if isinstance(robot, str):
            robot = world.get_robot_by_name(robot)
        if robot is None:
            self.world.logger.warning("Robot is not specified. Cannot place objects.")
            return

        robot.place_object(pose=self.pose)


class DetectRunner(QRunnable):
    """
    Helper class that wraps object detection execution in a QThread.
    """

    def __init__(self, canvas, robot, query):
        """
        Creates an object detection execution thread.

        :param canvas: A world canvas object linked to this thread.
        :type canvas: :class:`pyrobosim.gui.world_canvas.WorldCanvas`
        :param robot: Robot instance or name to execute action.
        :type robot: :class:`pyrobosim.core.robot.Robot` or str
        :param query: Query for object detection.
        :type query: str, optional
        """
        super(DetectRunner, self).__init__()
        self.canvas = canvas
        self.robot = robot
        self.query = query

    def run(self):
        """Runs the object detection execution thread."""
        robot = self.robot
        world = self.canvas.world

        if isinstance(robot, str):
            robot = world.get_robot_by_name(robot)
        if robot is None:
            self.world.logger.warning("Robot is not specified. Cannot detect objects.")
            return

        robot.detect_objects(self.query)


class OpenRunner(QRunnable):
    """
    Helper class that wraps location opening execution in a QThread.
    """

    def __init__(self, canvas, robot):
        """
        Creates a location opening execution thread.

        :param canvas: A world canvas object linked to this thread.
        :type canvas: :class:`pyrobosim.gui.world_canvas.WorldCanvas`
        :param robot: Robot instance or name to execute action.
        :type robot: :class:`pyrobosim.core.robot.Robot` or str
        """
        super(OpenRunner, self).__init__()
        self.canvas = canvas
        self.robot = robot

    def run(self):
        """Runs the location opening execution thread."""
        robot = self.robot
        world = self.canvas.world

        if isinstance(robot, str):
            robot = world.get_robot_by_name(robot)
        if robot is None:
            self.world.logger.warning("Robot is not specified. Cannot open locations.")
            return

        robot.open_location()


class CloseRunner(QRunnable):
    """
    Helper class that wraps location closing execution in a QThread.
    """

    def __init__(self, canvas, robot):
        """
        Creates a location closing execution thread.

        :param canvas: A world canvas object linked to this thread.
        :type canvas: :class:`pyrobosim.gui.world_canvas.WorldCanvas`
        :param robot: Robot instance or name to execute action.
        :type robot: :class:`pyrobosim.core.robot.Robot` or str
        """
        super(CloseRunner, self).__init__()
        self.canvas = canvas
        self.robot = robot

    def run(self):
        """Runs the location closing execution thread."""
        robot = self.robot
        world = self.canvas.world

        if isinstance(robot, str):
            robot = world.get_robot_by_name(robot)
        if robot is None:
            self.world.logger.warning("Robot is not specified. Cannot close locations.")
            return

        robot.close_location()
