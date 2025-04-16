"""
Utilities to convert between standalone PyRoboSim objects and
ROS representations (messages, services, etc.).
"""

from geometry_msgs.msg import Pose as RosPose
from rclpy.time import Duration

from pyrobosim_msgs.msg import (  # type: ignore[attr-defined]
    ExecutionResult as RosExecutionResult,
    GoalSpecification,
    Path as RosPath,
    TaskAction as RosTaskAction,
    TaskPlan as RosTaskPlan,
)
from pyrobosim.core.types import Entity
from pyrobosim.core.world import World
from pyrobosim.planning.actions import (
    ExecutionResult,
    ExecutionStatus,
    TaskAction,
    TaskPlan,
)
from pyrobosim.utils.path import Path
from pyrobosim.utils.pose import Pose


def pose_from_ros(msg: RosPose) -> Pose:
    """
    Converts ROS pose message to a PyRoboSim pose.

    :param msg: ROS message.
    :return: Pose object.
    """
    return Pose.from_list(
        [
            msg.position.x,
            msg.position.y,
            msg.position.z,
            msg.orientation.w,
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
        ]
    )


def pose_to_ros(pose: Pose | None) -> RosPose:
    """
    Converts a PyRoboSim Pose to a ROS message.

    :param act: Pose object.
    :return: ROS message.
    """
    pose_msg = RosPose()
    if pose is not None:
        pose_msg.position.x = pose.x
        pose_msg.position.y = pose.y
        pose_msg.position.z = pose.z
        pose_msg.orientation.w = pose.q[0]
        pose_msg.orientation.x = pose.q[1]
        pose_msg.orientation.y = pose.q[2]
        pose_msg.orientation.z = pose.q[3]
    return pose_msg


def path_from_ros(msg: RosPath) -> Path:
    """
    Converts ROS path message to a PyRoboSim motion Path.

    :param msg: ROS message.
    :return: Path object.
    """
    return Path(poses=[pose_from_ros(p) for p in msg.poses])


def path_to_ros(path: Path) -> RosPath:
    """
    Converts a PyRoboSim motion Path to a ROS message.

    :param path: Path object.
    :return: ROS message.
    """
    return RosPath(
        poses=[pose_to_ros(p) for p in path.poses],
        length=path.length,
    )


def get_entity_name(entity: Entity | str | None) -> str:
    """
    Gets the name of an entity, or if a string is specified, gets the string itself.

    :param entity: Entity from which to extract the name
    :return: Entity name.
    """
    if entity is None:
        return ""
    elif isinstance(entity, str):
        return entity
    elif isinstance(entity, Entity):
        return entity.name  # type: ignore[no-any-return]  # Always a string

    raise TypeError(f"Invalid entity type: {type(entity)}")


def goal_specification_from_ros(
    msg: GoalSpecification, world: World
) -> list[tuple[str, ...]]:
    """
    Uses a world object to resolve a GoalSpecification message to a
    list of goal literals for task and motion planning.

    :param msg: ROS message.
    :param world: World object to use to resolve literals.
    :return: List of literals describing a goal specification.
    """
    goal_literals: list[tuple[str, ...]] = []
    for pred_msg in msg.predicates:
        pred = [pred_msg.type]

        # For each argument in the goal predicate, try resolve it to a real entity
        # in the world. If so, use the entity itself. Otherwise, use the name.
        for arg in pred_msg.args:
            entity = world.get_entity_by_name(arg)
            if entity:
                pred.append(entity)
            else:
                pred.append(arg)

        goal_literals.append(tuple(pred))

    return goal_literals


def task_action_from_ros(msg: RosTaskAction) -> TaskAction:
    """
    Converts a TaskAction ROS message to a TaskAction object.

    :param msg: ROS message.
    :return: Task action object.
    """
    if not isinstance(msg, RosTaskAction):
        raise Exception("Input is not a TaskAction ROS message")
    return TaskAction(
        msg.type,
        robot=msg.robot if msg.robot else None,
        object=msg.object if msg.object else None,
        room=msg.room if msg.room else None,
        source_location=msg.source_location if msg.source_location else None,
        target_location=msg.target_location if msg.target_location else None,
        pose=pose_from_ros(msg.pose) if msg.has_pose else None,
        path=path_from_ros(msg.path),
        cost=msg.cost,
    )


def task_action_to_ros(act: TaskAction) -> RosTaskAction:
    """
    Converts a TaskAction object to a TaskAction ROS message.

    :param act: Task action object.
    :return: ROS message.
    """
    if not isinstance(act, TaskAction):
        raise Exception("Input is not a TaskAction object")

    act_msg = RosTaskAction(type=act.type)
    act_msg.robot = get_entity_name(act.robot)
    act_msg.object = get_entity_name(act.object)
    act_msg.room = get_entity_name(act.room)
    act_msg.source_location = get_entity_name(act.source_location)
    act_msg.target_location = get_entity_name(act.target_location)
    if act.pose:
        act_msg.has_pose = True
        act_msg.pose = pose_to_ros(act.pose)
    act_msg.path = path_to_ros(act.path)
    if act.cost:
        act_msg.cost = float(act.cost)

    return act_msg


def task_plan_from_ros(msg: RosTaskPlan) -> TaskPlan:
    """
    Converts a TaskPlan ROS message to a TaskPlan object.

    :param msg: ROS message.
    :return: Task plan object.
    """
    if not isinstance(msg, RosTaskPlan):
        raise Exception("Input is not a TaskPlan ROS message")
    actions = [task_action_from_ros(act_msg) for act_msg in msg.actions]
    return TaskPlan(robot=msg.robot, actions=actions)


def task_plan_to_ros(plan: TaskPlan) -> RosTaskPlan:
    """
    Converts a TaskPlan object to a TaskPlan ROS message.

    :param plan: Task plan object.
    :return: ROS message.
    """
    if not isinstance(plan, TaskPlan):
        raise Exception("Input is not a TaskPlan object")
    act_msgs = [task_action_to_ros(act) for act in plan.actions]
    return RosTaskPlan(robot=plan.robot, actions=act_msgs, cost=plan.total_cost)


def ros_duration_to_float(ros_duration: Duration) -> float:
    """
    Converts an rclpy Duration object to a floating-point time value, in seconds.

    :param ros_time: rclpy Duration object.
    :return: The duration time, in seconds.
    """
    return float(1.0e-9 * ros_duration.nanoseconds)


def execution_result_to_ros(result: ExecutionResult) -> RosExecutionResult:
    """
    Converts an execution result object to its corresponding ROS message.

    :param result: The execution result object.
    :return: The equivalent ROS message.
    """
    return RosExecutionResult(
        status=getattr(ExecutionStatus, result.status.name),
        message=result.message or "",
    )


def execution_result_from_ros(msg: RosExecutionResult) -> ExecutionResult:
    """
    Converts an execution result ROS message to its corresponding object.

    :param msg: The execution result ROS message.
    :return: The equivalent native PyRoboSim object.
    """
    return ExecutionResult(
        status=getattr(RosExecutionResult, msg.status.name),
        message=msg.message or None,
    )
