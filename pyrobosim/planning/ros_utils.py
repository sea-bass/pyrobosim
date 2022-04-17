"""
Utilities to convert between standalone planning objects and 
ROS representations (messages, services, etc.).
"""

import pyrobosim.msg as ros_msgs
import pyrobosim.planning.actions as acts

def task_action_from_ros(msg):
    """ 
    Converts a TaskAction ROS message to a TaskAction object.
    
    :param msg: ROS message.
    :type msg: :class:`pyrobosim.msg.TaskAction`
    :return: Task action object.
    :rtype: :class:`pyrobosim.planning.actions.TaskAction`
    """
    if not isinstance(msg, ros_msgs.TaskAction):
        raise Exception("Input is not a TaskAction ROS message")
    return acts.TaskAction(msg.type, object=msg.object, room=msg.room,
                 source_location=msg.source_location, target_location=msg.target_location,
                 pose=msg.pose, cost=msg.cost)

def task_action_to_ros(act):
    """ 
    Converts a TaskAction object to a TaskAction ROS message.

    :param act: Task action object.
    :type act: :class:`pyrobosim.planning.actions.TaskAction`    
    :return: ROS message.
    :rtype: :class:`pyrobosim.msg.TaskAction`
    """
    if not isinstance(act, acts.TaskAction):
        raise Exception("Input is not a TaskAction object")
    return ros_msgs.TaskAction(type=act.type, object=act.object, room=act.room,
                               source_location=act.source_location, target_location=act.target_location,
                               pose=act.pose, cost=act.cost) 

def task_plan_from_ros(msg):
    """
    Converts a TaskPlan ROS message to a TaskPlan object.
    
    :param msg: ROS message.
    :type msg: :class:`pyrobosim.msg.TaskPlan`
    :return: Task plan object.
    :rtype: :class:`pyrobosim.planning.actions.TaskPlan`
    """
    if not isinstance(msg, ros_msgs.TaskPlan):
        raise Exception("Input is not a TaskPlan ROS message")
    actions = [task_action_from_ros(act_msg) for act_msg in msg.actions]
    return acts.TaskPlan(actions=actions)

def task_plan_to_ros(plan):
    """
    Converts a TaskPlan object to a TaskPlan ROS message.
    
    :param plan: Task plan object.
    :type plan: :class:`pyrobosim.planning.actions.TaskPlan`    
    :return: ROS message.
    :rtype: :class:`pyrobosim.msg.TaskPlan`
    """
    if not isinstance(plan, acts.TaskPlan):
        raise Exception("Input is not a TaskPlan object")
    act_msgs = [task_action_to_ros(act) for act in plan.actions]
    return ros_msgs.TaskPlan(actions=act_msgs, cost=plan.total_cost)
