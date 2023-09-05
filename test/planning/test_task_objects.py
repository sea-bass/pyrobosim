#!/usr/bin/env python3

"""Unit tests for task action and plan objects."""

import pytest

from pyrobosim.planning.actions import TaskAction, TaskPlan
from pyrobosim.utils.motion import Path
from pyrobosim.utils.pose import Pose


def test_task_action_default_args():
    """Create TaskAction object with default arguments and validate results."""
    action = TaskAction("pick")

    assert action.type == "pick"
    assert action.robot is None
    assert action.cost is None
    assert action.object is None
    assert action.room is None
    assert action.source_location is None
    assert action.target_location is None
    assert action.pose is None
    assert isinstance(action.path, Path)
    assert action.path.num_poses == 0


def test_task_action_nondefault_args():
    """Create TaskAction object with nondefault arguments and validate results."""
    test_poses = [
        Pose(x=0.0, y=0.0, yaw=0.0),
        Pose(x=0.5, y=1.0, yaw=1.5),
        Pose(x=1.0, y=2.0, yaw=3.0),
    ]

    action = TaskAction(
        "Pick",
        robot="robot0",
        object="apple0",
        room="kitchen",
        source_location="table0_tabletop",
        target_location="counter0_right",
        pose=test_poses[-1],
        path=Path(poses=test_poses),
        cost=42.0,
    )

    assert action.type == "pick"  # Should be converted to lower case
    assert action.robot == "robot0"
    assert action.cost == pytest.approx(42.0)
    assert action.object == "apple0"
    assert action.room == "kitchen"
    assert action.source_location == "table0_tabletop"
    assert action.target_location == "counter0_right"
    assert action.pose == test_poses[-1]
    assert action.path == Path(poses=test_poses)


def test_task_plan_default_args():
    """Create TaskPlan object with default arguments and validate results."""
    plan = TaskPlan()

    assert plan.robot is None
    assert plan.actions == []
    assert plan.total_cost == pytest.approx(0.0)
    assert plan.size() == 0


def test_task_plan_nondefault_args():
    """Create TaskAction object with nondefault arguments and validate results."""
    nav_path = Path(
        poses=[
            Pose(x=0.0, y=0.0, z=0.0, q=[1.0, 0.0, 0.0, 0.0]),
            Pose(x=1.0, y=0.0, z=0.0, q=[0.707, 0.0, 0.0, 0.707]),
            Pose(x=1.0, y=1.0, z=0.0, q=[0.0, 0.0, 0.0, 1.0]),
        ]
    )
    place_pose = Pose(x=0.8, y=1.0, z=0.5, q=[1.0, 0.0, 0.0, 0.0])

    actions = [
        TaskAction("pick", object="apple", source_location="table0", cost=0.5),
        TaskAction(
            "navigate",
            source_location="table0",
            target_location="desk0",
            path=nav_path,
            cost=0.75,
        ),
        TaskAction(
            "place", object="apple", target_location="desk0", pose=place_pose, cost=0.5
        ),
    ]
    plan = TaskPlan(robot="robot0", actions=actions)

    assert plan.robot == "robot0"
    assert len(plan.actions) == 3
    assert plan.size() == 3
    assert plan.actions[0].type == "pick"
    assert plan.actions[1].type == "navigate"
    assert plan.actions[2].type == "place"
    assert plan.total_cost == pytest.approx(1.75)
