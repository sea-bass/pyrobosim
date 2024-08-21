#!/usr/bin/env python3

"""Unit tests for task action and plan objects."""

import pytest

from pyrobosim.planning.actions import (
    ExecutionResult,
    ExecutionStatus,
    TaskAction,
    TaskPlan,
)
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


def test_print_task_action(capsys):
    """Create various TaskAction objects to test printing capabilities."""
    print(TaskAction("navigate"))
    out, _ = capsys.readouterr()
    assert out == "Navigate\n"

    print(
        TaskAction(
            "navigate",
            source_location="start",
            target_location="goal",
            pose=Pose(x=1.0, y=2.0, z=3.0),
            path=Path(poses=[Pose(), Pose(x=3.0, y=4.0)]),
            robot="robby",
            cost=42.0,
        )
    )
    out, _ = capsys.readouterr()
    expected_str = (
        "[robby] Navigate from start to goal\n"
        + "  At Pose: [x=1.00, y=2.00, z=3.00, qw=1.000, qx=0.000, qy=-0.000, qz=0.000]\n"
        + "  Path with 2 points, Length 5.000\n"
        + "  Cost: 42.000\n"
    )
    assert out == expected_str

    print(TaskAction("pick"))
    out, _ = capsys.readouterr()
    assert out == "Pick object\n"

    print(
        TaskAction(
            "pick",
            object="apple",
            target_location="table",
            pose=Pose(x=1.0, y=2.0, z=3.0),
        )
    )
    out, _ = capsys.readouterr()
    expected_str = (
        "Pick apple from table\n"
        + "  At Pose: [x=1.00, y=2.00, z=3.00, qw=1.000, qx=0.000, qy=-0.000, qz=0.000]\n"
    )
    assert out == expected_str

    print(TaskAction("place"))
    out, _ = capsys.readouterr()
    assert out == "Place object\n"

    print(
        TaskAction(
            "place",
            object="apple",
            target_location="table",
            pose=Pose(x=1.0, y=2.0, z=3.0),
        )
    )
    out, _ = capsys.readouterr()
    expected_str = (
        "Place apple at table\n"
        + "  At Pose: [x=1.00, y=2.00, z=3.00, qw=1.000, qx=0.000, qy=-0.000, qz=0.000]\n"
    )
    assert out == expected_str

    print(TaskAction("detect"))
    out, _ = capsys.readouterr()
    assert out == "Detect objects\n"

    print(TaskAction("detect", object="banana", target_location="table"))
    out, _ = capsys.readouterr()
    assert out == "Detect banana at table\n"

    print(TaskAction("open"))
    out, _ = capsys.readouterr()
    assert out == "Open current location\n"

    print(TaskAction("open", target_location="drawer"))
    out, _ = capsys.readouterr()
    assert out == "Open drawer\n"

    print(TaskAction("close"))
    out, _ = capsys.readouterr()
    assert out == "Close current location\n"

    print(TaskAction("close", target_location="door"))
    out, _ = capsys.readouterr()
    assert out == "Close door\n"

    print(TaskAction("spin"))
    out, _ = capsys.readouterr()
    assert out == "Invalid action type: spin\n"


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
        TaskAction("detect", object="apple", cost=0.125),
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
    assert len(plan.actions) == 4
    assert plan.size() == 4
    assert plan.actions[0].type == "detect"
    assert plan.actions[1].type == "pick"
    assert plan.actions[2].type == "navigate"
    assert plan.actions[3].type == "place"
    assert plan.total_cost == pytest.approx(1.875)


def test_print_task_plan(capsys):
    """Create various TaskPlan objects to test printing capabilities."""
    print(TaskPlan())
    out, _ = capsys.readouterr()
    assert out == "Empty plan\n"

    actions = [
        TaskAction(
            "navigate", source_location="charger", target_location="table", cost=1.0
        ),
        TaskAction("detect", object="apple"),
        TaskAction("pick", object="apple", target_location="table", cost=0.75),
    ]
    print(TaskPlan(actions=actions))
    out, _ = capsys.readouterr()
    expected_str = (
        "\n=== Task Plan: ===\n"
        + "1. Navigate from charger to table\n"
        + "  Cost: 1.000\n"
        + "2. Detect apple\n"
        + "3. Pick apple from table\n"
        + "  Cost: 0.750\n"
        + "=== Total Cost: 1.750 ===\n\n"
    )
    assert out == expected_str


def test_print_execution_result(capsys):
    """Create ExecutionResult object and test printing it."""
    print(ExecutionResult(status=ExecutionStatus.SUCCESS))
    out, _ = capsys.readouterr()
    assert out == "Execution result with status: SUCCESS\n"
