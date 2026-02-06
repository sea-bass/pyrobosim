"""Local PyTrees integration for PyRoboSim (no ROS)."""

from __future__ import annotations

import threading
import time
from typing import Any, Callable

import py_trees

from pyrobosim.planning.actions import ExecutionResult, TaskAction
from pyrobosim.utils.path import Path
from pyrobosim.utils.pose import Pose


class RobotActionBehavior(py_trees.behaviour.Behaviour):
    """Non-blocking wrapper around a PyRoboSim TaskAction."""

    def __init__(
        self,
        name: str,
        robot: Any,
        action_type: str,
        params: dict[str, Any] | None = None,
        outputs: dict[str, str] | None = None,
        realtime_factor: float = 1.0,
    ) -> None:
        super().__init__(name)
        self.robot = robot
        self.action_type = action_type
        self.params = params or {}
        self.outputs = outputs or {}
        self.realtime_factor = realtime_factor

        self.blackboard = py_trees.blackboard.Blackboard()
        self._bb_client = py_trees.blackboard.Client(name=f"{name}_client")
        for key in _extract_blackboard_keys(self.params):
            self._bb_client.register_key(key=key, access=py_trees.common.Access.READ)
        for key in self.outputs.values():
            self._bb_client.register_key(key=key, access=py_trees.common.Access.WRITE)

        self._thread: threading.Thread | None = None
        self._result: ExecutionResult | None = None
        self._lock = threading.Lock()

    def initialise(self) -> None:
        resolved_params = {
            key: _resolve_param(value, self.blackboard) for key, value in self.params.items()
        }
        action = _build_task_action(self.action_type, resolved_params)

        def _run_action() -> None:
            result = self.robot.execute_action(action, realtime_factor=self.realtime_factor)
            with self._lock:
                self._result = result

        with self._lock:
            self._result = None
        self._thread = threading.Thread(target=_run_action, daemon=True)
        self._thread.start()

    def update(self) -> py_trees.common.Status:
        if self.robot.is_busy():
            return py_trees.common.Status.RUNNING

        with self._lock:
            result = self._result

        if result is None:
            return py_trees.common.Status.RUNNING

        self._write_outputs(result)
        return (
            py_trees.common.Status.SUCCESS
            if result.is_success()
            else py_trees.common.Status.FAILURE
        )

    def _write_outputs(self, result: ExecutionResult) -> None:
        for value_name, bb_key in self.outputs.items():
            if value_name == "status":
                setattr(self.blackboard, bb_key, result.status.name)
            elif value_name == "message":
                setattr(self.blackboard, bb_key, result.message)
            elif value_name == "result":
                setattr(self.blackboard, bb_key, result)
            elif value_name == "detected_objects":
                detected = [obj.name for obj in self.robot.last_detected_objects]
                setattr(self.blackboard, bb_key, detected)
            elif value_name == "battery_level":
                setattr(self.blackboard, bb_key, self.robot.battery_level)
            elif value_name == "last_nav_result":
                setattr(self.blackboard, bb_key, self.robot.last_nav_result)


class BlackboardCondition(py_trees.behaviour.Behaviour):
    """Simple blackboard condition node."""

    def __init__(
        self,
        name: str,
        key: str,
        operator: str,
        value: Any = None,
    ) -> None:
        super().__init__(name)
        self.key = key
        self.operator = operator
        self.value = value
        self.blackboard = py_trees.blackboard.Blackboard()
        self._bb_client = py_trees.blackboard.Client(name=f"{name}_condition")
        self._bb_client.register_key(key=key, access=py_trees.common.Access.READ)

    def update(self) -> py_trees.common.Status:
        if not hasattr(self.blackboard, self.key):
            return py_trees.common.Status.FAILURE
        current = getattr(self.blackboard, self.key)
        if _compare(self.operator, current, self.value):
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.FAILURE


SUPPORTED_COMPOSITES = {
    "sequence": py_trees.composites.Sequence,
    "selector": py_trees.composites.Selector,
    "parallel": py_trees.composites.Parallel,
}

SUPPORTED_DECORATORS = {
    "inverter": py_trees.decorators.Inverter,
    "retry": py_trees.decorators.Retry,
}


def build_tree_from_json(
    bt_json: dict[str, Any],
    robot: Any,
    realtime_factor: float = 1.0,
) -> py_trees.trees.BehaviourTree:
    _seed_blackboard(bt_json.get("blackboard", {}))
    root_spec = bt_json.get("root")
    if root_spec is None:
        raise ValueError("Behavior tree JSON is missing a root node.")

    root = _build_node(root_spec, robot, realtime_factor)
    return py_trees.trees.BehaviourTree(root=root)


def tick_tree(
    tree: py_trees.trees.BehaviourTree,
    period_ms: int = 100,
    cancel_event: threading.Event | None = None,
    on_tick: Callable[[py_trees.trees.BehaviourTree, int], None] | None = None,
) -> py_trees.common.Status:
    period_s = max(period_ms, 1) / 1000.0
    tick_count = 0
    while True:
        if cancel_event is not None and cancel_event.is_set():
            return py_trees.common.Status.FAILURE
        tree.tick()
        tick_count += 1
        if on_tick is not None:
            on_tick(tree, tick_count)
        if tree.root.status in (py_trees.common.Status.SUCCESS, py_trees.common.Status.FAILURE):
            return tree.root.status
        time.sleep(period_s)


def _seed_blackboard(blackboard_spec: dict[str, Any]) -> None:
    initial = blackboard_spec.get("initial", {}) if isinstance(blackboard_spec, dict) else {}
    blackboard = py_trees.blackboard.Blackboard()
    for key, value in initial.items():
        setattr(blackboard, key, value)


def _build_node(
    node_spec: dict[str, Any],
    robot: Any,
    realtime_factor: float,
) -> py_trees.behaviour.Behaviour:
    node_type = node_spec.get("type")
    name = node_spec.get("name", node_type or "node")

    if node_type in SUPPORTED_COMPOSITES:
        composite_cls = SUPPORTED_COMPOSITES[node_type]
        if node_type == "parallel":
            policy_name = node_spec.get("policy", "success_on_all")
            policy = _build_parallel_policy(policy_name)
            composite = composite_cls(name=name, policy=policy)
        else:
            memory = bool(node_spec.get("memory", False))
            composite = composite_cls(name=name, memory=memory)
        for child in node_spec.get("children", []):
            composite.add_child(_build_node(child, robot, realtime_factor))
        return composite

    if node_type == "decorator":
        decorator_name = node_spec.get("decorator")
        decorator_cls = SUPPORTED_DECORATORS.get(decorator_name)
        if decorator_cls is None:
            raise ValueError(f"Unsupported decorator: {decorator_name}")
        child_spec = node_spec.get("child")
        if child_spec is None:
            raise ValueError("Decorator node missing child.")
        if decorator_name == "retry":
            num_failures = int(node_spec.get("num_failures", 1))
            return decorator_cls(
                name=name,
                child=_build_node(child_spec, robot, realtime_factor),
                num_failures=num_failures,
            )
        return decorator_cls(name=name, child=_build_node(child_spec, robot, realtime_factor))

    if node_type == "condition":
        key = node_spec.get("key")
        operator = node_spec.get("operator", "==")
        value = node_spec.get("value")
        if not key:
            raise ValueError("Condition node missing key.")
        return BlackboardCondition(name=name, key=key, operator=operator, value=value)

    if node_type == "action":
        action_type = node_spec.get("action")
        if not action_type:
            raise ValueError("Action node missing action type.")
        params = node_spec.get("params", {})
        outputs = node_spec.get("outputs", {})
        return RobotActionBehavior(
            name=name,
            robot=robot,
            action_type=action_type,
            params=params,
            outputs=outputs,
            realtime_factor=realtime_factor,
        )

    raise ValueError(f"Unsupported node type: {node_type}")


def _build_parallel_policy(policy_name: str) -> py_trees.common.ParallelPolicy:
    if policy_name == "success_on_one":
        return py_trees.common.ParallelPolicy.SuccessOnOne()
    if policy_name == "success_on_all":
        return py_trees.common.ParallelPolicy.SuccessOnAll()
    raise ValueError(f"Unsupported parallel policy: {policy_name}")


def _compare(operator: str, current: Any, value: Any) -> bool:
    if operator == "==":
        return current == value
    if operator == "!=":
        return current != value
    if operator == ">":
        return current > value
    if operator == ">=":
        return current >= value
    if operator == "<":
        return current < value
    if operator == "<=":
        return current <= value
    if operator == "in":
        return current in value
    if operator == "contains":
        return value in current
    if operator == "truthy":
        return bool(current)
    return False


def _extract_blackboard_keys(params: dict[str, Any]) -> set[str]:
    keys: set[str] = set()
    for value in params.values():
        if isinstance(value, dict):
            source = value.get("source") or value.get("type")
            if source == "blackboard":
                key = value.get("key") or value.get("from_blackboard")
                if key:
                    keys.add(key)
            if "from_blackboard" in value and isinstance(value["from_blackboard"], str):
                keys.add(value["from_blackboard"])
    return keys


def _resolve_param(value: Any, blackboard: py_trees.blackboard.Blackboard) -> Any:
    if isinstance(value, dict):
        if value.get("source") == "blackboard" or value.get("type") == "blackboard":
            key = value.get("key") or value.get("from_blackboard")
            default = value.get("default")
            return _resolve_blackboard_value(blackboard, key, default)
        if "from_blackboard" in value:
            return _resolve_blackboard_value(blackboard, value["from_blackboard"], value.get("default"))
        if "pose" in value:
            return _pose_from_dict(value["pose"])
        if "path" in value:
            return _path_from_list(value["path"])
    return value


def _resolve_blackboard_value(
    blackboard: py_trees.blackboard.Blackboard,
    key: str | None,
    default: Any,
) -> Any:
    if key is None:
        return default
    if hasattr(blackboard, key):
        return getattr(blackboard, key)
    return default


def _pose_from_dict(data: dict[str, Any]) -> Pose:
    return Pose(
        x=data.get("x", 0.0),
        y=data.get("y", 0.0),
        yaw=data.get("yaw", 0.0),
    )


def _path_from_list(points: list[dict[str, Any]]) -> Path:
    poses = [_pose_from_dict(p) for p in points]
    return Path(poses=poses)


def _build_task_action(action_type: str, params: dict[str, Any]) -> TaskAction:
    return TaskAction(
        type=action_type,
        robot=params.get("robot"),
        object=params.get("object"),
        room=params.get("room"),
        source_location=params.get("source_location"),
        target_location=params.get("target_location"),
        pose=params.get("pose"),
        path=params.get("path", Path()),
        cost=params.get("cost"),
    )
