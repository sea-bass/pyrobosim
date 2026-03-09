from __future__ import annotations

from typing import Any

from pyrobosim.mcp.bt_schema import get_bt_schema


def iter_actions(node: dict[str, Any]) -> list[dict[str, Any]]:
    actions: list[dict[str, Any]] = []
    node_type = node.get("type")
    if node_type == "action":
        actions.append(node)
    for key in ("children", "child"):
        child = node.get(key)
        if isinstance(child, list):
            for c in child:
                if isinstance(c, dict):
                    actions.extend(iter_actions(c))
        elif isinstance(child, dict):
            actions.extend(iter_actions(child))
    return actions


def iter_nodes(node: dict[str, Any]) -> list[dict[str, Any]]:
    nodes = [node]
    for key in ("children", "child"):
        child = node.get(key)
        if isinstance(child, list):
            for c in child:
                if isinstance(c, dict):
                    nodes.extend(iter_nodes(c))
        elif isinstance(child, dict):
            nodes.extend(iter_nodes(child))
    return nodes


def has_memory_sequence(root: dict[str, Any]) -> bool:
    for node in iter_nodes(root):
        if node.get("type") == "sequence" and node.get("memory") is True:
            return True
    return False


def has_selector(root: dict[str, Any]) -> bool:
    return any(node.get("type") == "selector" for node in iter_nodes(root))


def has_condition_contains(root: dict[str, Any], value: str) -> bool:
    for node in iter_nodes(root):
        if node.get("type") == "condition" and node.get("operator") == "contains":
            if node.get("value") == value:
                return True
    return False


def extract_locations(actions: list[dict[str, Any]]) -> set[str]:
    locations: set[str] = set()
    for act in actions:
        params = act.get("params", {})
        if isinstance(params, dict):
            target = params.get("target_location")
            if isinstance(target, str):
                locations.add(target)
    return locations


def extract_action_types(actions: list[dict[str, Any]]) -> set[str]:
    return {act.get("action") for act in actions if act.get("action")}


def score_task(task: dict[str, Any], bt: dict[str, Any]) -> dict[str, Any]:
    root = bt.get("root") or {}
    actions = iter_actions(root)
    action_types = extract_action_types(actions)
    locations = extract_locations(actions)

    required = task.get("required", {})
    required_static = required.get("static", required)
    required_locations = set(required_static.get("locations", []))
    required_actions = set(required_static.get("actions", []))

    metrics: dict[str, Any] = {}
    metrics["actions_ok"] = required_actions.issubset(action_types)
    metrics["locations_ok"] = required_locations.issubset(locations)

    if required_static.get("must_use_memory_sequence"):
        metrics["memory_sequence_ok"] = has_memory_sequence(root)
    if required_static.get("must_use_selector"):
        metrics["selector_ok"] = has_selector(root)
    if required_static.get("condition_contains"):
        metrics["condition_contains_ok"] = has_condition_contains(root, required_static["condition_contains"])

    return metrics


def _load_node_types() -> set[str]:
    schema = get_bt_schema()
    types = schema.get("node_types")
    if isinstance(types, list):
        configured = {str(t) for t in types if isinstance(t, str) and t}
        if configured:
            return configured
    return {"sequence", "selector", "parallel", "decorator", "condition", "action"}


def _is_blackboard_ref(value: Any) -> bool:
    return isinstance(value, dict) and (
        value.get("source") == "blackboard"
        or value.get("type") == "blackboard"
        or "from_blackboard" in value
    )


def _type_ok(value: Any, expected: str) -> bool:
    if _is_blackboard_ref(value):
        return True
    if expected == "string":
        return isinstance(value, str)
    if expected == "bool":
        return isinstance(value, bool)
    if expected == "int":
        return isinstance(value, int) and not isinstance(value, bool)
    if expected in {"float", "number"}:
        return (isinstance(value, int) and not isinstance(value, bool)) or isinstance(value, float)
    if expected == "pose":
        return isinstance(value, dict) and "pose" in value
    if expected == "path":
        return isinstance(value, dict) and "path" in value
    return True


def evaluate_static_metrics(bt: dict[str, Any], skills: list[dict[str, Any]], task: dict[str, Any]) -> dict[str, Any]:
    issues: dict[str, list[str]] = {
        "valid_issues": [],
        "grounded_issues": [],
        "params_issues": [],
        "struct_issues": [],
    }

    root = bt.get("root")
    node_types = _load_node_types()
    if not isinstance(root, dict):
        issues["valid_issues"].append("missing_root")
        return {
            "valid": False,
            "grounded": False,
            "params_valid": False,
            "struct_comp": False,
            **issues,
        }

    nodes = iter_nodes(root)
    actions = [n for n in nodes if n.get("type") == "action"]
    skill_map = {s.get("name"): s for s in skills if isinstance(s, dict) and s.get("name")}

    for n in nodes:
        node_type = n.get("type")
        if node_type not in node_types:
            issues["valid_issues"].append(f"invalid_node:{node_type}")

    for a in actions:
        action_name = a.get("action")
        if action_name not in skill_map:
            issues["grounded_issues"].append(f"unknown_skill:{action_name}")

    for a in actions:
        action_name = a.get("action")
        if action_name not in skill_map:
            continue
        params = a.get("params", {})
        if not isinstance(params, dict):
            issues["params_issues"].append(f"invalid_params:{action_name}")
            continue
        schema_params = (skill_map[action_name].get("params", {}) if isinstance(skill_map[action_name], dict) else {})
        if not isinstance(schema_params, dict):
            schema_params = {}
        for key, spec in schema_params.items():
            if not isinstance(spec, dict):
                continue
            if bool(spec.get("required", False)) and key not in params:
                issues["params_issues"].append(f"missing_param:{action_name}:{key}")
            if key in params:
                expected = spec.get("type")
                if isinstance(expected, str) and not _type_ok(params[key], expected):
                    issues["params_issues"].append(f"invalid_type:{action_name}:{key}:{expected}")
                enum = spec.get("enum")
                if isinstance(enum, list) and params[key] not in enum and not _is_blackboard_ref(params[key]):
                    issues["params_issues"].append(f"invalid_enum:{action_name}:{key}")
        for key in params.keys():
            if key not in schema_params:
                issues["params_issues"].append(f"unknown_param:{action_name}:{key}")

    struct = score_task(task, bt)
    for k, v in struct.items():
        if not v:
            issues["struct_issues"].append(k)

    return {
        "valid": len(issues["valid_issues"]) == 0,
        "grounded": len(issues["grounded_issues"]) == 0,
        "params_valid": len(issues["params_issues"]) == 0,
        "struct_comp": len(issues["struct_issues"]) == 0,
        **issues,
        "struct_checks": struct,
    }
