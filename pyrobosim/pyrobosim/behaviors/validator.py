"""BT validator for operator/skill/parameter/structure checks."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any


@dataclass
class ValidationIssue:
    code: str
    message: str
    path: str | None = None


def validate_bt(bt: dict[str, Any], skills: list[dict[str, Any]]) -> list[ValidationIssue]:
    issues: list[ValidationIssue] = []
    skill_map = {s.get("name"): s for s in skills if isinstance(s, dict)}

    root = bt.get("root")
    if root is None:
        return [ValidationIssue("missing_root", "BT missing root node")]

    _validate_node(root, skill_map, issues, path="root")
    _validate_detect_before_pick(root, issues)
    return issues


def _validate_node(
    node: dict[str, Any],
    skill_map: dict[str, dict[str, Any]],
    issues: list[ValidationIssue],
    path: str,
) -> None:
    node_type = node.get("type")
    if node_type not in {"sequence", "selector", "parallel", "decorator", "condition", "action"}:
        issues.append(ValidationIssue("invalid_node", f"Invalid node type '{node_type}'", path))
        return

    if node_type in {"sequence", "selector", "parallel"}:
        children = node.get("children") or []
        if not isinstance(children, list):
            issues.append(ValidationIssue("invalid_children", "Children must be a list", path))
            return
        for idx, child in enumerate(children):
            if isinstance(child, dict):
                _validate_node(child, skill_map, issues, path=f"{path}.children[{idx}]")
            else:
                issues.append(ValidationIssue("invalid_child", "Child must be an object", path))
        return

    if node_type == "decorator":
        child = node.get("child")
        if not isinstance(child, dict):
            issues.append(ValidationIssue("invalid_decorator", "Decorator child must be an object", path))
            return
        _validate_node(child, skill_map, issues, path=f"{path}.child")
        return

    if node_type == "condition":
        if not node.get("key"):
            issues.append(ValidationIssue("invalid_condition", "Condition missing key", path))
        if not node.get("operator"):
            issues.append(ValidationIssue("invalid_condition", "Condition missing operator", path))
        return

    if node_type == "action":
        action = node.get("action")
        if action not in skill_map:
            issues.append(ValidationIssue("invalid_skill", f"Unknown skill '{action}'", path))
            return
        schema = skill_map[action]
        _validate_params(node.get("params", {}), schema.get("params", {}), issues, path)
        return


def _validate_params(
    params: dict[str, Any],
    schema_params: dict[str, Any],
    issues: list[ValidationIssue],
    path: str,
) -> None:
    if not isinstance(params, dict):
        issues.append(ValidationIssue("invalid_params", "Params must be an object", path))
        return

    for key, spec in schema_params.items():
        required = bool(spec.get("required", False)) if isinstance(spec, dict) else False
        if required and key not in params:
            issues.append(ValidationIssue("missing_param", f"Missing required param '{key}'", path))

    for key in params.keys():
        if key not in schema_params:
            issues.append(ValidationIssue("unknown_param", f"Unknown param '{key}'", path))


def _validate_detect_before_pick(root: dict[str, Any], issues: list[ValidationIssue]) -> None:
    """Simple structural rule: if Pick appears in a Sequence, Detect must precede it in that Sequence."""

    def walk(node: dict[str, Any]) -> None:
        if node.get("type") == "sequence":
            seen_detect = False
            for child in node.get("children", []):
                if isinstance(child, dict):
                    if child.get("type") == "action" and child.get("action") == "detect":
                        seen_detect = True
                    if child.get("type") == "action" and child.get("action") == "pick" and not seen_detect:
                        issues.append(
                            ValidationIssue(
                                "detect_before_pick",
                                "Pick appears before Detect in a sequence",
                            )
                        )
                    walk(child)
        else:
            for key in ("children", "child"):
                child = node.get(key)
                if isinstance(child, list):
                    for c in child:
                        if isinstance(c, dict):
                            walk(c)
                elif isinstance(child, dict):
                    walk(child)

    walk(root)
