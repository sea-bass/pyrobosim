"""MCP server for BT generation only (no execution)."""

from __future__ import annotations

import argparse
import json
import os
import uuid
import urllib.request
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Any

from .bt_schema import get_bt_schema
from .skills import get_skill_schemas
from .world_entities import build_world_entities
from pyrobosim.behaviors.rootstocks import list_rootstocks
from pyrobosim.behaviors.validator import validate_bt as validate_bt_tree
from pyrobosim.core.yaml_utils import WorldYamlLoader
from pyrobosim.utils.general import get_data_folder
from .eval_logger import append_record

try:
    from fastmcp import FastMCP
except ImportError as exc:  # pragma: no cover - runtime guard
    raise ImportError(
        "FastMCP is required to run the MCP server. Install fastmcp and retry."
    ) from exc


REPO_ROOT = Path(__file__).resolve().parents[3]
GENERATED_DIR = Path(__file__).resolve().parent.parent / "behaviors" / "generated"
DEFAULT_EVAL_LOG = REPO_ROOT / "eval" / "submissions.jsonl"
DEFAULT_CONTROL_URL = os.getenv("PYROBOSIM_CONTROL_URL", "http://127.0.0.1:9001")


@dataclass
class ServerValidationConfig:
    send_static_enforce: bool = True
    expose_validate_tool: bool = True
    check_vocabulary: bool = True
    expose_rootstocks: bool = False


def build_mcp(config: ServerValidationConfig | None = None) -> FastMCP:
    cfg = config or ServerValidationConfig()
    mcp = FastMCP("pyrobosim")

    @mcp.tool()
    def list_skills() -> list[dict[str, Any]]:
        return get_skill_schemas()

    @mcp.tool()
    def get_bt_format() -> dict[str, Any]:
        return get_bt_schema()

    @mcp.tool()
    def send_to_robot(
        bt_json: dict[str, Any] | str,
        output_name: str | None = None,
        task_prompt: str | None = None,
        token_usage: dict[str, Any] | None = None,
    ) -> dict[str, Any]:
        bt_data = _coerce_bt_json(bt_json)
        _validate_bt_json(bt_data)

        checked = _validate_impl(
            bt_data,
            check_vocabulary=cfg.check_vocabulary,
        )
        issues: list[dict[str, Any]] = checked["issues"]
        valid = checked["valid"]
        if cfg.send_static_enforce and not valid:
            raise ValueError(f"Validation failed: {issues}")

        GENERATED_DIR.mkdir(parents=True, exist_ok=True)
        name = _sanitize_name(output_name or f"bt_{uuid.uuid4().hex}")
        generated_path = GENERATED_DIR / f"{name}.json"
        generated_path.write_text(json.dumps(bt_data, indent=2), encoding="utf-8")

        submission_id = uuid.uuid4().hex
        generated_rel = _relative_to_repo(generated_path)
        record = {
            "submission_id": submission_id,
            "timestamp": datetime.utcnow().isoformat() + "Z",
            "generated": generated_rel,
            "task_prompt": task_prompt,
        }
        if token_usage:
            record["token_usage"] = token_usage
        append_record(DEFAULT_EVAL_LOG, record)

        return {
            "submission_id": submission_id,
            "generated": generated_rel,
            "logged": str(DEFAULT_EVAL_LOG),
            "static_validation": {
                "enabled": True,
                "valid": valid,
                "issues": issues,
            },
        }

    if cfg.expose_rootstocks:
        @mcp.tool()
        def list_rootstocks_tool() -> list[dict[str, Any]]:
            return list_rootstocks()

    @mcp.tool()
    def list_world_entities(
        mode: str = "vocab",
        world_file: str | None = None,
        control_url: str | None = None,
        robot: str | None = None,
        allow_full: bool = False,
    ) -> dict[str, Any]:
        """
        Returns world entity vocabulary without leaking task-critical state by default.

        Modes:
          - vocab: names/categories only (no locations or open/locked state)
          - observed: robot-known objects only (requires control_url)
          - full: includes ground-truth locations/state (requires allow_full=True)
        """
        return _resolve_world_entities(
            mode=mode,
            world_file=world_file,
            control_url=control_url,
            robot=robot,
            allow_full=allow_full,
        )

    def _validate_impl(
        bt_json: dict[str, Any] | str,
        check_vocabulary: bool = False,
        world_file: str | None = None,
        control_url: str | None = None,
        robot: str | None = None,
        allow_full: bool = False,
    ) -> dict[str, Any]:
        bt_data = _coerce_bt_json(bt_json)
        # Server-side static validation is schema/vocabulary only.
        # Behavior-policy constraints are evaluated in experiment metrics, not enforced here.
        issues = validate_bt_tree(
            bt_data,
            get_skill_schemas(),
        )
        vocab_issues: list[dict[str, Any]] = []
        if check_vocabulary:
            entities = _resolve_world_entities(
                mode="vocab",
                world_file=world_file,
                control_url=control_url,
                robot=robot,
                allow_full=allow_full,
            )
            vocab_issues = _validate_vocab_references(bt_data, entities)
        return {
            "valid": len(issues) == 0 and len(vocab_issues) == 0,
            "issues": [issue.__dict__ for issue in issues] + vocab_issues,
        }

    if cfg.expose_validate_tool:
        @mcp.tool()
        def validate_bt(bt_json: dict[str, Any] | str) -> dict[str, Any]:
            return _validate_impl(
                bt_json,
                check_vocabulary=cfg.check_vocabulary,
            )

    @mcp.resource("mcp://pyrobosim/skills")
    def skills_resource() -> dict[str, Any]:
        return {"skills": get_skill_schemas()}

    @mcp.resource("mcp://pyrobosim/bt_schema")
    def bt_schema_resource() -> dict[str, Any]:
        return get_bt_schema()

    if cfg.expose_rootstocks:
        @mcp.resource("mcp://pyrobosim/rootstocks")
        def rootstocks_resource() -> dict[str, Any]:
            return {"rootstocks": list_rootstocks()}

    return mcp


def _coerce_bt_json(bt_json: dict[str, Any] | str) -> dict[str, Any]:
    if isinstance(bt_json, dict):
        return bt_json
    if isinstance(bt_json, str):
        return json.loads(bt_json)
    raise TypeError("bt_json must be a dict or a JSON string")


def _validate_bt_json(bt_json: dict[str, Any]) -> None:
    if "root" not in bt_json:
        raise ValueError("Behavior tree JSON must contain a root node.")


def _sanitize_name(name: str) -> str:
    return "".join(ch for ch in name if ch.isalnum() or ch in ("_", "-")) or "bt"


def _relative_to_repo(path: Path) -> str:
    try:
        return str(path.relative_to(REPO_ROOT))
    except ValueError:
        return str(path)

def _resolve_world_file(world_file: str) -> Path:
    world_path = Path(world_file)
    if world_path.is_absolute():
        return world_path
    return get_data_folder() / world_file

def _call_control(url: str, endpoint: str, args: dict[str, Any]) -> dict[str, Any]:
    request = urllib.request.Request(
        url.rstrip("/") + endpoint,
        data=json.dumps(args).encode("utf-8"),
        headers={"Content-Type": "application/json", "Accept": "application/json"},
        method="POST",
    )
    with urllib.request.urlopen(request) as response:
        return json.loads(response.read().decode("utf-8"))


def _resolve_world_entities(
    mode: str,
    world_file: str | None,
    control_url: str | None,
    robot: str | None,
    allow_full: bool,
) -> dict[str, Any]:
    if mode == "full" and not allow_full:
        raise ValueError("full mode requires allow_full=True")

    if not control_url and not world_file:
        control_url = DEFAULT_CONTROL_URL

    if control_url:
        payload = {
            "mode": mode,
            "robot": robot,
            "allow_full": allow_full,
        }
        return _call_control(control_url, "/world_entities", payload)

    if mode == "observed":
        raise ValueError("observed mode requires control_url")

    if world_file is None:
        raise ValueError("world_file or control_url must be provided")

    world_path = _resolve_world_file(world_file)
    world = WorldYamlLoader().from_file(world_path)
    return build_world_entities(world, mode=mode, allow_full=allow_full)


def _iter_nodes(node: dict[str, Any], path: str = "root"):
    yield node, path
    for key in ("children", "child"):
        child = node.get(key)
        if isinstance(child, list):
            for idx, c in enumerate(child):
                if isinstance(c, dict):
                    yield from _iter_nodes(c, f"{path}.children[{idx}]")
        elif isinstance(child, dict):
            yield from _iter_nodes(child, f"{path}.child")


def _is_blackboard_param(value: Any) -> bool:
    return isinstance(value, dict) and (
        value.get("source") == "blackboard"
        or value.get("type") == "blackboard"
        or "from_blackboard" in value
    )


def _validate_vocab_references(bt: dict[str, Any], entities: dict[str, Any]) -> list[dict[str, Any]]:
    issues: list[dict[str, Any]] = []
    root = bt.get("root")
    if not isinstance(root, dict):
        return issues

    rooms = set(entities.get("rooms", []))
    location_names = {loc.get("name") for loc in entities.get("locations", []) if isinstance(loc, dict)}
    location_categories = {
        loc.get("category") for loc in entities.get("locations", []) if isinstance(loc, dict) and loc.get("category")
    }
    spawn_names = {sp.get("name") for sp in entities.get("object_spawns", []) if isinstance(sp, dict)}
    hallway_names = {h.get("name") for h in entities.get("hallways", []) if isinstance(h, dict)}
    object_names = {obj.get("name") for obj in entities.get("objects", []) if isinstance(obj, dict)}
    object_categories = set(entities.get("object_categories", []))

    valid_locations = {x for x in rooms | location_names | location_categories | spawn_names | hallway_names if x}
    valid_objects = {x for x in object_names | object_categories if x}

    for node, path in _iter_nodes(root):
        if node.get("type") != "action":
            continue
        action = node.get("action")
        params = node.get("params", {})
        if not isinstance(params, dict):
            continue

        if action in {"navigate", "open", "close"} and "target_location" in params:
            value = params.get("target_location")
            if isinstance(value, str) and value not in valid_locations:
                issues.append(
                    {
                        "code": "unknown_location_vocab",
                        "message": f"Unknown location vocabulary '{value}'",
                        "path": path,
                    }
                )
        if action in {"pick", "detect"} and "object" in params:
            value = params.get("object")
            if isinstance(value, str) and value and value not in valid_objects:
                issues.append(
                    {
                        "code": "unknown_object_vocab",
                        "message": f"Unknown object vocabulary '{value}'",
                        "path": path,
                    }
                )
            if _is_blackboard_param(value):
                # Runtime-resolved values cannot be validated statically.
                continue

    return issues


def main() -> None:
    parser = argparse.ArgumentParser(description="PyRoboSim MCP server (BT generation only).")
    parser.add_argument("--transport", default="streamable-http", help="MCP transport.")
    parser.add_argument("--host", default="0.0.0.0", help="MCP host.")
    parser.add_argument("--port", type=int, default=9000, help="MCP port.")
    parser.add_argument(
        "--disable-validate-tool",
        action="store_true",
        help="Hide validate_bt tool from clients.",
    )
    parser.add_argument(
        "--disable-vocabulary-check",
        action="store_true",
        help="Disable vocabulary checks in static validation.",
    )
    parser.add_argument(
        "--no-send-static-enforce",
        action="store_true",
        help="Run send_to_robot static validation but do not reject invalid BTs.",
    )
    parser.add_argument(
        "--enable-rootstocks",
        action="store_true",
        help="Expose rootstocks tool/resource to clients.",
    )
    args = parser.parse_args()

    config = ServerValidationConfig(
        send_static_enforce=not args.no_send_static_enforce,
        expose_validate_tool=not args.disable_validate_tool,
        check_vocabulary=not args.disable_vocabulary_check,
        expose_rootstocks=args.enable_rootstocks,
    )
    mcp = build_mcp(config)
    mcp.run(transport=args.transport, host=args.host, port=args.port)


if __name__ == "__main__":
    main()
