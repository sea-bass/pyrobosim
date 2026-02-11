"""MCP server for BT generation only (no execution)."""

from __future__ import annotations

import argparse
import json
import os
import uuid
import urllib.request
from datetime import datetime
from pathlib import Path
from typing import Any

from .bt_schema import get_bt_schema
from .skills import get_skill_schemas
from .world_entities import build_world_entities
from pyrobosim.behaviors.rootstocks import list_rootstocks
from pyrobosim.behaviors.validator import validate_bt
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


def build_mcp() -> FastMCP:
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
    ) -> dict[str, Any]:
        bt_data = _coerce_bt_json(bt_json)
        _validate_bt_json(bt_data)

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
        append_record(DEFAULT_EVAL_LOG, record)

        return {
            "submission_id": submission_id,
            "generated": generated_rel,
            "logged": str(DEFAULT_EVAL_LOG),
        }

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

    @mcp.tool()
    def validate(bt_json: dict[str, Any] | str) -> dict[str, Any]:
        bt_data = _coerce_bt_json(bt_json)
        issues = validate_bt(bt_data, get_skill_schemas())
        return {
            "valid": len(issues) == 0,
            "issues": [issue.__dict__ for issue in issues],
        }

    @mcp.resource("mcp://pyrobosim/skills")
    def skills_resource() -> dict[str, Any]:
        return {"skills": get_skill_schemas()}

    @mcp.resource("mcp://pyrobosim/bt_schema")
    def bt_schema_resource() -> dict[str, Any]:
        return get_bt_schema()

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


def main() -> None:
    parser = argparse.ArgumentParser(description="PyRoboSim MCP server (BT generation only).")
    parser.add_argument("--transport", default="streamable-http", help="MCP transport.")
    parser.add_argument("--host", default="0.0.0.0", help="MCP host.")
    parser.add_argument("--port", type=int, default=9000, help="MCP port.")
    args = parser.parse_args()

    mcp = build_mcp()
    mcp.run(transport=args.transport, host=args.host, port=args.port)


if __name__ == "__main__":
    main()
