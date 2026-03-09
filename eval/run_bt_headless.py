"""Headless single-run BT executor for evaluation."""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path
from typing import Any

import py_trees

from pyrobosim.behaviors.local_bt import build_tree_from_json
from pyrobosim.core.yaml_utils import WorldYamlLoader
from pyrobosim.utils.general import get_data_folder


def _color(text: str, code: str) -> str:
    return f"\033[{code}m{text}\033[0m"


def _headless_log(msg: str, level: str = "info") -> None:
    prefix = "    " + _color("[headless]", "36")
    if level == "warn":
        msg = _color(msg, "33")
    elif level == "error":
        msg = _color(msg, "31")
    elif level == "ok":
        msg = _color(msg, "32")
    print(f"{prefix} {msg}", file=sys.stderr, flush=True)


def load_bt(path: Path) -> dict[str, Any]:
    return json.loads(path.read_text(encoding="utf-8"))


def world_state(world, robot_name: str | None = None) -> dict[str, Any]:
    robot = world.get_robot_by_name(robot_name) if robot_name else world.robots[0]
    held = None
    if robot.manipulated_object is not None:
        held = {
            "name": robot.manipulated_object.name,
            "category": robot.manipulated_object.category,
        }
    objects = []
    for obj in world.objects:
        parent = obj.parent.name if obj.parent is not None else None
        objects.append({"name": obj.name, "category": obj.category, "location": parent})
    location = robot.location.name if robot.location is not None else None
    return {
        "robot": robot.name,
        "location": location,
        "held_object": held,
        "objects": objects,
    }


def run_bt(
    tree: py_trees.trees.BehaviourTree,
    timeout_s: int,
    tick_period_s: float = 0.1,
    progress_interval_s: float = 0.0,
) -> dict[str, Any]:
    start = time.time()
    last_tree = ""
    last_progress = start
    tick_count = 0

    while True:
        tree.tick()
        tick_count += 1
        tree_text = py_trees.display.unicode_tree(tree.root, show_status=True)
        if tree_text != last_tree:
            last_tree = tree_text

        status = tree.root.status
        now = time.time()
        if progress_interval_s > 0 and (now - last_progress) >= progress_interval_s:
            _headless_log(
                f"elapsed={now - start:.1f}s tick={tick_count} status={status.name}",
                level="info",
            )
            last_progress = now

        if status in (py_trees.common.Status.SUCCESS, py_trees.common.Status.FAILURE):
            _headless_log(
                f"finished status={status.name} ticks={tick_count} runtime={time.time() - start:.1f}s",
                level="ok" if status.name == "SUCCESS" else "warn",
            )
            return {
                "exec_status": status.name,
                "runtime_ms": int((time.time() - start) * 1000),
                "tick_count": tick_count,
            }

        if time.time() - start > timeout_s:
            _headless_log(
                f"timeout after {timeout_s}s (ticks={tick_count})",
                level="warn",
            )
            return {
                "exec_status": "TIMEOUT",
                "runtime_ms": int((time.time() - start) * 1000),
                "tick_count": tick_count,
            }

        time.sleep(max(0.0, tick_period_s))


def main() -> None:
    parser = argparse.ArgumentParser(description="Run a BT once in headless mode.")
    parser.add_argument("--bt-file", required=True, help="Path to BT JSON file.")
    parser.add_argument("--world-file", required=True, help="World YAML filename in data folder.")
    parser.add_argument("--robot", default=None, help="Robot name (default: first).")
    parser.add_argument("--timeout-s", type=int, default=60)
    parser.add_argument("--tick-period-s", type=float, default=0.1)
    parser.add_argument(
        "--progress-interval-s",
        type=float,
        default=0.0,
        help="If > 0, print progress heartbeat every N seconds.",
    )
    args = parser.parse_args()

    world = WorldYamlLoader().from_file(get_data_folder() / args.world_file)
    try:
        bt = load_bt(Path(args.bt_file))
        robot = world.get_robot_by_name(args.robot) if args.robot else world.robots[0]
        tree = build_tree_from_json(bt, robot)

        pre = world_state(world, robot.name)
        exec_result = run_bt(
            tree,
            args.timeout_s,
            tick_period_s=args.tick_period_s,
            progress_interval_s=args.progress_interval_s,
        )
        post = world_state(world, robot.name)

        output = {
            "pre_state": pre,
            "post_state": post,
            **exec_result,
        }
        print(json.dumps(output), flush=True)
    finally:
        # Ensure sensor/path threads are stopped so this child process exits cleanly.
        world.shutdown()


if __name__ == "__main__":
    main()
