"""Headless single-run BT executor for evaluation."""

from __future__ import annotations

import argparse
import json
import time
from pathlib import Path
from typing import Any

import py_trees

from pyrobosim.behaviors.local_bt import build_tree_from_json
from pyrobosim.core.yaml_utils import WorldYamlLoader
from pyrobosim.utils.general import get_data_folder


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


def run_bt(tree: py_trees.trees.BehaviourTree, timeout_s: int, stall_s: int) -> dict[str, Any]:
    start = time.time()
    last_tree = ""
    last_change = time.time()
    tick_count = 0

    while True:
        tree.tick()
        tick_count += 1
        tree_text = py_trees.display.unicode_tree(tree.root, show_status=True)
        if tree_text != last_tree:
            last_tree = tree_text
            last_change = time.time()

        status = tree.root.status
        if status in (py_trees.common.Status.SUCCESS, py_trees.common.Status.FAILURE):
            return {
                "exec_status": status.name,
                "runtime_ms": int((time.time() - start) * 1000),
                "tick_count": tick_count,
            }

        if time.time() - start > timeout_s:
            return {
                "exec_status": "TIMEOUT",
                "runtime_ms": int((time.time() - start) * 1000),
                "tick_count": tick_count,
            }

        if time.time() - last_change > stall_s:
            return {
                "exec_status": "STALL",
                "runtime_ms": int((time.time() - start) * 1000),
                "tick_count": tick_count,
            }

        time.sleep(0.1)


def main() -> None:
    parser = argparse.ArgumentParser(description="Run a BT once in headless mode.")
    parser.add_argument("--bt-file", required=True, help="Path to BT JSON file.")
    parser.add_argument("--world-file", required=True, help="World YAML filename in data folder.")
    parser.add_argument("--robot", default=None, help="Robot name (default: first).")
    parser.add_argument("--timeout-s", type=int, default=60)
    parser.add_argument("--stall-seconds", type=int, default=10)
    args = parser.parse_args()

    world = WorldYamlLoader().from_file(get_data_folder() / args.world_file)
    bt = load_bt(Path(args.bt_file))
    robot = world.get_robot_by_name(args.robot) if args.robot else world.robots[0]
    tree = build_tree_from_json(bt, robot)

    pre = world_state(world, robot.name)
    exec_result = run_bt(tree, args.timeout_s, args.stall_seconds)
    post = world_state(world, robot.name)

    output = {
        "pre_state": pre,
        "post_state": post,
        **exec_result,
    }
    print(json.dumps(output))


if __name__ == "__main__":
    main()
