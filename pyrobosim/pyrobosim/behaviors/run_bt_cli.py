"""CLI runner for BT JSON files with terminal visualization."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import py_trees

from pyrobosim.behaviors.local_bt import build_tree_from_json, tick_tree
from pyrobosim.core.yaml_utils import WorldYamlLoader
from pyrobosim.utils.general import get_data_folder


def load_bt_json(path: str) -> dict:
    return json.loads(Path(path).read_text(encoding="utf-8"))


def find_latest_bt_file(directory: Path) -> Path | None:
    if not directory.exists():
        return None
    candidates = sorted(directory.glob("*.json"), key=lambda p: p.stat().st_mtime, reverse=True)
    return candidates[0] if candidates else None


def load_world(world_file: str | None) -> object:
    loader = WorldYamlLoader()
    data_folder = get_data_folder()
    if world_file:
        return loader.from_file(data_folder / world_file)
    return loader.from_file(data_folder / "test_world.yaml")


def main() -> None:
    parser = argparse.ArgumentParser(description="Run a BT JSON file locally.")
    parser.add_argument("--bt-file", help="Path to BT JSON file.")
    parser.add_argument("--latest", action="store_true", help="Run latest generated BT JSON file.")
    parser.add_argument("--world-file", default=None, help="YAML world file in data folder.")
    parser.add_argument("--robot", default=None, help="Robot name (default: first robot).")
    parser.add_argument("--tick-ms", type=int, default=100, help="Tick period in ms.")
    parser.add_argument("--print-every", type=int, default=1, help="Print tree every N ticks.")
    args = parser.parse_args()

    bt_path = None
    if args.latest:
        bt_path = find_latest_bt_file(Path(__file__).resolve().parent / "generated")
        if bt_path is None:
            raise SystemExit("No generated BT JSON files found.")
    else:
        if not args.bt_file:
            raise SystemExit("Provide --bt-file or --latest.")
        bt_path = Path(args.bt_file)

    bt_json = load_bt_json(str(bt_path))
    world = load_world(args.world_file)
    if args.robot:
        robot = world.get_robot_by_name(args.robot)
        if robot is None:
            names = [r.name for r in world.robots]
            raise SystemExit(f"Robot '{args.robot}' not found. उपलब्ध: {names}")
    else:
        robot = world.robots[0]

    tree = build_tree_from_json(bt_json, robot)

    def on_tick(local_tree: py_trees.trees.BehaviourTree, count: int) -> None:
        if args.print_every <= 0:
            return
        if count % args.print_every != 0:
            return
        print(py_trees.display.unicode_tree(local_tree.root, show_status=True))

    tick_tree(tree, period_ms=args.tick_ms, on_tick=on_tick)


if __name__ == "__main__":
    main()
