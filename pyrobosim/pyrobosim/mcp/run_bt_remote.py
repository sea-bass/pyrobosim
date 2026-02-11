"""CLI tool to ask the sim server to run a BT file."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any

import time
import urllib.request


def load_bt_json(path: str) -> dict[str, Any]:
    return json.loads(Path(path).read_text(encoding="utf-8"))


def find_latest_bt_file(directory: Path) -> Path | None:
    if not directory.exists():
        return None
    candidates = sorted(directory.glob("*.json"), key=lambda p: p.stat().st_mtime, reverse=True)
    return candidates[0] if candidates else None


def call_control(url: str, endpoint: str, args: dict[str, Any]) -> dict[str, Any]:
    request = urllib.request.Request(
        url.rstrip("/") + endpoint,
        data=json.dumps(args).encode("utf-8"),
        headers={
            "Content-Type": "application/json",
            "Accept": "application/json",
        },
        method="POST",
    )
    with urllib.request.urlopen(request) as response:
        data = response.read().decode("utf-8")
        return json.loads(data)


def main() -> None:
    parser = argparse.ArgumentParser(description="Run a BT file via the sim server.")
    parser.add_argument("--bt-file", help="Path to BT JSON file.")
    parser.add_argument("--latest", action="store_true", help="Run latest generated BT JSON file.")
    parser.add_argument("--server", default="http://127.0.0.1:9001", help="Control server URL.")
    parser.add_argument("--robot", default=None, help="Robot name.")
    parser.add_argument("--tick-ms", type=int, default=100, help="Tick period in ms.")
    parser.add_argument("--print-every", type=int, default=1, help="Print tree every N ticks.")
    parser.add_argument(
        "--print-blackboard",
        action="store_true",
        help="Print blackboard outputs (e.g., detect_status) each tick.",
    )
    args = parser.parse_args()

    bt_path = None
    if args.latest:
        bt_path = find_latest_bt_file(Path(__file__).resolve().parent.parent / "behaviors" / "generated")
        if bt_path is None:
            raise SystemExit("No generated BT JSON files found.")
    else:
        if not args.bt_file:
            raise SystemExit("Provide --bt-file or --latest.")
        bt_path = Path(args.bt_file)

    bt_json = load_bt_json(str(bt_path))
    tool_args: dict[str, Any] = {
        "bt_json": bt_json,
        "robot": args.robot,
        "tick_ms": args.tick_ms,
    }

    result = call_control(args.server, "/run_bt", tool_args)
    if "run_id" not in result:
        print(json.dumps(result, indent=2))
        return

    run_id = result["run_id"]
    last_tick = -1
    last_bb = None
    while True:
        status = call_control(args.server, "/bt_status", {"run_id": run_id})
        tick_count = int(status.get("tick_count", 0))
        tree = status.get("tree", "")
        blackboard = status.get("blackboard", {})
        if args.print_every > 0 and tick_count != last_tick and tick_count % args.print_every == 0:
            if tree:
                print(tree)
            last_tick = tick_count
        if args.print_blackboard and blackboard != last_bb:
            print(json.dumps(blackboard, indent=2))
            last_bb = blackboard
        if status.get("status") in ("SUCCESS", "FAILURE", "CANCELED"):
            if tree:
                print(tree)
            break
        time.sleep(max(args.tick_ms, 1) / 1000.0)


if __name__ == "__main__":
    main()
