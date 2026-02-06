"""Standalone simulator + control server (no MCP)."""

from __future__ import annotations

import argparse
import json
import threading
import uuid
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Any

import py_trees

from pyrobosim.behaviors.local_bt import build_tree_from_json, tick_tree
from pyrobosim.core.world import World
from pyrobosim.core.yaml_utils import WorldYamlLoader
from pyrobosim.gui import start_gui
from pyrobosim.utils.general import get_data_folder


DATA_FOLDER = get_data_folder()


class SimContext:
    def __init__(self, world: World) -> None:
        self.world = world
        self.bt_runs: dict[str, dict[str, Any]] = {}
        self.lock = threading.Lock()

    def get_robot(self, name: str | None = None):
        if name:
            robot = self.world.get_robot_by_name(name)
            if robot is None:
                names = [r.name for r in self.world.robots]
                raise ValueError(f"Robot '{name}' not found. Available: {names}")
            return robot
        if not self.world.robots:
            raise ValueError("No robots in world")
        return self.world.robots[0]

    def start_bt(
        self,
        bt_json: dict[str, Any],
        robot_name: str | None,
        tick_ms: int,
        print_every: int = 1,
    ) -> str:
        robot = self.get_robot(robot_name)
        tree = build_tree_from_json(bt_json, robot)
        cancel_event = threading.Event()
        run_id = str(uuid.uuid4())

        def _on_tick(local_tree: py_trees.trees.BehaviourTree, count: int) -> None:
            tree_text = ""
            if print_every > 0 and count % print_every == 0:
                tree_text = py_trees.display.unicode_tree(local_tree.root, show_status=True)
            with self.lock:
                if run_id in self.bt_runs:
                    self.bt_runs[run_id]["tick_count"] = count
                    if tree_text:
                        self.bt_runs[run_id]["tree"] = tree_text

        def _runner() -> None:
            status = tick_tree(
                tree,
                period_ms=tick_ms,
                cancel_event=cancel_event,
                on_tick=_on_tick,
            )
            with self.lock:
                if run_id in self.bt_runs:
                    self.bt_runs[run_id]["status"] = status.name

        with self.lock:
            self.bt_runs[run_id] = {
                "status": "RUNNING",
                "cancel_event": cancel_event,
                "tick_count": 0,
                "tree": "",
            }
        thread = threading.Thread(target=_runner, daemon=True)
        thread.start()
        with self.lock:
            self.bt_runs[run_id]["thread"] = thread
        return run_id

    def bt_status(self, run_id: str) -> dict[str, Any]:
        with self.lock:
            entry = self.bt_runs.get(run_id)
            if not entry:
                return {"status": "UNKNOWN"}
            return {
                "status": entry.get("status", "UNKNOWN"),
                "tick_count": entry.get("tick_count", 0),
                "tree": entry.get("tree", ""),
            }


class ControlHandler(BaseHTTPRequestHandler):
    context: SimContext | None = None

    def _send_json(self, payload: dict[str, Any], status: int = 200) -> None:
        body = json.dumps(payload).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_POST(self) -> None:  # noqa: N802
        if self.path not in ("/run_bt", "/bt_status"):
            self._send_json({"error": "not_found"}, status=404)
            return

        length = int(self.headers.get("Content-Length", "0"))
        raw = self.rfile.read(length).decode("utf-8") if length > 0 else "{}"
        try:
            data = json.loads(raw)
        except json.JSONDecodeError:
            self._send_json({"error": "invalid_json"}, status=400)
            return

        if self.context is None:
            self._send_json({"error": "no_context"}, status=500)
            return

        if self.path == "/bt_status":
            run_id = data.get("run_id")
            if not run_id:
                self._send_json({"error": "run_id_required"}, status=400)
                return
            self._send_json(self.context.bt_status(run_id))
            return

        bt_json = data.get("bt_json")
        if not isinstance(bt_json, dict):
            self._send_json({"error": "bt_json_required"}, status=400)
            return

        robot = data.get("robot")
        tick_ms = int(data.get("tick_ms", 100))
        print_every = int(data.get("print_every", 1))
        try:
            run_id = self.context.start_bt(bt_json, robot, tick_ms, print_every=print_every)
        except Exception as exc:
            self._send_json({"error": str(exc)}, status=400)
            return
        self._send_json({"run_id": run_id})


def start_control_server(context: SimContext, host: str, port: int) -> ThreadingHTTPServer:
    ControlHandler.context = context
    server = ThreadingHTTPServer((host, port), ControlHandler)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    return server


def load_world(world_file: str | None, multirobot: bool) -> World:
    loader = WorldYamlLoader()
    if world_file:
        return loader.from_file(DATA_FOLDER / world_file)
    if multirobot:
        return loader.from_file(DATA_FOLDER / "test_world_multirobot.yaml")
    return loader.from_file(DATA_FOLDER / "test_world.yaml")


def main() -> None:
    parser = argparse.ArgumentParser(description="PyRoboSim simulator + control server.")
    parser.add_argument("--world-file", default=None, help="YAML world file in data folder.")
    parser.add_argument("--multirobot", action="store_true", help="Use multirobot test world.")
    parser.add_argument("--host", default="0.0.0.0", help="Control server host.")
    parser.add_argument("--control-port", type=int, default=9001, help="Control server port.")
    args = parser.parse_args()

    world = load_world(args.world_file, args.multirobot)
    context = SimContext(world)
    start_control_server(context, args.host, args.control_port)
    start_gui(world)


if __name__ == "__main__":
    main()
