from __future__ import annotations

import json
import time
import urllib.request
from typing import Any


def call_control(url: str, endpoint: str, args: dict[str, Any]) -> dict[str, Any]:
    request = urllib.request.Request(
        url.rstrip("/") + endpoint,
        data=json.dumps(args).encode("utf-8"),
        headers={"Content-Type": "application/json", "Accept": "application/json"},
        method="POST",
    )
    with urllib.request.urlopen(request) as response:
        data = response.read().decode("utf-8")
        return json.loads(data)


def run_bt_control(bt: dict[str, Any], control_url: str, robot: str, timeout_s: int) -> dict[str, Any]:
    start = time.time()
    result = call_control(control_url, "/run_bt", {"bt_json": bt, "robot": robot, "tick_ms": 1000})
    run_id = result.get("run_id")
    if not run_id:
        return {"exec_status": "ERROR", "error": result}

    tick_count = 0
    while True:
        status = call_control(control_url, "/bt_status", {"run_id": run_id})
        tick_count = int(status.get("tick_count", 0))
        state = status.get("status")

        if state in ("SUCCESS", "FAILURE", "CANCELED"):
            return {
                "exec_status": state,
                "runtime_ms": int((time.time() - start) * 1000),
                "tick_count": tick_count,
            }

        if time.time() - start > timeout_s:
            return {
                "exec_status": "TIMEOUT",
                "runtime_ms": int((time.time() - start) * 1000),
                "tick_count": tick_count,
            }

        time.sleep(0.2)


def get_world_state(control_url: str, robot: str) -> dict[str, Any]:
    return call_control(control_url, "/world_state", {"robot": robot})


def reset_world(control_url: str, deterministic: bool = False, seed: int = -1) -> dict[str, Any]:
    return call_control(control_url, "/reset_world", {"deterministic": deterministic, "seed": seed})


def reload_world(control_url: str) -> dict[str, Any]:
    return call_control(control_url, "/reload_world", {})
