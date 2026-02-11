"""Manual-mode evaluation harness for BT composition quality and runtime outcomes."""

from __future__ import annotations

import json
import time
from pathlib import Path
from typing import Any
from datetime import datetime

import urllib.request
import yaml

from pyrobosim.behaviors.validator import validate_bt
from pyrobosim.mcp.skills import get_skill_schemas
from pyrobosim.core.yaml_utils import WorldYamlLoader
from pyrobosim.utils.general import get_data_folder


def load_yaml(path: Path) -> dict[str, Any]:
    return yaml.safe_load(path.read_text(encoding="utf-8"))


def load_bt(path: Path) -> dict[str, Any]:
    return json.loads(path.read_text(encoding="utf-8"))


def load_submissions(path: Path) -> list[dict[str, Any]]:
    if not path.exists():
        return []
    records: list[dict[str, Any]] = []
    for line in path.read_text(encoding="utf-8").splitlines():
        if line.strip():
            records.append(json.loads(line))
    return records


def parse_timestamp(ts: str | None) -> float:
    if not ts:
        return 0.0
    try:
        return datetime.fromisoformat(ts.replace("Z", "+00:00")).timestamp()
    except Exception:
        return 0.0


def select_submission_by_prompt(task_prompt: str, submissions: list[dict[str, Any]]) -> dict[str, Any] | None:
    # import pdb;pdb.set_trace()
    matches = [s for s in submissions if s.get("task_prompt") == task_prompt]
    if not matches:
        return None
    matches.sort(key=lambda s: parse_timestamp(s.get("timestamp")), reverse=True)
    return matches[0]


def select_submission_by_id(submission_id: str, submissions: list[dict[str, Any]]) -> dict[str, Any] | None:
    matches = [s for s in submissions if s.get("submission_id") == submission_id]
    if not matches:
        return None
    return matches[0]


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
    required_actions = set(required.get("actions", []))
    required_locations = set(required.get("locations", []))

    metrics: dict[str, Any] = {}
    metrics["actions_ok"] = required_actions.issubset(action_types)
    metrics["locations_ok"] = required_locations.issubset(locations)

    if required.get("must_use_memory_sequence"):
        metrics["memory_sequence_ok"] = has_memory_sequence(root)
    if required.get("must_use_selector"):
        metrics["selector_ok"] = has_selector(root)
    if required.get("condition_contains"):
        metrics["condition_contains_ok"] = has_condition_contains(
            root, required["condition_contains"]
        )

    return metrics


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


def run_bt(bt: dict[str, Any], control_url: str, robot: str, timeout_s: int, stall_s: int) -> dict[str, Any]:
    start = time.time()
    result = call_control(control_url, "/run_bt", {"bt_json": bt, "robot": robot, "tick_ms": 100})
    run_id = result.get("run_id")
    if not run_id:
        return {"exec_status": "ERROR", "error": result}

    last_tree = ""
    last_change = time.time()
    tick_count = 0

    while True:
        status = call_control(control_url, "/bt_status", {"run_id": run_id})
        tick_count = int(status.get("tick_count", 0))
        tree = status.get("tree", "")
        state = status.get("status")

        if tree and tree != last_tree:
            last_tree = tree
            last_change = time.time()

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

        if time.time() - last_change > stall_s:
            return {
                "exec_status": "STALL",
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

def has_object(state: dict[str, Any], *, name: str | None = None, category: str | None = None) -> bool:
    for obj in state.get("objects", []):
        if name and obj.get("name") == name:
            return True
        if category and obj.get("category") == category:
            return True
    return False

def placed_category_at(state: dict[str, Any], category: str, location: str) -> bool:
    for obj in state.get("objects", []):
        if obj.get("category") == category and obj.get("location") == location:
            return True
    return False


def object_at(state: dict[str, Any], name: str, location: str) -> bool:
    for obj in state.get("objects", []):
        if obj.get("name") == name and obj.get("location") == location:
            return True
    return False


def location_exists(state: dict[str, Any], location: str) -> bool:
    if not location:
        return False
    if state.get("location") == location:
        return True
    for obj in state.get("objects", []):
        if obj.get("location") == location:
            return True
    return False

def evaluate_success(task: dict[str, Any], pre: dict[str, Any], post: dict[str, Any]) -> dict[str, Any]:
    success = task.get("success", {}) or {}
    result: dict[str, Any] = {}

    feasible = True
    if "must_hold_name" in success:
        feasible = feasible and has_object(pre, name=success["must_hold_name"])
    if "must_hold_category" in success:
        feasible = feasible and has_object(pre, category=success["must_hold_category"])
    if "must_place_category_at" in success:
        cat = success["must_place_category_at"].get("category")
        loc = success["must_place_category_at"].get("location")
        if cat:
            feasible = feasible and has_object(pre, category=cat)
        if loc is None:
            feasible = False
        if loc:
            feasible = feasible and location_exists(pre, loc)
    if "object_at" in success:
        name = success["object_at"].get("name")
        loc = success["object_at"].get("location")
        if name:
            feasible = feasible and has_object(pre, name=name)
        if loc is None:
            feasible = False
        if loc:
            feasible = feasible and location_exists(pre, loc)
    if "robot_at" in success:
        loc = success["robot_at"]
        feasible = feasible and location_exists(pre, loc)

    result["feasible"] = feasible
    if not feasible:
        result["goal_satisfied"] = False
        return result

    held = post.get("held_object") or {}
    checks: list[bool] = []
    if "must_hold_name" in success:
        checks.append(held.get("name") == success["must_hold_name"])
    if "must_hold_category" in success:
        checks.append(held.get("category") == success["must_hold_category"])
    if "must_place_category_at" in success:
        cat = success["must_place_category_at"].get("category")
        loc = success["must_place_category_at"].get("location")
        checks.append(bool(cat and loc and placed_category_at(post, cat, loc)))
    if "object_at" in success:
        name = success["object_at"].get("name")
        loc = success["object_at"].get("location")
        checks.append(bool(name and loc and object_at(post, name, loc)))
    if "robot_at" in success:
        checks.append(post.get("location") == success["robot_at"])
    if "not_holding" in success:
        checks.append(post.get("held_object") is None)

    result["goal_satisfied"] = all(checks) if checks else None
    return result


def evaluate_success_criteria(task: dict[str, Any]) -> dict[str, Any]:
    # Placeholder for world-state checks; currently we rely on exec_status
    return {}


def main() -> None:
    import argparse

    parser = argparse.ArgumentParser(description="Run evaluation harness.")
    parser.add_argument("--tasks-file", default="tasks.yaml", help="Tasks YAML file.")
    parser.add_argument("--task-id", help="Run a single task by id.")
    parser.add_argument("--step", action="store_true", help="Pause between tasks.")
    parser.add_argument("--submission-id", help="Evaluate a specific submission id.")
    args = parser.parse_args()

    root_dir = Path(__file__).resolve().parent
    repo_root = root_dir.parent
    config = load_yaml(root_dir / args.tasks_file)

    bt_dir = Path(repo_root / config.get("bt_output_dir", ""))
    submissions = load_submissions(root_dir / "submissions.jsonl")
    control_url = config.get("control_server", "http://127.0.0.1:9001")
    execution_mode = config.get("execution_mode", "control_server")
    runtime_cfg = config.get("runtime", {})
    timeout_s = int(runtime_cfg.get("timeout_s", 60))
    stall_s = int(runtime_cfg.get("stall_seconds", 10))
    reset_between = bool(runtime_cfg.get("reset_between", False))
    hard_reset_between = bool(runtime_cfg.get("hard_reset_between", False))

    skills = get_skill_schemas()

    results: list[dict[str, Any]] = []
    out_path = root_dir / "results.json"
    print(f"Loaded {len(config.get('tasks', []))} tasks")
    print(f"Submissions log: {root_dir / 'submissions.jsonl'}")
    print(f"BT output dir: {bt_dir}")
    print(f"Control server: {control_url}")
    print(f"Timeout: {timeout_s}s, Stall: {stall_s}s")
    tasks = config.get("tasks", [])
    if args.task_id:
        tasks = [t for t in tasks if t.get("id") == args.task_id]

    for task in tasks:
        task_id = task.get("id")
        print(f"\n=== Evaluating {task_id} ===")
        print(f"Task Prompt: {task.get('task_prompt', '')}")
        submission = None

        if args.submission_id:
            submission = select_submission_by_id(args.submission_id, submissions)
        else:
            submission = select_submission_by_prompt(task.get("task_prompt", ""), submissions)
        if submission and submission.get("generated"):
            generated_path = Path(submission["generated"])
            bt_path = generated_path if generated_path.is_absolute() else (repo_root / generated_path)
            print(f"Using submission: {bt_path}")
        elif bt_path.exists():
            print(f"Using task_id file: {bt_path}")
        else:
            print(f"Missing BT file: {bt_path}")

        if not bt_path.exists():
            print("Missing BT file.")
            results.append({"id": task_id, "missing_bt": True})
            continue

        bt = load_bt(bt_path)
        issues = validate_bt(bt, skills)
        if issues:
            print(f"Validation issues: {[i.code for i in issues]}")
        else:
            print("Validation OK")
        metrics = score_task(task, bt)
        print(f"Static metrics: {metrics}")

        row = {
            "id": task_id,
            "task_prompt": task.get("task_prompt", ""),
            "valid": len(issues) == 0,
            "issues": [i.code for i in issues],
            "bt_path": str(bt_path),
        }
        row.update(metrics)

        # Runtime execution
        robot_name = config.get("robot_name", "robot")
        if execution_mode == "headless_per_run":
            print("Running BT headless in a fresh process...")
            headless_cmd = [
                "python3",
                str((root_dir / "run_bt_headless.py").resolve()),
                "--bt-file",
                str(bt_path),
                "--world-file",
                config.get("world_file", "test_world.yaml"),
                "--robot",
                robot_name,
                "--timeout-s",
                str(timeout_s),
                "--stall-seconds",
                str(stall_s),
            ]
            import subprocess

            completed = subprocess.run(headless_cmd, capture_output=True, text=True)
            if completed.returncode != 0:
                exec_result = {"exec_status": "ERROR", "error": completed.stderr.strip()}
                pre_state = {}
                post_state = {}
            else:
                payload = json.loads(completed.stdout.strip() or "{}")
                pre_state = payload.get("pre_state", {})
                post_state = payload.get("post_state", {})
                exec_result = {
                    "exec_status": payload.get("exec_status"),
                    "runtime_ms": payload.get("runtime_ms"),
                    "tick_count": payload.get("tick_count"),
                }
        else:
            if hard_reset_between:
                print("Hard resetting world...")
                reload_world(control_url)
            elif reset_between:
                print("Resetting world...")
                reset_world(control_url)
            pre_state = get_world_state(control_url, robot_name)
            print("Running BT...")
            exec_result = run_bt(bt, control_url, robot_name, timeout_s, stall_s)
            print(f"Execution result: {exec_result}")
            post_state = get_world_state(control_url, robot_name)
        if execution_mode == "headless_per_run":
            print(f"Execution result: {exec_result}")
        success_result = evaluate_success(task, pre_state, post_state)
        print(f"Success eval: {success_result}")
        row.update(exec_result)
        row.update(success_result)

        results.append(row)

        if args.step:
            input("Press Enter to continue...")

        out_path.write_text(json.dumps(results, indent=2), encoding="utf-8")
        print(f"Wrote results to {out_path}")


if __name__ == "__main__":
    main()
