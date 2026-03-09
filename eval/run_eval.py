"""Manual-mode evaluation harness for BT composition quality and runtime outcomes."""

from __future__ import annotations

import argparse
import json
import subprocess
from pathlib import Path
from typing import Any

from pyrobosim.mcp.skills import get_skill_schemas

from eval.lib.io_utils import (
    extract_json_payload,
    load_bt,
    load_submissions,
    load_yaml,
    rate,
    select_submission_by_id,
    select_submission_by_prompt,
    task_archetype,
)
from eval.lib.reporting import log
from eval.lib.runtime_utils import get_world_state, reload_world, reset_world, run_bt_control
from eval.lib.static_metrics import evaluate_static_metrics
from eval.lib.success_metrics import evaluate_success, get_world_location_names


def main() -> None:
    parser = argparse.ArgumentParser(description="Run evaluation harness.")
    parser.add_argument("--tasks-file", default="tasks.yaml", help="Tasks YAML file.")
    parser.add_argument("--task-id", help="Run a single task by id.")
    parser.add_argument("--step", action="store_true", help="Pause between tasks.")
    parser.add_argument("--submission-id", help="Evaluate a specific submission id.")
    parser.add_argument(
        "--headless-progress-interval-s",
        type=float,
        default=2.0,
        help="Heartbeat interval for headless runs (seconds). Use 0 to disable.",
    )
    args = parser.parse_args()

    root_dir = Path(__file__).resolve().parent
    repo_root = root_dir.parent
    config = load_yaml(root_dir / args.tasks_file)
    world_locations = get_world_location_names(config.get("world_file"))

    bt_dir = Path(repo_root / config.get("bt_output_dir", ""))
    submissions = load_submissions(root_dir / "submissions.jsonl")
    control_url = config.get("control_server", "http://127.0.0.1:9001")
    execution_mode = config.get("execution_mode", "control_server")
    runtime_cfg = config.get("runtime", {})
    timeout_s = int(runtime_cfg.get("timeout_s", 30))
    tick_period_s = float(runtime_cfg.get("tick_period_s", 0.1))
    reset_between = bool(runtime_cfg.get("reset_between", False))
    hard_reset_between = bool(runtime_cfg.get("hard_reset_between", False))

    skills = get_skill_schemas()

    results: list[dict[str, Any]] = []
    out_path = root_dir / (f"{args.task_id}_results.json" if args.task_id else "results.json")
    log(f"Loaded {len(config.get('tasks', []))} tasks", "info")
    log(f"Submissions log: {root_dir / 'submissions.jsonl'}", "info")
    log(f"BT output dir: {bt_dir}", "info")
    log(f"Control server: {control_url}", "info")
    log(f"Timeout: {timeout_s}s", "info")
    log(f"Tick period: {tick_period_s}s", "info")
    tasks = config.get("tasks", [])
    if args.task_id:
        tasks = [t for t in tasks if t.get("id") == args.task_id]

    for task in tasks:
        task_id = task.get("id")
        print("")
        log(f"Evaluating {task_id}", "task")
        log(f"Prompt: {task.get('task_prompt', '')}", "info")
        submission = None
        bt_path = bt_dir / f"{task_id}.json"

        if args.submission_id:
            submission = select_submission_by_id(args.submission_id, submissions)
        else:
            submission = select_submission_by_prompt(task.get("task_prompt", ""), submissions)
        if submission and submission.get("generated"):
            generated_path = Path(submission["generated"])
            bt_path = generated_path if generated_path.is_absolute() else (repo_root / generated_path)
            log(f"Using submission: {bt_path}", "ok")
        elif bt_path.exists():
            log(f"Using task_id file: {bt_path}", "warn")
        else:
            log(f"Missing BT file: {bt_path}", "error")

        if not bt_path.exists():
            log("Missing BT file.", "error")
            results.append({"id": task_id, "missing_bt": True})
            continue

        bt = load_bt(bt_path)
        static_metrics = evaluate_static_metrics(bt, skills, task)
        log(
            "Static metrics: "
            f"valid={static_metrics['valid']}, "
            f"grounded={static_metrics['grounded']}, "
            f"params_valid={static_metrics['params_valid']}, "
            f"struct_comp={static_metrics['struct_comp']}",
            "info"
            if all(
                (
                    static_metrics["valid"],
                    static_metrics["grounded"],
                    static_metrics["params_valid"],
                    static_metrics["struct_comp"],
                )
            )
            else "warn",
        )

        row = {
            "id": task_id,
            "archetype": task_archetype(task),
            "task_prompt": task.get("task_prompt", ""),
            "valid": static_metrics["valid"],
            "grounded": static_metrics["grounded"],
            "params_valid": static_metrics["params_valid"],
            "struct_comp": static_metrics["struct_comp"],
            "valid_issues": static_metrics["valid_issues"],
            "grounded_issues": static_metrics["grounded_issues"],
            "params_issues": static_metrics["params_issues"],
            "struct_issues": static_metrics["struct_issues"],
            "struct_checks": static_metrics["struct_checks"],
            "bt_path": str(bt_path),
            # Placeholder for trace compliance metrics. We will compute this from
            # execution traces in a follow-up change.
            "trace_comp": None,
        }

        robot_name = config.get("robot_name", "robot")
        if execution_mode == "headless_per_run":
            log("Running BT headless in a fresh process...", "info")
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
                "--tick-period-s",
                str(tick_period_s),
                "--progress-interval-s",
                str(args.headless_progress_interval_s),
            ]
            subprocess_timeout_s = timeout_s + 20
            import pdb;pdb.set_trace()
            try:
                completed = subprocess.run(
                    headless_cmd,
                    stdout=subprocess.PIPE,
                    stderr=None,
                    text=True,
                    timeout=subprocess_timeout_s,
                )
                log(f"Headless process exited rc={completed.returncode}", "info")
            except subprocess.TimeoutExpired:
                log(f"Headless process timeout after {subprocess_timeout_s}s", "error")
                exec_result = {
                    "exec_status": "ERROR",
                    "error": f"headless subprocess timeout ({subprocess_timeout_s}s)",
                }
                pre_state = {}
                post_state = {}
                completed = None

            if completed is not None and completed.returncode != 0:
                exec_result = {
                    "exec_status": "ERROR",
                    "error": f"headless runner exited with code {completed.returncode}",
                }
                pre_state = {}
                post_state = {}
            elif completed is not None:
                log("Parsing headless JSON payload...", "info")
                payload = extract_json_payload(completed.stdout)
                pre_state = payload.get("pre_state", {})
                post_state = payload.get("post_state", {})
                exec_result = {
                    "exec_status": payload.get("exec_status"),
                    "runtime_ms": payload.get("runtime_ms"),
                    "tick_count": payload.get("tick_count"),
                }
        else:
            if hard_reset_between:
                log("Hard resetting world...", "warn")
                reload_world(control_url)
            elif reset_between:
                log("Resetting world...", "warn")
                reset_world(control_url)
            pre_state = get_world_state(control_url, robot_name)
            log("Running BT...", "info")
            exec_result = run_bt_control(bt, control_url, robot_name, timeout_s)
            log(
                f"Execution result: {exec_result}",
                "ok" if exec_result.get("exec_status") == "SUCCESS" else "warn",
            )
            post_state = get_world_state(control_url, robot_name)

        if execution_mode == "headless_per_run":
            log(
                f"Execution result: {exec_result}",
                "ok" if exec_result.get("exec_status") == "SUCCESS" else "warn",
            )

        success_result = evaluate_success(task, pre_state, post_state, world_locations)
        log(f"Success eval: {success_result}", "ok" if success_result.get("goal_satisfied") else "warn")

        row.update(exec_result)
        row.update(success_result)
        row["success"] = row.get("goal_satisfied")
        results.append(row)

        if args.step:
            input("Press Enter to continue...")

        out_path.write_text(json.dumps(results, indent=2), encoding="utf-8")
        log(f"Wrote results to {out_path}", "info")

    summary_by_archetype: dict[str, dict[str, Any]] = {}
    for row in results:
        archetype = str(row.get("archetype", "unlabeled"))
        summary_by_archetype.setdefault(archetype, {"rows": []})
        summary_by_archetype[archetype]["rows"].append(row)

    summary_out: dict[str, Any] = {}
    for archetype, block in summary_by_archetype.items():
        rows = block["rows"]
        summary_out[archetype] = {
            "count": len(rows),
            "valid_rate": rate(rows, "valid"),
            "grounded_rate": rate(rows, "grounded"),
            "params_rate": rate(rows, "params_valid"),
            "struct_comp_rate": rate(rows, "struct_comp"),
            "success_rate": rate(rows, "success"),
            "trace_comp_rate": rate(rows, "trace_comp"),
        }

    summary_path = root_dir / "results_summary_by_archetype.json"
    summary_path.write_text(json.dumps(summary_out, indent=2), encoding="utf-8")
    log(f"Wrote archetype summary to {summary_path}", "ok")


if __name__ == "__main__":
    main()
