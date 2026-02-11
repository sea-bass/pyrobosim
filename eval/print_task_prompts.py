#!/usr/bin/env python3
"""Print task_prompt entries from a tasks YAML file."""

from __future__ import annotations

import argparse
from pathlib import Path

import yaml


def main() -> None:
    parser = argparse.ArgumentParser(description="Print task_prompt lines from tasks YAML.")
    parser.add_argument(
        "--tasks-file",
        default="tasks50.yaml",
        help="Path to tasks YAML file (default: tasks50.yaml).",
    )
    args = parser.parse_args()

    tasks_path = Path(args.tasks_file)
    if not tasks_path.is_file():
        raise SystemExit(f"Tasks file not found: {tasks_path}")

    data = yaml.safe_load(tasks_path.read_text(encoding="utf-8"))
    tasks = data.get("tasks", [])
    for idx, task in enumerate(tasks):
        prompt = task.get("task_prompt")
        if prompt:
            print(f"{idx}: task prompt: {prompt}")


if __name__ == "__main__":
    main()
