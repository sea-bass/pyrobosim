from __future__ import annotations

import json
from datetime import datetime
from pathlib import Path
from typing import Any

import yaml


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


def extract_json_payload(stdout_text: str) -> dict[str, Any]:
    lines = [ln.strip() for ln in stdout_text.splitlines() if ln.strip()]
    for line in reversed(lines):
        try:
            payload = json.loads(line)
            if isinstance(payload, dict):
                return payload
        except Exception:
            continue
    return {}


def rate(rows: list[dict[str, Any]], field: str) -> float | None:
    vals = [r.get(field) for r in rows if isinstance(r.get(field), bool)]
    if not vals:
        return None
    return round(sum(1 for v in vals if v) / len(vals), 4)


def task_archetype(task: dict[str, Any]) -> str:
    return task.get("archetype", "unlabeled")
