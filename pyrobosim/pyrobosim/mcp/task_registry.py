"""Task registry for evaluation binding."""

from __future__ import annotations

from pathlib import Path
from typing import Any

import yaml


class TaskRegistry:
    def __init__(self, path: Path) -> None:
        self.path = path
        self._tasks = self._load(path)

    def _load(self, path: Path) -> list[dict[str, Any]]:
        data = yaml.safe_load(path.read_text(encoding="utf-8"))
        tasks = data.get("tasks", []) if isinstance(data, dict) else []
        return [t for t in tasks if isinstance(t, dict)]

    def match_task_id(self, prompt: str | None) -> str | None:
        if not prompt:
            return None
        for task in self._tasks:
            if task.get("prompt") == prompt:
                return task.get("id")
        return None
