"""Skill schema definitions for PyRoboSim actions (YAML-backed)."""

from __future__ import annotations

from pathlib import Path
from typing import Any

import yaml


_DATA_PATH = Path(__file__).resolve().parent / "data" / "skills.yaml"


def get_skill_schemas() -> list[dict[str, Any]]:
    data = yaml.safe_load(_DATA_PATH.read_text(encoding="utf-8"))
    return data.get("skills", []) if isinstance(data, dict) else []
