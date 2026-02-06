"""Rootstock (BT template) library for synthesis guidance (YAML-backed)."""

from __future__ import annotations

from pathlib import Path
from typing import Any

import yaml


_DATA_PATH = Path(__file__).resolve().parent.parent / "mcp" / "data" / "rootstocks.yaml"


def list_rootstocks() -> list[dict[str, Any]]:
    data = yaml.safe_load(_DATA_PATH.read_text(encoding="utf-8"))
    return data.get("rootstocks", []) if isinstance(data, dict) else []
