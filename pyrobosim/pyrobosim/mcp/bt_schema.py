"""BT JSON schema (YAML-backed)."""

from __future__ import annotations

from pathlib import Path
from typing import Any

import yaml


_DATA_PATH = Path(__file__).resolve().parent / "data" / "bt_schema.yaml"


def get_bt_schema() -> dict[str, Any]:
    data = yaml.safe_load(_DATA_PATH.read_text(encoding="utf-8"))
    return data if isinstance(data, dict) else {}
