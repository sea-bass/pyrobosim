"""Skill schema definitions for PyRoboSim actions."""

from __future__ import annotations

from typing import Any


def get_skill_schemas() -> list[dict[str, Any]]:
    return [
        {
            "name": "navigate",
            "description": "Plan and follow a path to a target location or pose.",
            "params": {
                "target_location": {
                    "type": "string",
                    "source": ["literal", "blackboard"],
                    "required": False,
                },
                "pose": {
                    "type": "pose",
                    "source": ["literal", "blackboard"],
                    "required": False,
                },
                "path": {
                    "type": "path",
                    "source": ["literal", "blackboard"],
                    "required": False,
                },
            },
            "outputs": ["status", "message", "last_nav_result", "battery_level"],
        },
        {
            "name": "pick",
            "description": "Pick up an object at the current location.",
            "params": {
                "object": {
                    "type": "string",
                    "source": ["literal", "blackboard"],
                    "required": False,
                },
                "pose": {
                    "type": "pose",
                    "source": ["literal", "blackboard"],
                    "required": False,
                },
            },
            "outputs": ["status", "message", "battery_level"],
        },
        {
            "name": "place",
            "description": "Place the currently held object at the current location.",
            "params": {
                "pose": {
                    "type": "pose",
                    "source": ["literal", "blackboard"],
                    "required": False,
                },
            },
            "outputs": ["status", "message", "battery_level"],
        },
        {
            "name": "detect",
            "description": "Detect objects at the current location.",
            "params": {
                "object": {
                    "type": "string",
                    "source": ["literal", "blackboard"],
                    "required": False,
                },
            },
            "outputs": ["status", "message", "detected_objects", "battery_level"],
        },
        {
            "name": "open",
            "description": "Open the current location or target location if provided.",
            "params": {
                "target_location": {
                    "type": "string",
                    "source": ["literal", "blackboard"],
                    "required": False,
                }
            },
            "outputs": ["status", "message", "battery_level"],
        },
        {
            "name": "close",
            "description": "Close the current location or target location if provided.",
            "params": {
                "target_location": {
                    "type": "string",
                    "source": ["literal", "blackboard"],
                    "required": False,
                }
            },
            "outputs": ["status", "message", "battery_level"],
        },
    ]
