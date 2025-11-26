"""Robot policies backed by local LLMs."""

from __future__ import annotations

import json
from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Iterable, TYPE_CHECKING, Any

from ..core.locations import ObjectSpawn
from ..core.objects import Object
from ..planning.actions import TaskAction, TaskPlan
from .local_llm import TextGenerator

if TYPE_CHECKING:
    from ..core.locations import Location
    from ..core.robot import Robot
    from ..core.world import World


class LLMPolicyError(RuntimeError):
    """Raised when the LLM policy cannot produce a valid plan."""


class RobotPolicy(ABC):
    """Base class for policies that decide robot behaviour."""

    @abstractmethod
    def propose_plan(self, world: World, robot: Robot) -> TaskPlan | None:
        """Return a task plan for the robot given the current world state."""


def _format_locations(world: "World") -> str:
    """Create a compact summary of locations and their objects."""
    lines: list[str] = []
    for location in world.locations:
        objects = _objects_at_location(location)
        obj_str = ", ".join(objects) if objects else "empty"
        lines.append(f"- {location.name} ({location.category}): {obj_str}")
    return "\n".join(lines)


def _objects_at_location(location: "Location") -> list[str]:
    """Return names of objects currently spawned at a location."""
    names: list[str] = []
    for spawn in getattr(location, "children", []):
        if not isinstance(spawn, ObjectSpawn):
            continue
        for obj in getattr(spawn, "children", []):
            if isinstance(obj, Object):
                names.append(obj.name)
    return names


def _format_robot_state(robot: "Robot") -> str:
    """Summarize the robot state for the prompt."""
    pose = robot.get_pose()
    location_name = robot.location.name if robot.location else "unknown"
    carried_object = robot.manipulated_object.name if robot.manipulated_object else "none"
    yaw = pose.get_yaw()
    return (
        f"Robot {robot.name} at {location_name} "
        f"(pose=({pose.x:.2f}, {pose.y:.2f}, yaw={yaw:.2f})) "
        f"holding {carried_object}"
    )


def _extract_json_payload(text: str) -> Any:
    """
    Extract and parse the first JSON object or array from the text.

    :raises ValueError: if no JSON payload could be found.
    """
    start = None
    for idx, char in enumerate(text):
        if char in "{[":
            start = idx
            break
    if start is None:
        raise ValueError("No JSON payload found in model output.")

    stack = [text[start]]
    in_string = False
    escape = False
    pairs = {"{": "}", "[": "]"}
    for idx in range(start + 1, len(text)):
        char = text[idx]
        if in_string:
            if escape:
                escape = False
            elif char == "\\":
                escape = True
            elif char == '"':
                in_string = False
            continue

        if char == '"':
            in_string = True
            continue

        if char in "{[":
            stack.append(char)
        elif char in "}]":
            if not stack:
                raise ValueError("Found closing brace before opening brace in model output.")
            opener = stack.pop()
            if pairs.get(opener) != char:
                raise ValueError("Mismatched braces in JSON payload.")
            if not stack:
                candidate = text[start : idx + 1]
                return json.loads(candidate)
    raise ValueError("JSON payload not terminated in model output.")


def _action_from_payload(payload: dict, robot_name: str) -> TaskAction:
    """Convert a JSON payload into a TaskAction."""
    action_type = payload.get("type")
    if action_type is None:
        raise KeyError("Missing 'type' in action payload.")

    valid_types = {"navigate", "pick", "place", "open", "close"}
    if action_type.lower() not in valid_types:
        raise ValueError(f"Unsupported action type: {action_type}")

    return TaskAction(
        type=action_type,
        robot=payload.get("robot", robot_name),
        object=payload.get("object"),
        room=payload.get("room"),
        source_location=payload.get("source_location"),
        target_location=payload.get("target_location"),
    )
def _actions_from_payload(payload: Any, robot_name: str) -> list[TaskAction]:
    """Convert a payload (object, list, or dict with 'actions') into TaskActions."""
    actions_raw: list[dict[str, Any]]

    if isinstance(payload, list):
        actions_raw = payload
    elif isinstance(payload, dict):
        if "actions" in payload and isinstance(payload["actions"], list):
            actions_raw = payload["actions"]
        elif "type" in payload:
            actions_raw = [payload]
        else:
            raise KeyError("JSON payload missing 'actions' list or 'type' field.")
    else:
        raise TypeError("JSON payload must be an object or array.")

    if len(actions_raw) == 0:
        raise ValueError("JSON payload did not contain any actions.")

    return [_action_from_payload(action, robot_name) for action in actions_raw]


@dataclass
class LLMPolicy(RobotPolicy):
    """Policy that queries a local LLM for the next step."""

    generator: TextGenerator
    prompt_template: str = (
        "You control service robot {robot_name}. "
        "Use the provided world state to plan a short sequence of high-level actions.\n\n"
        "World locations:\n{location_summary}\n\n"
        "Robot state:\n{robot_summary}\n\n"
        "Respond with a JSON array of ordered actions (earliest first). Each action "
        "must include the key 'type' (navigate|pick|place|open|close) and only the other "
        "keys it needs: object, room, source_location, target_location. "
        "Always include a navigate action before any pick/place at a different location. "
        "Stop once the robot has reached the appropriate location and can pick or place. Example:\n"
        "[{{\"type\": \"navigate\", \"target_location\": \"kitchen_table\"}}, "
        "{{\"type\": \"pick\", \"object\": \"banana0\", \"source_location\": \"table0_tabletop\"}}]\n"
        "Do not add explanations outside of the JSON array."
    )
    max_tokens: int = 256
    temperature: float = 0.1
    stop_sequences: Iterable[str] | None = ("###",)
    user_directive: str | None = None

    def set_user_directive(self, directive: str | None) -> None:
        """Update the free-form user directive included in the prompt."""
        self.user_directive = directive.strip() if directive else None

    def propose_plan(self, world: "World", robot: "Robot") -> TaskPlan | None:
        if robot.world is None:
            raise LLMPolicyError("Robot must belong to a world before calling the policy.")

        prompt = self.prompt_template.format(
            robot_name=robot.name,
            location_summary=_format_locations(world),
            robot_summary=_format_robot_state(robot),
        )
        if self.user_directive:
            prompt += f"\n\nUser request:\n{self.user_directive}\n"

        output = self.generator.generate(
            prompt,
            max_tokens=self.max_tokens,
            temperature=self.temperature,
            stop=list(self.stop_sequences) if self.stop_sequences else None,
        )

        try:
            payload = _extract_json_payload(output)
            actions = _actions_from_payload(payload, robot.name)
        except Exception as exc:
            raise LLMPolicyError(f"Failed to parse LLM output: {exc}. Output was: {output}") from exc

        return TaskPlan(robot=robot.name, actions=actions)
