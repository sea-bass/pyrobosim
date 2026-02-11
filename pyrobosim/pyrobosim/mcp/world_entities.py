"""Utilities for listing world entities with controllable detail."""

from __future__ import annotations

from typing import Any

from pyrobosim.core.locations import ObjectSpawn
from pyrobosim.core.objects import Object
from pyrobosim.core.robot import Robot
from pyrobosim.core.world import World


def build_world_entities(
    world: World,
    mode: str = "vocab",
    robot: Robot | None = None,
    allow_full: bool = False,
) -> dict[str, Any]:
    """
    Build a dictionary of world entities.

    Modes:
      - vocab: names/categories only (no ground-truth locations or open/locked state)
      - observed: includes only robot-known objects (partial observability)
      - full: includes ground-truth object locations and open/locked state
    """
    if mode not in ("vocab", "observed", "full"):
        raise ValueError("mode must be one of: vocab, observed, full")
    if mode == "full" and not allow_full:
        raise ValueError("full mode requires allow_full=True")
    if mode == "observed" and robot is None:
        raise ValueError("observed mode requires a robot")

    rooms = [room.name for room in world.rooms]

    locations: list[dict[str, Any]] = []
    object_spawns: list[dict[str, Any]] = []
    for loc in world.locations:
        loc_entry: dict[str, Any] = {
            "name": loc.name,
            "category": loc.category,
        }
        if loc.parent is not None:
            loc_entry["room"] = getattr(loc.parent, "name", None)
        if mode == "full":
            loc_entry["is_open"] = loc.is_open
            loc_entry["is_locked"] = loc.is_locked
        locations.append(loc_entry)

        for child in loc.children:
            if isinstance(child, ObjectSpawn):
                spawn_entry: dict[str, Any] = {
                    "name": child.name,
                    "location": loc.name,
                }
                if loc.parent is not None:
                    spawn_entry["room"] = getattr(loc.parent, "name", None)
                object_spawns.append(spawn_entry)

    hallways: list[dict[str, Any]] = []
    for hall in world.hallways:
        hall_entry: dict[str, Any] = {
            "name": hall.name,
            "room_start": hall.room_start.name,
            "room_end": hall.room_end.name,
        }
        if mode == "full":
            hall_entry["is_open"] = hall.is_open
            hall_entry["is_locked"] = hall.is_locked
        hallways.append(hall_entry)

    object_categories = sorted({obj.category for obj in world.objects})

    if mode == "observed":
        objects = sorted(robot.get_known_objects(), key=lambda o: o.name)
    else:
        objects = sorted(world.objects, key=lambda o: o.name)

    object_entries: list[dict[str, Any]] = []
    for obj in objects:
        if not isinstance(obj, Object):
            continue
        entry: dict[str, Any] = {"name": obj.name, "category": obj.category}
        if mode == "full":
            entry["location"] = getattr(obj.parent, "name", None)
        object_entries.append(entry)

    payload: dict[str, Any] = {
        "mode": mode,
        "rooms": rooms,
        "locations": locations,
        "object_spawns": object_spawns,
        "object_categories": object_categories,
        "objects": object_entries,
        "hallways": hallways,
    }

    if mode == "observed":
        closed = []
        if robot is not None:
            closed = [h.name for h in robot.get_known_closed_hallways()]
        payload["known_closed_hallways"] = closed

    return payload
