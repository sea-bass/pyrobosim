from __future__ import annotations

from typing import Any

from pyrobosim.core.yaml_utils import WorldYamlLoader
from pyrobosim.utils.general import get_data_folder


def has_object(state: dict[str, Any], *, name: str | None = None, category: str | None = None) -> bool:
    for obj in state.get("objects", []):
        if name and obj.get("name") == name:
            return True
        if category and obj.get("category") == category:
            return True
    return False


def placed_category_at(state: dict[str, Any], category: str, location: str) -> bool:
    for obj in state.get("objects", []):
        if obj.get("category") == category and obj.get("location") == location:
            return True
    return False


def object_at(state: dict[str, Any], name: str, location: str) -> bool:
    for obj in state.get("objects", []):
        if obj.get("name") == name and obj.get("location") == location:
            return True
    return False


def location_exists(state: dict[str, Any], location: str) -> bool:
    if not location:
        return False
    if state.get("location") == location:
        return True
    for obj in state.get("objects", []):
        if obj.get("location") == location:
            return True
    return False


def get_world_location_names(world_file: str | None) -> set[str]:
    if not world_file:
        return set()
    world = WorldYamlLoader().from_file(get_data_folder() / world_file)
    try:
        names: set[str] = set()
        names.update(room.name for room in world.rooms)
        names.update(loc.name for loc in world.locations)
        names.update(spawn.name for spawn in world.object_spawns)
        names.update(h.name for h in world.hallways)
        return {n for n in names if n}
    finally:
        world.shutdown()


def location_is_known(location: str, pre_state: dict[str, Any], world_locations: set[str] | None = None) -> bool:
    if not location:
        return False
    if world_locations and location in world_locations:
        return True
    return location_exists(pre_state, location)


def evaluate_success(
    task: dict[str, Any],
    pre: dict[str, Any],
    post: dict[str, Any],
    world_locations: set[str] | None = None,
) -> dict[str, Any]:
    success = task.get("success", {}) or {}
    result: dict[str, Any] = {}

    feasible = True
    if "must_hold_name" in success:
        feasible = feasible and has_object(pre, name=success["must_hold_name"])
    if "must_hold_category" in success:
        feasible = feasible and has_object(pre, category=success["must_hold_category"])
    if "must_place_category_at" in success:
        cat = success["must_place_category_at"].get("category")
        loc = success["must_place_category_at"].get("location")
        if cat:
            feasible = feasible and has_object(pre, category=cat)
        if loc is None:
            feasible = False
        if loc:
            feasible = feasible and location_is_known(loc, pre, world_locations)
    if "object_at" in success:
        name = success["object_at"].get("name")
        loc = success["object_at"].get("location")
        if name:
            feasible = feasible and has_object(pre, name=name)
        if loc is None:
            feasible = False
        if loc:
            feasible = feasible and location_is_known(loc, pre, world_locations)
    if "robot_at" in success:
        loc = success["robot_at"]
        feasible = feasible and location_is_known(loc, pre, world_locations)

    result["feasible"] = feasible
    if not feasible:
        result["goal_satisfied"] = False
        return result

    held = post.get("held_object") or {}
    checks: list[bool] = []
    if "must_hold_name" in success:
        checks.append(held.get("name") == success["must_hold_name"])
    if "must_hold_category" in success:
        checks.append(held.get("category") == success["must_hold_category"])
    if "must_place_category_at" in success:
        cat = success["must_place_category_at"].get("category")
        loc = success["must_place_category_at"].get("location")
        checks.append(bool(cat and loc and placed_category_at(post, cat, loc)))
    if "object_at" in success:
        name = success["object_at"].get("name")
        loc = success["object_at"].get("location")
        checks.append(bool(name and loc and object_at(post, name, loc)))
    if "robot_at" in success:
        checks.append(post.get("location") == success["robot_at"])
    if "not_holding" in success:
        checks.append(post.get("held_object") is None)

    result["goal_satisfied"] = all(checks) if checks else None
    return result
