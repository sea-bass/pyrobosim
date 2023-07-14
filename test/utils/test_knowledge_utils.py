#!/usr/bin/env python3

"""
Test script for world knowledge utilities
"""

import pytest
import os
import numpy as np

from pyrobosim.core import WorldYamlLoader
from pyrobosim.core import Robot
from pyrobosim.core import World

# import functions to test
from pyrobosim.utils.knowledge import (
    apply_resolution_strategy,
    query_to_entity,
    resolve_to_location,
    resolve_to_object,
)

from pyrobosim.utils.pose import Pose
from pyrobosim.utils.general import get_data_folder


def load_world():
    """Load a test world."""
    world_file = os.path.join(get_data_folder(), "test_world.yaml")
    return WorldYamlLoader().from_yaml(world_file)


def test_apply_resolution_strategy():
    # First, test all strategies with empty entity_list
    entity_list = []
    entity = apply_resolution_strategy([], entity_list, "first")
    assert entity is None
    entity = apply_resolution_strategy([], entity_list, "random")
    assert entity is None
    entity = apply_resolution_strategy([], entity_list, "nearest")
    assert entity is None

    # Test non-existent strategy
    entity_list = ["don't return this"]
    # catch the warning as an error
    import warnings

    warnings.filterwarnings("error")
    try:
        entity = apply_resolution_strategy([], entity_list, "non-existent")
    except UserWarning:
        # test passed, this should warn
        pass
    else:
        assert (
            False
        ), "apply_resolution_strategy didn't warn on non-implemented resolution strategy"

    # Test 'first' strategy
    entity_list = ["First", "Second"]
    entity = apply_resolution_strategy([], entity_list, "first")
    assert entity == "First"

    entity_list = [["list"], "second"]
    entity = apply_resolution_strategy([], entity_list, "first")
    assert entity == ["list"]

    # Test 'random' strategy
    entity_list = ["First", "Second", "Third"]
    for _ in range(0, 3):
        # try multiple times
        entity = apply_resolution_strategy([], entity_list, "first")
        assert entity in entity_list

    entity_list = ["Only"]
    entity = apply_resolution_strategy([], entity_list, "first")
    assert entity == "Only"

    # Test 'nearest' strategy
    # Test that no robot warns
    entity_list = ["Only"]
    try:
        entity = apply_resolution_strategy([], entity_list, "nearest", None)
    except UserWarning:
        # test passed, this should warn
        pass
    else:
        assert (
            False
        ), "apply_resolution_strategy didn't warn on non-existent robot strategy"

    robot = Robot("test_robot")

    class Entity:
        pose = Pose()

    entity_list = [Entity(), Entity(), Entity()]
    entity_list[0].pose = Pose.from_list([1, 0])
    entity_list[1].pose = Pose.from_list([2, 0])
    entity_list[2].pose = Pose.from_list([3, 0])
    entity = apply_resolution_strategy([], entity_list, "nearest", robot)
    assert entity == entity_list[0]

    robot.pose = Pose.from_list([2, 1])
    entity = apply_resolution_strategy([], entity_list, "nearest", robot)
    assert entity == entity_list[1]

    robot.pose = Pose.from_list([1000, -1000])
    entity = apply_resolution_strategy([], entity_list, "nearest", robot)
    assert entity == entity_list[2]


def test_query_to_entity():
    pass


def test_resolve_to_location():
    pass


def test_resolve_to_object():
    test_world = load_world()

    # test that we can get the first object added to the world
    obj = resolve_to_object(test_world)
    assert obj.name == "banana0"

    for category in ["apple", "banana", "water", "coke"]:
        # all of these can be found somewhere
        obj = resolve_to_object(test_world, category=category)
        assert obj.category == category

    for location in ["table", "desk", "counter", "trash_can"]:
        # all of these locations have objects
        obj = resolve_to_object(test_world, location=location)
        assert obj.parent.category == location

    for room in ["kitchen", "bedroom", "bathroom"]:
        # all of these rooms have objects
        obj = resolve_to_object(test_world, room=room)
        assert obj.parent.parent.parent.name == room

    # now test specific objects

    # set our position to be the same as a banana on the desk and make sure we find that
    robot = Robot("test_robot")
    robot.pose = Pose.from_list([3.2, 3.5, 0.0, 0.0])
    obj = resolve_to_object(test_world, resolution_strategy="nearest", robot=robot)
    assert obj.category == "apple" and obj.parent.parent.name == "my_desk"

    # this shouldn't be the apple even though it's nearest because it doesn't fit the category
    obj = resolve_to_object(
        test_world, category="banana", resolution_strategy="nearest", robot=robot
    )
    assert not (obj.category == "apple" and obj.parent.parent.name == "my_desk")

    # this shouldn't be the apple even though it's nearest because it doesn't fit the location
    obj = resolve_to_object(
        test_world, location="table", resolution_strategy="nearest", robot=robot
    )
    assert not (obj.category == "apple" and obj.parent.parent.name == "my_desk")

    # this shouldn't be the apple even though it's nearest because it doesn't fit the room
    obj = resolve_to_object(
        test_world, room="bathroom", resolution_strategy="nearest", robot=robot
    )
    assert not (obj.category == "apple" and obj.parent.parent.name == "my_desk")

    # if we pick up the nearest object we should find it when not ignoring grasped, but not when ignoring grasped
    obj_nearest = resolve_to_object(
        test_world, resolution_strategy="nearest", robot=robot
    )
    robot._attach_object(obj_nearest)
    obj = resolve_to_object(
        test_world, resolution_strategy="nearest", ignore_grasped=False, robot=robot
    )
    assert obj == obj_nearest
    obj = resolve_to_object(
        test_world, resolution_strategy="nearest", ignore_grasped=True, robot=robot
    )
    assert obj != obj_nearest


if __name__ == "__main__":
    test_apply_resolution_strategy()
    test_query_to_entity()
    test_resolve_to_location()
    test_resolve_to_object()
