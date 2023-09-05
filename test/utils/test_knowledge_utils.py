#!/usr/bin/env python3

"""
Unit tests for world knowledge utilities.
"""

import os
import pytest

from pyrobosim.core import WorldYamlLoader
from pyrobosim.core import Robot

# import functions to test
from pyrobosim.utils.knowledge import (
    apply_resolution_strategy,
    query_to_entity,
    resolve_to_location,
    resolve_to_object,
)
from pyrobosim.utils.pose import Pose
from pyrobosim.utils.general import get_data_folder


class MockEntity:
    pose = Pose()


def load_world():
    """Load a test world."""
    world_file = os.path.join(get_data_folder(), "test_world.yaml")
    return WorldYamlLoader().from_yaml(world_file)


def test_apply_resolution_strategy():
    # First, test all strategies with empty entity_list
    entity_list = []
    entity = apply_resolution_strategy(entity_list, "first")
    assert entity is None
    entity = apply_resolution_strategy(entity_list, "random")
    assert entity is None
    entity = apply_resolution_strategy(entity_list, "nearest")
    assert entity is None

    # Test non-existent strategy
    entity_list = ["don't return this"]

    with pytest.warns(UserWarning):
        entity = apply_resolution_strategy(entity_list, "non-existent")


def test_apply_first_resolution_strategy():
    # Test 'first' strategy
    entity_list = ["First", "Second"]
    entity = apply_resolution_strategy(entity_list, "first")
    assert entity == "First"

    # Test 'random' strategy
    entity_list = ["First", "Second", "Third"]
    for _ in range(0, 3):
        # try multiple times
        entity = apply_resolution_strategy(entity_list, "random")
        assert entity in entity_list

    entity_list = ["Only"]
    entity = apply_resolution_strategy(entity_list, "first")
    assert entity == "Only"


def test_apply_nearest_resolution_strategy():
    # Test 'nearest' strategy
    # Test that no robot warns
    entity_list = ["Only"]
    with pytest.warns(UserWarning):
        entity = apply_resolution_strategy(entity_list, "nearest", None)

    robot = Robot("test_robot")

    entity_list = [MockEntity(), MockEntity(), MockEntity()]
    entity_list[0].pose = Pose(x=1, y=0)
    entity_list[1].pose = Pose(x=2, y=0)
    entity_list[2].pose = Pose(x=3, y=0)
    entity = apply_resolution_strategy(entity_list, "nearest", robot)
    assert entity == entity_list[0]

    robot.pose = Pose(x=2, y=1)
    entity = apply_resolution_strategy(entity_list, "nearest", robot)
    assert entity == entity_list[1]

    robot.pose = Pose(x=1000, y=-1000)
    entity = apply_resolution_strategy(entity_list, "nearest", robot)
    assert entity == entity_list[2]


def test_query_to_entity():
    test_world = load_world()
    query = "kitchen table apple"
    entity = query_to_entity(test_world, query.split(), "location")
    assert entity.name == "table0_tabletop"
    entity = query_to_entity(test_world, query.split(), "object")
    assert entity.name == "gala"

    query = "kitchen table"
    entity = query_to_entity(test_world, query.split(), "location")
    assert entity.name == "table0"

    # if we query to a random object on the kitchen table, its parent category should be table
    entity = query_to_entity(test_world, query.split(), "object", "random")
    assert entity.parent.category == "table"

    # nearest object on the kitchen table should be the banana
    robot = Robot("test_robot")
    robot.pose = Pose(x=1.0, y=-0.5)
    query = "kitchen table"
    entity = query_to_entity(test_world, query.split(), "object", "nearest", robot)
    assert (
        entity.category == "banana"
        and entity.parent.category == "table"
        and entity.parent.parent.parent.name == "kitchen"
    )

    # we want the nearest apple on the kitchen table, should no longer return the banana even though it is the nearest object
    query = "kitchen table apple"
    entity = query_to_entity(test_world, query.split(), "object", "nearest", robot)
    assert entity.name == "gala"

    # things that should warn
    query = "kitchen table fake"
    # search for nonexistent object in real location
    for mode in ["object", "location"]:
        with pytest.warns(UserWarning):
            entity = query_to_entity(test_world, query.split(), mode)

    # search for absolute garbage
    query = "fake fake fake"
    for mode in ["object", "location"]:
        with pytest.warns(UserWarning):
            entity = query_to_entity(test_world, query.split(), mode)


def test_resolve_to_location():
    test_world = load_world()

    # table0 is the first location in the test world
    loc = resolve_to_location(test_world)
    assert loc.name == "table0"

    for category in ["table", "desk", "counter", "trash_can"]:
        # we should be able to find each category
        loc = resolve_to_location(test_world, category=category)
        assert loc.category == category

    for room in ["kitchen", "bedroom", "bathroom"]:
        # all of these rooms have objects
        loc = resolve_to_location(test_world, room=room)
        assert loc.parent.name == room

    # put ourselves on a table and make sure it is the nearest one
    # then, search for a different category and make sure we find something that isn't the same table we're on
    robot = Robot("test_robot")
    robot.pose = Pose(x=0.85, y=-0.5)
    nearest_loc = resolve_to_location(
        test_world, resolution_strategy="nearest", robot=robot
    )
    assert nearest_loc.name == "table0"
    loc = resolve_to_location(
        test_world, category="desk", resolution_strategy="nearest", robot=robot
    )
    assert loc.name == "my_desk"

    # test expanding locations
    loc = resolve_to_location(
        test_world,
        category="desk",
        resolution_strategy="nearest",
        robot=robot,
        expand_locations=True,
    )
    assert loc.parent.category == "desk"

    # things that should warn
    # fake location category
    with pytest.warns(UserWarning):
        loc = resolve_to_location(test_world, category="fake")

    # combination that doesn't exist
    with pytest.warns(UserWarning):
        loc = resolve_to_location(test_world, room="bedroom", category="trash_can")


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


def test_specific_resolve_to_object():
    test_world = load_world()
    # now test specific objects

    # set our position to be the same as a banana on the desk and make sure we find that
    robot = Robot("test_robot")
    robot.pose = Pose(x=3.2, y=3.5)
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


def test_resolve_to_object_warnings():
    test_world = load_world()
    robot = Robot("test_robot")
    # things that should warn
    # fake object category
    with pytest.warns(UserWarning):
        obj = resolve_to_object(test_world, category="fake")

    # fake location
    with pytest.warns(UserWarning):
        obj = resolve_to_object(test_world, location="fake")

    # fake room
    with pytest.warns(UserWarning):
        obj = resolve_to_object(test_world, robot=robot, room="fake")

    # combination that doesn't exist
    with pytest.warns(UserWarning):
        obj = resolve_to_object(
            test_world,
            robot=robot,
            room="kitchen",
            category="banana",
            location="trash_can",
        )


def test_resolve_to_object_grasp():
    test_world = load_world()
    robot = Robot("test_robot")
    robot.pose = Pose(x=3.2, y=3.5)

    # if we pick up the nearest object we should find it when not ignoring grasped, but not when ignoring grasped
    # test this last because it changes the state of the world
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
    test_apply_nearest_resolution_strategy()
    test_apply_first_resolution_strategy()
    test_apply_nearest_resolution_strategy()
    test_query_to_entity()
    test_resolve_to_location()
    test_resolve_to_object()
    test_specific_resolve_to_object()
    test_resolve_to_object_warnings()
    test_resolve_to_object_grasp()
