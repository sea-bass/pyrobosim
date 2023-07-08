#!/usr/bin/env python3

"""
Test script for world knowledge utilities
"""

import pytest
import numpy as np

import pyrobosim.core
from pyrobosim.utils.knowledge import (
    apply_resolution_strategy,
    query_to_entity,
    resolve_to_location,
    resolve_to_object,
)
from pyrobosim.utils.pose import Pose
from pyrobosim.core import Robot


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
    pass


if __name__ == "__main__":
    test_apply_resolution_strategy()
    test_query_to_entity()
    test_resolve_to_location()
    test_resolve_to_object()
