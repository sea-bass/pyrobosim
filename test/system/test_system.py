#!/usr/bin/env python3

"""
System-level tests for the pyrobosim UI functionality to execute tasks.
"""

import os
import sys
import time

import pytest

from pyrobosim.core import WorldYamlLoader
from pyrobosim.gui import PyRoboSimGUI
from pyrobosim.utils.knowledge import query_to_entity


# Needed for PyQt5 tests to work with CI
os.environ["QT_QPA_PLATFORM"] = "offscreen"


@pytest.mark.skipif(
    sys.version_info < (3, 10), reason="Test does not work in versions before 3.10."
)
class TestSystem:
    @pytest.fixture(autouse=True)
    def create_world_and_app(self):
        # Load world from file.
        cur_path = os.path.dirname(os.path.realpath(__file__))
        world_file_path = os.path.join(cur_path, "test_system_world.yaml")
        world = WorldYamlLoader().from_yaml(world_file_path)

        # Create headless app.
        self.app = PyRoboSimGUI(world, sys.argv, show=False)
        time.sleep(0.5)

    def nav_helper(self, nav_query):
        """
        Helper function to test navigation UI action.

        :param nav_query: Query for navigation goal.
        :type nav_query: str
        """
        window = self.app.main_window
        world = self.app.world
        robot = window.get_current_robot()
        expected_location = query_to_entity(
            world,
            nav_query.split(" "),
            mode="location",
            robot=robot,
            resolution_strategy="nearest",
        )

        window.goal_textbox.setText(nav_query)
        window.on_navigate_click()

        while not robot.executing_nav:
            time.sleep(0.1)
        while robot.executing_nav:
            time.sleep(0.1)

        assert (
            robot.location == expected_location
            or robot.location in expected_location.children
        )

    @pytest.mark.dependency(name="test_nav")
    def test_nav(self):
        """
        Test navigation UI action.
        """
        nav_queries = [
            "bathroom",
            "bedroom desk",
            "counter0_right",
            "kitchen apple",
        ]
        for nav_query in nav_queries:
            self.nav_helper(nav_query)

    @pytest.mark.dependency(name="test_pick_place", depends=["test_nav"])
    def test_pick_place(self):
        """
        Test pick and place UI actions.
        """
        pick_place_queries = [
            ("table", "gala", "table"),  # Pick and place in same location
            ("counter0_left", "water", "desk"),  # Pick and place in different location
        ]

        window = self.app.main_window
        world = self.app.world
        robot = window.get_current_robot()

        for pick_query, obj_query, place_query in pick_place_queries:
            # Navigate to pick location
            self.nav_helper(pick_query)

            # Pick an object
            expected_object = query_to_entity(
                world,
                obj_query.split(" "),
                mode="object",
                robot=robot,
                resolution_strategy="nearest",
            )
            window.goal_textbox.setText(obj_query)
            window.on_pick_click()
            assert robot.manipulated_object == expected_object

            # Navigate to place location
            self.nav_helper(place_query)

            # Place an object
            window.on_place_click()
            assert robot.manipulated_object is None
