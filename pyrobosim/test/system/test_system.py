#!/usr/bin/env python3

"""
System-level tests for the pyrobosim UI functionality to execute tasks.
"""

import os
import sys
import pytest
import time

from pyrobosim.core import WorldYamlLoader
from pyrobosim.gui import PyRoboSimGUI
from pyrobosim.utils.knowledge import query_to_entity


# Needed for UI tests to work with CI
os.environ["QT_QPA_PLATFORM"] = "offscreen"


class TestSystem:
    @pytest.fixture(autouse=True)
    def setup_and_teardown(self):
        # Load world from file.
        cur_path = os.path.dirname(os.path.realpath(__file__))
        world_file_path = os.path.join(cur_path, "test_system_world.yaml")
        world = WorldYamlLoader().from_file(world_file_path)

        # Create headless app.
        self.app = PyRoboSimGUI.instance()
        if self.app is None:
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
            nav_query,
            mode="location",
            robot=robot,
            resolution_strategy="nearest",
        )

        window.goal_textbox.setText(nav_query)
        window.on_navigate_click()

        while not robot.executing_nav:
            time.sleep(0.2)
        while robot.executing_nav:
            time.sleep(0.2)
        robot.location = world.get_location_from_pose(robot.get_pose())

        assert robot.last_nav_result.is_success()
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
            "hall_kitchen_bathroom",
            "counter0_right",
            "kitchen apple",
        ]
        for nav_query in nav_queries:
            self.nav_helper(nav_query)

    @pytest.mark.dependency(name="test_pick_detect_place", depends=["test_nav"])
    def test_pick_detect_place(self):
        """
        Test pick, detect, and place UI actions.
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

            # Detect objects
            window.on_detect_click()

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

    @pytest.mark.dependency(name="test_open_close", depends=["test_pick_detect_place"])
    def test_open_close(self):
        """
        Test open and close UI actions.
        """
        location_queries = ["hall_kitchen_bathroom", "my_desk"]

        window = self.app.main_window
        world = self.app.world

        for location_name in location_queries:
            # Navigate to hallway location
            location = world.get_entity_by_name(location_name)
            self.nav_helper(location.name)

            # Close the location and verify that it's closed.
            window.on_close_click()
            assert not location.is_open

            # Open the location and verify that it's open.
            window.on_open_click()
            assert location.is_open

    @pytest.mark.dependency(name="test_nav_cancel", depends=["test_open_close"])
    def test_nav_cancel(self):
        """
        Test canceling navigation UI action.
        """
        nav_query = "hall_kitchen_bathroom"
        window = self.app.main_window
        robot = window.get_current_robot()

        window.goal_textbox.setText(nav_query)
        window.on_navigate_click()

        while not robot.executing_nav:
            time.sleep(0.2)
        if robot.executing_nav:
            time.sleep(0.2)
            window.on_cancel_action_click()

        # State should be reset after canceling is complete.
        assert not robot.canceling_execution
