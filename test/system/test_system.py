#!/usr/bin/env python3

"""
System-level tests for the pyrobosim UI functionality to execute tasks.
"""

import numpy as np
import os
import pytest
import sys
import threading
import time

from pyrobosim.core import Robot, World
from pyrobosim.gui import PyRoboSimGUI
from pyrobosim.manipulation import GraspGenerator, ParallelGraspProperties
from pyrobosim.navigation import ConstantVelocityExecutor, PathPlanner
from pyrobosim.utils.general import get_data_folder
from pyrobosim.utils.knowledge import query_to_entity
from pyrobosim.utils.pose import Pose


# Needed for PyQt5 tests to work with CI
os.environ["QT_QPA_PLATFORM"] = "offscreen"


class TestSystem:
    mutex = threading.Lock()

    @pytest.fixture(autouse=True)
    def create_world_and_app(self):
        world = World()
        data_folder = get_data_folder()

        # Set the location and object metadata
        world.set_metadata(
            locations=os.path.join(data_folder, "example_location_data.yaml"),
            objects=os.path.join(data_folder, "example_object_data.yaml"),
        )

        # Add rooms
        r1coords = [(-1, -1), (1.5, -1), (1.5, 1.5), (0.5, 1.5)]
        world.add_room(
            name="kitchen",
            footprint=r1coords,
            color=[1, 0, 0],
            nav_poses=[Pose(x=0.75, y=0.75, z=0.0, yaw=0.0)],
        )
        r2coords = [(1.75, 2.5), (3.5, 2.5), (3.5, 4), (1.75, 4)]
        world.add_room(name="bedroom", footprint=r2coords, color=[0, 0.6, 0])
        r3coords = [(-1, 1), (-1, 3.5), (-3.0, 3.5), (-2.5, 1)]
        world.add_room(name="bathroom", footprint=r3coords, color=[0, 0, 0.6])

        # Add hallways between the rooms
        world.add_hallway(room_start="kitchen", room_end="bathroom", width=0.7)
        world.add_hallway(
            room_start="bathroom",
            room_end="bedroom",
            width=0.5,
            conn_method="angle",
            conn_angle=0,
            offset=0.8,
        )
        world.add_hallway(
            room_start="kitchen",
            room_end="bedroom",
            width=0.6,
            conn_method="points",
            conn_points=[(1.0, 0.5), (2.5, 0.5), (2.5, 3.0)],
        )

        # Add locations
        table = world.add_location(
            category="table",
            parent="kitchen",
            pose=Pose(x=0.85, y=-0.5, z=0.0, yaw=-np.pi / 2.0),
        )
        desk = world.add_location(
            category="desk", parent="bedroom", pose=Pose(x=3.15, y=3.65, z=0.0, yaw=0.0)
        )
        counter = world.add_location(
            category="counter",
            parent="bathroom",
            pose=Pose(x=-2.45, y=2.5, z=0.0, q=[0.634411, 0.0, 0.0, 0.7729959]),
        )

        # Add objects
        world.add_object(
            category="banana",
            parent=table,
            pose=Pose(x=1.0, y=-0.5, z=0.0, q=[0.9238811, 0.0, 0.0, 0.3826797]),
        )
        world.add_object(
            category="apple", parent=desk, pose=Pose(x=3.2, y=3.5, z=0.0, yaw=0.0)
        )
        world.add_object(category="apple", parent=table)
        world.add_object(category="apple", parent=table)
        world.add_object(category="water", parent="counter0_left")
        world.add_object(category="banana", parent="counter0_right")
        world.add_object(category="water", parent=desk)

        # Add robot
        grasp_props = ParallelGraspProperties(
            max_width=0.175,
            depth=0.1,
            height=0.04,
            width_clearance=0.01,
            depth_clearance=0.01,
        )

        robot0 = Robot(
            name="robot0",
            radius=0.1,
            path_executor=ConstantVelocityExecutor(linear_velocity=3.0),
            grasp_generator=GraspGenerator(grasp_props),
        )
        planner_config_rrt = {
            "world": world,
            "bidirectional": True,
            "rrt_connect": False,
            "rrt_star": True,
            "compress_path": False,
        }
        rrt_planner = PathPlanner("rrt", **planner_config_rrt)
        robot0.set_path_planner(rrt_planner)
        world.add_robot(robot0, loc="kitchen")

        # Create headless app.
        self.app = PyRoboSimGUI(world, sys.argv, show=False)
        time.sleep(0.5)

    def nav_helper(self, nav_query):
        """
        Helper function to test navigation UI action.

        :param nav_query: Query for navigation goal.
        :type nav_query: str
        """
        self.mutex.acquire()
        try:
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
        finally:
            self.mutex.release()

        assert (
            robot.location == expected_location
            or robot.location in expected_location.children
        )

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

    @pytest.mark.dependency(depends=["TestSystem::test_nav"])
    def test_pick_place(self):
        """
        Test pick and place UI actions.
        """
        pick_place_queries = [
            ("table", "apple2", "table"),  # Pick and place in same location
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
            self.mutex.acquire()
            try:
                window.goal_textbox.setText(obj_query)
                window.on_pick_click()
            finally:
                self.mutex.release()
            assert robot.manipulated_object == expected_object

            # Navigate to place location
            self.nav_helper(place_query)

            # Place an object
            self.mutex.acquire()
            try:
                window.on_place_click()
            finally:
                self.mutex.release()
            assert robot.manipulated_object is None
