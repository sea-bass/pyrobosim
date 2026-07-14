#!/usr/bin/env python3

"""
System-level tests for the web UI functionality to execute tasks.

These mirror ``test/system/test_system.py``, but drive the Dash application
instead of the Qt GUI. Actions are exercised end-to-end through the app's
HTTP callback endpoint (via the Flask test client), the same route the
browser posts to, so no browser is needed. While actions execute, the engine
callback is invoked the way the browser's refresh timer would, and its
responses are checked.

The whole module is skipped if the optional web dependencies (plotly/dash)
are not installed.
"""

import pathlib
import time
from typing import Any, Callable, ClassVar

import pytest

pytest.importorskip("plotly")
pytest.importorskip("dash")

from dash import Dash
from flask.testing import FlaskClient

from pyrobosim.core import Robot, World, WorldYamlLoader
from pyrobosim.utils.knowledge import query_to_entity
from pyrobosim.web.app import _ACTION_BUTTONS, _DEFAULT_VISIBILITY, create_app

# Endpoint the Dash renderer posts callback invocations to.
_UPDATE_ROUTE = "/_dash-update-component"


class TestSystemWeb:
    app: ClassVar[Dash | None] = None
    client: ClassVar[FlaskClient]
    world: ClassVar[World]
    robot: ClassVar[Robot]
    engine_key: ClassVar[str]
    dispatch_key: ClassVar[str]
    num_ticks: ClassVar[int] = 0

    @pytest.fixture(autouse=True)  # type: ignore[misc]
    def setup_and_teardown(self) -> None:
        if TestSystemWeb.app is not None:
            return

        # Load the same world as the Qt GUI system tests.
        world_file_path = (
            pathlib.Path(__file__).parents[1] / "system" / "test_system_world.yaml"
        )
        TestSystemWeb.world = WorldYamlLoader().from_file(world_file_path)
        TestSystemWeb.robot = TestSystemWeb.world.robots[0]

        # Create the app and an HTTP client against its Flask server.
        TestSystemWeb.app = create_app(TestSystemWeb.world)
        TestSystemWeb.client = TestSystemWeb.app.server.test_client()
        assert TestSystemWeb.client.get("/").status_code == 200

        # Discover the identifiers Dash assigned to the callbacks: the engine
        # (the multi-output callback updating the status), and the action
        # dispatch (the no-output callback, keyed by a hash with no ".").
        keys = [key for key in TestSystemWeb.app.callback_map if key is not None]
        TestSystemWeb.engine_key = next(key for key in keys if "status.children" in key)
        TestSystemWeb.dispatch_key = next(key for key in keys if "." not in key)

    def post_callback(self, body: dict[str, Any]) -> dict[str, Any] | None:
        """
        Posts a callback invocation to the app, as the Dash renderer would.

        :param body: The callback request body.
        :return: The response's component updates, or None if the callback
            made no updates (HTTP 204).
        """
        response = self.client.post(_UPDATE_ROUTE, json=body)
        assert response.status_code in (200, 204)
        if response.status_code == 204:
            return None
        updates = response.get_json()["response"]
        assert isinstance(updates, dict)
        return updates

    def tick_engine(self) -> dict[str, Any] | None:
        """
        Invokes the engine callback once, like the browser's refresh timer.

        :return: The engine's component updates, or None if it made none.
        """
        TestSystemWeb.num_ticks += 1
        # Multi-output callbacks are keyed "..id.prop...id.prop..", and their
        # invocations must carry the same outputs as a list.
        output_spec = [
            {"id": part.rsplit(".", 1)[0], "property": part.rsplit(".", 1)[1]}
            for part in self.engine_key.strip(".").split("...")
        ]
        return self.post_callback(
            {
                "output": self.engine_key,
                "outputs": output_spec,
                "inputs": [
                    {
                        "id": "tick",
                        "property": "n_intervals",
                        "value": self.num_ticks,
                    }
                ],
                "changedPropIds": ["tick.n_intervals"],
                "state": [
                    {
                        "id": "robot-select",
                        "property": "value",
                        "value": self.robot.name,
                    },
                    {
                        "id": "visibility",
                        "property": "value",
                        "value": _DEFAULT_VISIBILITY,
                    },
                ],
            }
        )

    def click_action(self, action: str, goal: str = "") -> None:
        """
        Invokes the action dispatch callback, like clicking an action button.

        :param action: The action button ID, e.g. ``"navigate"``.
        :param goal: The goal query text to submit with the action.
        """
        assert action in _ACTION_BUTTONS
        self.post_callback(
            {
                "output": self.dispatch_key,
                "inputs": [
                    {
                        "id": button,
                        "property": "n_clicks",
                        "value": int(button == action),
                    }
                    for button in _ACTION_BUTTONS
                ],
                "changedPropIds": [f"{action}.n_clicks"],
                "state": [
                    {
                        "id": "robot-select",
                        "property": "value",
                        "value": self.robot.name,
                    },
                    {"id": "goal-input", "property": "value", "value": goal},
                ],
            }
        )

    def wait_until(self, condition: Callable[[], bool], timeout: float = 60.0) -> None:
        """
        Waits for a condition, ticking the engine like the browser would.

        :param condition: The condition to wait for.
        :param timeout: Time (in seconds) after which the test fails.
        """
        start_time = time.time()
        while not condition():
            if time.time() - start_time > timeout:
                pytest.fail(f"Timed out waiting for {condition}")
            self.tick_engine()
            time.sleep(0.1)

    def nav_helper(self, nav_query: str) -> None:
        """
        Helper function to test navigation UI action.

        :param nav_query: Query for navigation goal.
        """
        world = self.world
        robot = self.robot
        expected_location = query_to_entity(
            world,
            nav_query,
            mode="location",
            robot=robot,
            resolution_strategy="nearest",
        )

        self.click_action("navigate", goal=nav_query)

        self.wait_until(lambda: robot.executing_nav)
        # While navigating, the engine must keep the buttons in sync.
        updates = self.tick_engine()
        if robot.executing_nav and updates is not None:
            assert updates["navigate"]["disabled"]
            assert not updates["cancel"]["disabled"]
        self.wait_until(lambda: not robot.executing_nav)
        robot.location = world.get_location_from_pose(robot.get_pose())

        assert robot.last_nav_result.is_success()
        assert (
            robot.location == expected_location
            or robot.location in expected_location.children
        )

    @pytest.mark.dependency(name="test_nav_web")  # type: ignore[misc]
    def test_nav(self) -> None:
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

    @pytest.mark.dependency(name="test_pick_detect_place_web", depends=["test_nav_web"])  # type: ignore[misc]
    def test_pick_detect_place(self) -> None:
        """
        Test pick, detect, and place UI actions.
        """
        pick_place_queries = [
            ("table", "gala", "table"),  # Pick and place in same location
            ("counter0_left", "water", "desk"),  # Pick and place in different location
        ]

        world = self.world
        robot = self.robot

        for pick_query, obj_query, place_query in pick_place_queries:
            # Navigate to pick location
            self.nav_helper(pick_query)

            # Detect objects
            self.click_action("detect")
            self.wait_until(lambda: not robot.executing_action, timeout=10.0)

            # Pick an object
            expected_object = query_to_entity(
                world,
                obj_query.split(" "),
                mode="object",
                robot=robot,
                resolution_strategy="nearest",
            )
            self.click_action("pick", goal=obj_query)
            self.wait_until(lambda: robot.manipulated_object is not None, timeout=10.0)
            assert robot.manipulated_object == expected_object

            # The pick is a world change, so the next engine tick must send a
            # full figure rebuild (not a patch) to the browser.
            updates = self.tick_engine()
            assert updates is not None
            assert "layout" in updates["figbuf"]["data"]

            # Navigate to place location
            self.nav_helper(place_query)

            # Place an object
            self.click_action("place")
            self.wait_until(lambda: robot.manipulated_object is None, timeout=10.0)

    @pytest.mark.dependency(name="test_open_close_web", depends=["test_pick_detect_place_web"])  # type: ignore[misc]
    def test_open_close(self) -> None:
        """
        Test open and close UI actions.
        """
        location_queries = ["hall_kitchen_bathroom", "my_desk"]

        world = self.world

        for location_name in location_queries:
            # Navigate to the location
            location = world.get_entity_by_name(location_name)
            self.nav_helper(location.name)

            # Close the location and verify that it's closed.
            self.click_action("close")
            self.wait_until(lambda: not location.is_open, timeout=10.0)

            # Open the location and verify that it's open.
            self.click_action("open")
            self.wait_until(lambda: location.is_open, timeout=10.0)

    @pytest.mark.dependency(name="test_nav_cancel_web", depends=["test_open_close_web"])  # type: ignore[misc]
    def test_nav_cancel(self) -> None:
        """
        Test canceling navigation UI action.
        """
        nav_query = "hall_kitchen_bathroom"
        robot = self.robot

        self.click_action("navigate", goal=nav_query)

        self.wait_until(lambda: robot.executing_nav)
        if robot.executing_nav:
            time.sleep(0.2)
            self.click_action("cancel")

        # State should be reset after canceling is complete.
        self.wait_until(lambda: not robot.executing_nav, timeout=10.0)
        assert not robot.canceling_execution
