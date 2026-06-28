"""
Dash web application for viewing and driving a PyRoboSim world in a browser.

This reproduces the matplotlib/PySide6 GUI's affordances: a robot selector and
goal query, navigate/pick/place/detect/open/close actions, randomize/reset/
cancel controls, and name/collision-visibility toggles. The world view animates
live.

A single "engine" callback drives the view each tick. To keep things responsive
and light on network traffic it:

* returns a lightweight ``dash.Patch`` (only moving traces) during smooth motion,
* rebuilds the full figure only on structural changes (commands, planner-graph
  changes, selection/visibility changes),
* leaves the figure untouched when the world is idle, and
* slows the refresh timer when idle so an untouched scene isn't polled at 10 Hz.
"""

import logging
import threading
from typing import Any

from dash import Dash, Input, Output, State, ctx, dcc, html, no_update

from . import commands, figure
from .commands import GOAL_ACTIONS, WORLD_ACTIONS
from .headless import HeadlessGui
from ..core.world import World

# Refresh period (ms) while the world is active, and the slower idle period.
TICK_MS = 100
IDLE_TICK_MS = 750

# Number of frames to keep fully refreshing after an async command, so discrete
# state changes made on background threads (e.g. pick/place) are captured.
FORCE_FRAMES = 6

_BUTTON_STYLE = {"flex": "1", "margin": "2px", "padding": "6px"}

# Action buttons whose disabled state is driven by robot state, in output order.
_TOGGLEABLE = [
    "navigate",
    "pick",
    "place",
    "detect",
    "open",
    "close",
    "cancel",
    "reset-world",
    "reset-planner",
    "rand-pose",
]


def _button(button_id: str, label: str) -> html.Button:
    """Creates a styled action button."""
    return html.Button(label, id=button_id, n_clicks=0, style=_BUTTON_STYLE)


def _row(children: list[Any]) -> html.Div:
    """Lays out children in a horizontal flex row."""
    return html.Div(children, style={"display": "flex", "width": "100%"})


def _visibility_flags(visibility: list[str]) -> dict[str, bool]:
    """Maps the visibility checklist values to make_figure keyword flags."""
    return {
        "show_room_names": "rooms" in visibility,
        "show_location_names": "locations" in visibility,
        "show_object_names": "objects" in visibility,
        "show_robot_names": "robots" in visibility,
        "show_collision_polygons": "collision" in visibility,
    }


def _layout(world: World) -> html.Div:
    """Builds the application layout for a world."""
    robot_names = world.get_robot_names() + ["world"]
    default_robot = robot_names[0] if robot_names else "world"
    default_visibility = ["rooms", "locations", "objects", "robots"]

    return html.Div(
        style={"display": "flex", "flexDirection": "column", "height": "100vh"},
        children=[
            _row(
                [
                    _button("rand-pose", "Randomize robot pose"),
                    _button("rand-goal", "Randomize nav goal"),
                    _button("rand-obj", "Randomize target object"),
                ]
            ),
            _row(
                [
                    html.B(
                        "Robot name:", style={"alignSelf": "center", "margin": "4px"}
                    ),
                    dcc.Dropdown(
                        id="robot-select",
                        options=[{"label": n, "value": n} for n in robot_names],
                        value=default_robot,
                        clearable=False,
                        style={"flex": "2", "margin": "2px"},
                    ),
                    html.B(
                        "Goal query:", style={"alignSelf": "center", "margin": "4px"}
                    ),
                    dcc.Input(
                        id="goal-input",
                        type="text",
                        value="",
                        style={"flex": "4", "margin": "2px"},
                    ),
                ]
            ),
            _row(
                [
                    _button("navigate", "Navigate"),
                    _button("pick", "Pick"),
                    _button("place", "Place"),
                ]
            ),
            _row(
                [
                    _button("detect", "Detect"),
                    _button("open", "Open"),
                    _button("close", "Close"),
                ]
            ),
            html.Div(
                id="status",
                style={
                    "textAlign": "center",
                    "fontWeight": "bold",
                    "whiteSpace": "pre-line",
                    "minHeight": "2.6em",
                },
            ),
            dcc.Graph(
                id="world-graph",
                figure=figure.make_figure(
                    world,
                    selected_robot=commands.resolve_robot(world, default_robot),
                    **_visibility_flags(default_visibility),
                ),
                style={"flex": "1"},
                config={"scrollZoom": True, "displaylogo": False},
            ),
            _row(
                [
                    dcc.Checklist(
                        id="visibility",
                        options=[
                            {"label": "Room names", "value": "rooms"},
                            {"label": "Location names", "value": "locations"},
                            {"label": "Object names", "value": "objects"},
                            {"label": "Robot names", "value": "robots"},
                            {"label": "Collision polygons", "value": "collision"},
                        ],
                        value=default_visibility,
                        inline=True,
                        style={"flex": "2", "alignSelf": "center", "margin": "4px"},
                    ),
                    _button("reset-world", "Reset world"),
                    _button("reset-planner", "Reset path planner"),
                    _button("cancel", "Cancel action"),
                ]
            ),
            dcc.Interval(id="tick", interval=TICK_MS, n_intervals=0),
        ],
    )


def create_app(world: World, title: str = "PyRoboSim") -> Dash:
    """
    Creates the interactive Dash application for a world.

    :param world: The world to render and drive.
    :param title: Browser tab title for the application.
    :return: A configured Dash application (not yet running).
    """
    # ``update_title=None`` stops Dash from flashing "Updating..." in the browser
    # tab on every callback (the timer fires constantly).
    app = Dash(__name__, title=title, update_title=None)  # type: ignore[arg-type]

    # Attach a no-op GUI so core's GUI-refresh hooks are satisfied and examples
    # that wait for ``world.gui`` to be set proceed in web mode. Its change
    # counter lets the engine cheaply detect discrete world changes.
    world.gui = HeadlessGui()  # type: ignore[assignment]

    app.layout = _layout(world)

    # Shared refresh state, mutated by the dispatch and engine callbacks.
    refresh: dict[str, Any] = {
        "force": 0,
        "prev_active": False,
        "need_full": False,
        "count": -1,
        "interval": TICK_MS,
        "resetting": False,
    }

    @app.callback(
        Output("world-graph", "figure"),
        Output("status", "children"),
        *[Output(action, "disabled") for action in _TOGGLEABLE],
        Output("tick", "interval"),
        Input("tick", "n_intervals"),
        Input("robot-select", "value"),
        Input("visibility", "value"),
    )
    def _engine(_n: int, robot_name: str, visibility: list[str]) -> list[Any]:
        # While a reset is reloading the world on a background thread, leave
        # everything untouched rather than reading a half-rebuilt world.
        if refresh["resetting"]:
            return [no_update] * (3 + len(_TOGGLEABLE))

        visibility = visibility or []
        selected = commands.resolve_robot(world, robot_name)
        active = any(robot.is_moving() for robot in world.robots)
        # The stub GUI bumps this counter on every structural change (pick/place/
        # open/close/detect/plan), including ones triggered outside the web
        # callbacks (e.g. task-plan execution), so they show without polling.
        gui = world.gui
        change_count = gui.change_count if isinstance(gui, HeadlessGui) else 0

        structural = (
            ctx.triggered_id in ("robot-select", "visibility")
            or refresh["need_full"]
            or refresh["force"] > 0
            or change_count != refresh["count"]
        )
        if structural:
            fig_out: Any = figure.make_figure(
                world, selected_robot=selected, **_visibility_flags(visibility)
            )
            refresh["need_full"] = False
            if refresh["force"] > 0:
                refresh["force"] -= 1
        elif active or refresh["prev_active"]:
            # Smooth motion: update only the moving traces.
            fig_out = figure.dynamic_patch(world, selected)
        else:
            # Idle: leave the figure untouched so pan/zoom stays smooth.
            fig_out = no_update

        refresh["prev_active"] = active
        refresh["count"] = change_count

        # Poll fast while busy, slowly when idle. Only emit a new interval value
        # when it changes, to avoid restarting the timer every tick.
        target = TICK_MS if (active or refresh["force"] > 0) else IDLE_TICK_MS
        interval_out: Any = no_update
        if target != refresh["interval"]:
            refresh["interval"] = target
            interval_out = target

        disabled = _button_disabled_states(world, robot_name)
        return [fig_out, figure.status_text(selected), *disabled, interval_out]

    @app.callback(
        Output("goal-input", "value"),
        Output("tick", "interval", allow_duplicate=True),
        [
            Input(action, "n_clicks")
            for action in list(WORLD_ACTIONS) + list(GOAL_ACTIONS)
        ],
        State("robot-select", "value"),
        State("goal-input", "value"),
        prevent_initial_call=True,
    )
    def _dispatch(*_clicks: int) -> tuple[Any, Any]:
        robot_name = ctx.states["robot-select.value"]
        goal = ctx.states["goal-input.value"] or ""
        action = ctx.triggered_id

        if action in GOAL_ACTIONS:
            new_goal = GOAL_ACTIONS[action](world, robot_name, goal)
            return (new_goal if new_goal is not None else no_update, no_update)

        if action == "reset-world":
            # Reloading the world from YAML is slow (~0.5 s); run it off-thread
            # so the UI stays responsive, and refresh once it completes.
            if not refresh["resetting"]:
                refresh["resetting"] = True

                def _do_reset() -> None:
                    try:
                        world.reset()
                    finally:
                        refresh["resetting"] = False
                        refresh["need_full"] = True

                threading.Thread(target=_do_reset, daemon=True).start()
            refresh["interval"] = TICK_MS
            return (no_update, TICK_MS)

        if action in WORLD_ACTIONS:
            WORLD_ACTIONS[action](world, robot_name, goal)
            refresh["need_full"] = True
            if action in commands.ASYNC_ACTIONS:
                refresh["force"] = FORCE_FRAMES
            # Wake the engine immediately even if it was idling slowly.
            refresh["interval"] = TICK_MS
            return (no_update, TICK_MS)

        return (no_update, no_update)

    return app


def _button_disabled_states(world: World, robot_name: str) -> list[bool]:
    """
    Computes the disabled state of each toggleable button, mirroring the GUI's
    ``update_button_state``. Returns values in the order of ``_TOGGLEABLE``.
    """
    robot = commands.resolve_robot(world, robot_name)
    if robot is None:
        # "world" selected: only open/close and reset-world are meaningful.
        disabled = {action: True for action in _TOGGLEABLE}
        disabled["open"] = False
        disabled["close"] = False
        disabled["reset-world"] = False
        return [disabled[action] for action in _TOGGLEABLE]

    is_moving = robot.is_moving()
    location = robot.location
    is_location_open = (
        location is not None and not isinstance(location, str) and location.is_open
    )
    at_open_spawn = robot.at_object_spawn() and is_location_open
    can_pick = robot.manipulated_object is None
    can_open_close = robot.at_openable_location() and can_pick

    states = {
        "navigate": is_moving,
        "pick": not (can_pick and at_open_spawn),
        "place": not ((not can_pick) and at_open_spawn),
        "detect": not at_open_spawn,
        "open": not (can_open_close and not is_location_open),
        "close": not (can_open_close and is_location_open),
        "cancel": not is_moving,
        "reset-world": is_moving,
        "reset-planner": is_moving,
        "rand-pose": is_moving,
    }
    return [states[action] for action in _TOGGLEABLE]


def run(
    world: World,
    host: str = "127.0.0.1",
    port: int = 8050,
    debug: bool = False,
    title: str = "PyRoboSim",
) -> None:
    """
    Builds and runs the interactive Dash web application for a world.

    :param world: The world to render and drive.
    :param host: Host interface to bind to.
    :param port: Port to serve on.
    :param debug: If True, runs Dash in debug mode with auto-reloading.
    :param title: Browser tab title for the application.
    """
    # Silence the per-request access log ("POST /_dash-update-component"), which
    # is noisy given the refresh timer fires continuously.
    logging.getLogger("werkzeug").setLevel(logging.ERROR)
    app = create_app(world, title=title)
    app.run(host=host, port=port, debug=debug)
