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

Figure updates are not written to the graph directly: they go to the "figbuf"
store and a clientside callback forwards them to the graph. While the user is
interacting, the full update is stashed and applied when the gesture ends;
during mouse drags the trace data is still applied via ``Plotly.restyle`` so
the world keeps animating under the pan. A full re-render mid-drag breaks
Plotly's drag machinery: Plotly only commits new axis ranges when the gesture
ends, so a re-render snaps the view back to the stale committed ranges and
(with a 10 Hz stream) leaves the drag half-dead afterwards. During wheel-zoom
bursts even a restyle flickers (scroll zoom previews via layer transforms and
defers its redraw), so those hold updates entirely. The gate must live
client-side -- with a server-side "is dragging" flag there is always one
already-in-flight update that lands right after the gesture starts, and one is
enough to break it.
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

# Shared UI font, so the status text and other elements match the buttons
# instead of falling back to the browser's serif default.
_FONT_FAMILY = (
    "system-ui, -apple-system, 'Segoe UI', Roboto, Helvetica, Arial, sans-serif"
)

_BUTTON_STYLE = {
    "flex": "1",
    "margin": "2px",
    "padding": "6px",
    "fontFamily": _FONT_FAMILY,
}

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
    """
    Creates a styled action button.

    :param button_id: The Dash component ID for the button.
    :param label: The text shown on the button.
    :return: The button component.
    """
    return html.Button(label, id=button_id, n_clicks=0, style=_BUTTON_STYLE)


def _row(children: list[Any], extra_style: dict[str, Any] | None = None) -> html.Div:
    """
    Lays out children in a horizontal flex row.

    :param children: The components to lay out.
    :param extra_style: Additional CSS style entries for the row, if any.
    :return: The row component.
    """
    style: dict[str, Any] = {"display": "flex", "width": "100%"}
    if extra_style:
        style.update(extra_style)
    return html.Div(children, style=style)


def _visibility_flags(visibility: list[str]) -> dict[str, bool]:
    """
    Maps the visibility checklist values to ``make_figure`` keyword flags.

    :param visibility: The checked visibility values, e.g. ``["rooms", "objects"]``.
    :return: The corresponding keyword arguments for :func:`.figure.make_figure`.
    """
    return {
        "show_room_names": "rooms" in visibility,
        "show_location_names": "locations" in visibility,
        "show_object_names": "objects" in visibility,
        "show_robot_names": "robots" in visibility,
        "show_collision_polygons": "collision" in visibility,
    }


def _layout(world: World) -> html.Div:
    """
    Builds the application layout for a world.

    :param world: The world to build the layout for.
    :return: The root component of the application layout.
    """
    robot_names = world.get_robot_names() + ["world"]
    default_robot = robot_names[0]
    default_visibility = ["rooms", "locations", "objects", "robots"]
    initial_figure = figure.make_figure(
        world,
        selected_robot=commands.resolve_robot(world, default_robot),
        **_visibility_flags(default_visibility),
    )

    return html.Div(
        style={
            "display": "flex",
            "flexDirection": "column",
            "height": "100vh",
            "boxSizing": "border-box",
            "paddingBottom": "10px",
            "fontFamily": _FONT_FAMILY,
        },
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
                ],
                # Lift this row (and the dropdown's open menu) above the world
                # graph below, which otherwise paints over the lower options and
                # hides robot names when the menu drops down.
                {"position": "relative", "zIndex": 10},
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
                figure=initial_figure,
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
                ],
                {"alignItems": "center", "flexWrap": "wrap"},
            ),
            dcc.Interval(id="tick", interval=TICK_MS, n_intervals=0),
            # Buffer for figure updates; a clientside callback forwards its
            # contents to the graph when no pan/zoom gesture is in progress.
            # It must always hold a complete figure: smooth-motion frames are
            # ``dash.Patch`` outputs, which the renderer applies against this
            # store's current value (patching an empty store throws).
            dcc.Store(id="figbuf", data=initial_figure),
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
    gui = HeadlessGui()
    world.gui = gui  # type: ignore[assignment]

    # Snapshot each robot's current planner path/graphs into the display stash,
    # so plans made before the app started (e.g. the planner demos) still show.
    for robot in world.robots:
        planner = robot.path_planner
        gui.canvas.show_planner_and_path_signal.emit(
            robot, True, planner.get_latest_path() if planner else None
        )

    app.layout = _layout(world)

    # Forward buffered figure updates to the graph, unless the user is
    # interacting: during mouse drags (``gd._dragging``), apply only the trace
    # data and stash the full update for drag end; during wheel-zoom bursts
    # (recent wheel / ``plotly_relayouting`` event), stash and apply nothing.
    # ``plotly_relayout`` fires once a gesture commits.
    app.clientside_callback(
        """
        function(fig) {
            const gd = document.querySelector('#world-graph .js-plotly-plot');
            if (!gd) {
                return fig;
            }
            if (!gd._pyrobosimHooked && gd.on) {
                gd._pyrobosimHooked = true;
                const bump = () => { window._pyrobosimGestureTs = Date.now(); };
                gd.on('plotly_relayouting', bump);
                gd.addEventListener('wheel', bump, {passive: true});
                gd.on('plotly_relayout', () => {
                    window._pyrobosimGestureTs = 0;
                    const pending = window._pyrobosimPendingFig;
                    if (pending) {
                        window._pyrobosimPendingFig = null;
                        setTimeout(() => window.dash_clientside.set_props(
                            'world-graph', {figure: pending}), 0);
                    }
                });
            }
            if (gd._dragging) {
                window._pyrobosimPendingFig = fig;
                // Keep the world animating under the drag: trace data does
                // not touch the layout or gesture state. Skip if the trace
                // count changed (structural change); it applies on drag end.
                if (gd.data && gd.data.length === fig.data.length) {
                    window.Plotly.restyle(gd, {
                        x: fig.data.map(t => t.x),
                        y: fig.data.map(t => t.y),
                    });
                }
                return window.dash_clientside.no_update;
            }
            const sinceGesture = Date.now() - (window._pyrobosimGestureTs || 0);
            if (sinceGesture < 400) {
                // Wheel-zoom burst: hold everything. Scroll zoom previews via
                // layer transforms and only redraws ~270 ms after the last
                // wheel event, so both react and restyle flicker against the
                // preview. The commit fires plotly_relayout, which releases
                // the stashed update right away.
                window._pyrobosimPendingFig = fig;
                return window.dash_clientside.no_update;
            }
            window._pyrobosimPendingFig = null;
            return fig;
        }
        """,
        Output("world-graph", "figure"),
        Input("figbuf", "data"),
    )

    # Shared refresh state, mutated by the dispatch and engine callbacks.
    # ``force`` marks that a command just ran, so the next frame must fully
    # rebuild the figure; effects landing later on background threads are
    # caught by the change counter.
    refresh: dict[str, Any] = {
        "force": False,
        "prev_active": False,
        "count": -1,
        "interval": TICK_MS,
        "resetting": False,
    }

    @app.callback(
        Output("figbuf", "data"),
        Output("status", "children"),
        *[Output(action, "disabled") for action in _TOGGLEABLE],
        Output("tick", "interval"),
        Input("tick", "n_intervals"),
        Input("robot-select", "value"),
        Input("visibility", "value"),
    )
    def _engine(_n: int, robot_name: str, visibility: list[str]) -> list[Any]:
        """
        Refreshes the view each tick (see the module docstring for the policy).

        :param _n: The tick counter (unused; the tick itself is the trigger).
        :param robot_name: The currently selected robot name.
        :param visibility: The checked visibility values.
        :return: The figure update, status text, button disabled states, and
            tick interval, matching the callback outputs.
        """
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
        change_count = gui.change_count

        structural = (
            ctx.triggered_id in ("robot-select", "visibility")
            or refresh["force"]
            or change_count != refresh["count"]
        )
        if structural:
            fig_out: Any = figure.make_figure(
                world, selected_robot=selected, **_visibility_flags(visibility)
            )
            refresh["force"] = False
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
        target = TICK_MS if (active or refresh["force"]) else IDLE_TICK_MS
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
            for action in ["reset-world", *WORLD_ACTIONS, *GOAL_ACTIONS]
        ],
        State("robot-select", "value"),
        State("goal-input", "value"),
        prevent_initial_call=True,
    )
    def _dispatch(*_clicks: int) -> tuple[Any, Any]:
        """
        Runs the command for whichever action button was clicked.

        :param _clicks: The button click counters (unused; the triggering
            button is read from the callback context instead).
        :return: The new goal text and tick interval, matching the callback
            outputs.
        """
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
                    """Reloads the world, then unblocks and refreshes the view."""
                    try:
                        world.reset()
                    finally:
                        refresh["resetting"] = False
                        refresh["force"] = True

                threading.Thread(target=_do_reset, daemon=True).start()
            refresh["interval"] = TICK_MS
            return (no_update, TICK_MS)

        if action in WORLD_ACTIONS:
            WORLD_ACTIONS[action](world, robot_name, goal)
            refresh["force"] = True
            # Wake the engine immediately even if it was idling slowly.
            refresh["interval"] = TICK_MS
            return (no_update, TICK_MS)

        return (no_update, no_update)

    return app


def _button_disabled_states(world: World, robot_name: str) -> list[bool]:
    """
    Computes the disabled state of each toggleable button, mirroring the GUI's
    ``update_button_state``.

    :param world: The world containing the robot.
    :param robot_name: The selected robot name, or ``"world"`` for no robot.
    :return: The disabled states, in the order of ``_TOGGLEABLE``.
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
    try:
        app.run(host=host, port=port, debug=debug)
    finally:
        # The server returns on Ctrl+C, but non-daemon threads (e.g., sensors)
        # would keep the interpreter hanging in threading shutdown without this.
        world.shutdown()
