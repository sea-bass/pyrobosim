"""
Dash web application for viewing and driving a PyRoboSim world in a browser.

The controls mirror the Qt GUI: a robot selector and goal query, action
buttons, and visibility toggles, with a live world view. The app assumes a
single local user and a single server process, since the world model and
refresh state live in server memory.

A single tick-driven "engine" callback refreshes the view. It rebuilds the
full figure after discrete world changes, sends a lightweight ``dash.Patch``
of just the moving traces during smooth motion, and otherwise leaves the
figure untouched, so idle ticks are nearly free.

Figure updates are not written to the graph directly: they go to the "figbuf"
store, and a clientside callback forwards them to the graph only when no
pan/zoom gesture is in progress, since a full figure update mid-gesture breaks
Plotly's pan/zoom handling. The gate lives client-side so that updates already
in flight when a gesture starts are held as well.
"""

import logging
from dataclasses import dataclass
from typing import Any

from dash import Dash, Input, Output, State, ctx, dcc, html, no_update
from dash.exceptions import PreventUpdate

from . import commands, figure
from .headless import HeadlessGui
from ..core.robot import Robot
from ..core.world import World

# Refresh period (ms). The tick stays fixed and fast: idle ticks return
# ``no_update`` (so they are nearly free), and world changes made outside the
# web callbacks (e.g., task plans) show within one tick of starting.
TICK_MS = 100

# Shared UI font, so all elements match instead of falling back to the
# browser's serif default.
_FONT_FAMILY = (
    "system-ui, -apple-system, 'Segoe UI', Roboto, Helvetica, Arial, sans-serif"
)

_BUTTON_STYLE = {
    "flex": "1",
    "margin": "2px",
    "padding": "6px",
    "fontFamily": _FONT_FAMILY,
}

# Buttons that run a world/robot action and whose disabled state is managed.
_ACTION_BUTTONS = (
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
)

# Visibility checklist labels, keyed by the make_figure keyword they control.
_VISIBILITY_OPTIONS = {
    "show_room_names": "Room names",
    "show_location_names": "Location names",
    "show_object_names": "Object names",
    "show_robot_names": "Robot names",
    "show_collision_polygons": "Collision polygons",
}
_DEFAULT_VISIBILITY = [
    flag for flag in _VISIBILITY_OPTIONS if flag != "show_collision_polygons"
]


def _visibility_kwargs(checked: list[str]) -> dict[str, bool]:
    """
    Maps the checked visibility values to ``make_figure`` keyword flags.

    :param checked: The checked visibility checklist values.
    :return: The corresponding keyword arguments for :func:`.figure.make_figure`.
    """
    return {flag: flag in checked for flag in _VISIBILITY_OPTIONS}


@dataclass
class _RefreshState:
    """View-refresh bookkeeping shared by the engine and dispatch callbacks."""

    #: A command just ran, so the next frame must fully rebuild the figure.
    force: bool = False
    #: Whether any robot was moving on the previous engine tick.
    prev_active: bool = False
    #: The world change count at the last engine tick.
    change_count: int = -1
    #: The world is reloading on a background thread and must not be read.
    resetting: bool = False


def status_text(robot: Robot | None) -> str:
    """
    Builds the selected robot's status string (battery / location / holding).

    It is shown in an HTML element beside the graph rather than as the figure
    title, so per-frame updates do not relayout the plot.

    :param robot: The robot to describe, or None for an empty string.
    :return: The status text, formatted over two lines.
    """
    if robot is None:
        return ""
    bits = []
    if robot.location is not None:
        loc = robot.location if isinstance(robot.location, str) else robot.location.name
        bits.append(f"Location: {loc}")
    if robot.manipulated_object is not None:
        bits.append(f"Holding: {robot.manipulated_object.name}")
    return f"[{robot.name}] Battery: {robot.battery_level:.2f}%\n{', '.join(bits)}"


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


def _disabled_states(robot: Robot | None) -> dict[str, bool]:
    """
    Computes the disabled state of each action button, keyed by button ID,
    mirroring the Qt GUI's ``update_button_state``.

    :param robot: The selected robot, or None if "world" is selected.
    :return: The disabled state of each button in ``_ACTION_BUTTONS``.
    """
    if robot is None:
        # With no robot, only open/close (by name) and reset-world make sense.
        enabled = ("open", "close", "reset-world")
        return {button: button not in enabled for button in _ACTION_BUTTONS}

    is_moving = robot.is_moving()
    location = robot.location
    is_location_open = (
        location is not None and not isinstance(location, str) and location.is_open
    )
    at_open_spawn = robot.at_object_spawn() and is_location_open
    can_pick = robot.manipulated_object is None
    can_open_close = robot.at_openable_location() and can_pick

    return {
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


def _layout(world: World) -> html.Div:
    """
    Builds the application layout for a world.

    :param world: The world to build the layout for.
    :return: The root component of the application layout.
    """
    robot_names = world.get_robot_names() + ["world"]
    default_robot = robot_names[0]
    initial_figure = figure.make_figure(
        world,
        selected_robot=commands.resolve_robot(world, default_robot),
        **_visibility_kwargs(_DEFAULT_VISIBILITY),
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
                # Lift this row above the graph so the dropdown's open menu is
                # not painted over.
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
                            {"label": label, "value": flag}
                            for flag, label in _VISIBILITY_OPTIONS.items()
                        ],
                        value=_DEFAULT_VISIBILITY,
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
            # Buffer for figure updates, forwarded to the graph by the
            # clientside callback. It must always hold a complete figure:
            # smooth-motion frames are ``dash.Patch`` objects, which Dash
            # applies against this store's current value.
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
    # ``update_title=None`` stops Dash from flashing "Updating..." in the
    # browser tab every time the refresh timer fires.
    app = Dash(__name__, title=title, update_title=None)  # type: ignore[arg-type]

    # Attach a no-op GUI so core's GUI hooks work in web mode and examples
    # that wait for ``world.gui`` to be set proceed.
    gui = HeadlessGui()
    world.gui = gui  # type: ignore[assignment]

    # Snapshot each robot's current planner path/graphs, so plans made before
    # the app started (e.g., by the planner demos) still show.
    for robot in world.robots:
        planner = robot.path_planner
        gui.canvas.show_planner_and_path_signal.emit(
            robot, True, planner.get_latest_path() if planner else None
        )

    # Layout as a function: Dash re-evaluates it on each page load, so a
    # reloaded page renders the world's current state instead of a stale
    # snapshot from app startup.
    app.layout = lambda: _layout(world)

    # Starting from the current change count means the first engine tick does
    # not rebuild the figure the layout just rendered.
    state = _RefreshState(change_count=gui.change_count)

    # Forward buffered figure updates to the graph, unless a pan/zoom gesture
    # is in progress; a full update mid-gesture breaks Plotly's drag and
    # scroll-zoom handling.
    app.clientside_callback(
        """
        function(fig) {
            const gd = document.querySelector('#world-graph .js-plotly-plot');
            if (!gd) {
                return fig;
            }
            if (!gd._pyrobosimHooked && gd.on) {
                // First run: track gestures. Wheel / plotly_relayouting mark
                // one in progress; plotly_relayout fires when it commits.
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
                // Mid-drag: stash the full update, but keep the world moving
                // by restyling trace data only, which does not disturb the
                // drag. Skip that if the trace count changed; the stashed
                // update lands on drag end.
                window._pyrobosimPendingFig = fig;
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
                // Wheel-zoom burst: hold updates entirely (even a restyle
                // flickers against the zoom preview) until the zoom commits
                // and plotly_relayout releases the stashed update.
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

    @app.callback(
        output={
            "figure": Output("figbuf", "data"),
            "status": Output("status", "children"),
            "disabled": {
                button: Output(button, "disabled") for button in _ACTION_BUTTONS
            },
        },
        inputs={"_ticks": Input("tick", "n_intervals")},
        state={
            "robot_name": State("robot-select", "value"),
            "visibility": State("visibility", "value"),
        },
    )
    def _engine(_ticks: int, robot_name: str, visibility: list[str]) -> dict[str, Any]:
        """
        Refreshes the view each tick (see the module docstring for the policy).

        :param _ticks: The tick counter (unused; the tick itself is the trigger).
        :param robot_name: The currently selected robot name.
        :param visibility: The checked visibility flags.
        :return: The figure update, status text, and button disabled states,
            keyed by output name.
        """
        # While a reset is reloading the world on a background thread, leave
        # everything untouched rather than reading a half-rebuilt world.
        if state.resetting:
            raise PreventUpdate

        selected = commands.resolve_robot(world, robot_name)
        active = any(robot.is_moving() for robot in world.robots)
        # The headless GUI bumps this counter on every structural change,
        # including ones made outside the web callbacks (e.g., task plans).
        change_count = gui.change_count

        if state.force or change_count != state.change_count:
            fig_out: Any = figure.make_figure(
                world, selected_robot=selected, **_visibility_kwargs(visibility or [])
            )
            state.force = False
        elif active or state.prev_active:
            # Smooth motion: update only the moving traces.
            fig_out = figure.dynamic_patch(world, selected)
        else:
            # Idle: leave the figure untouched so pan/zoom stays smooth.
            fig_out = no_update

        state.prev_active = active
        state.change_count = change_count

        return {
            "figure": fig_out,
            "status": status_text(selected),
            "disabled": _disabled_states(selected),
        }

    @app.callback(
        output=Output("figbuf", "data", allow_duplicate=True),
        inputs={
            "robot_name": Input("robot-select", "value"),
            "visibility": Input("visibility", "value"),
        },
        prevent_initial_call=True,
    )
    def _on_view_change(robot_name: str, visibility: list[str]) -> Any:
        """
        Rebuilds the figure when the robot selection or visibility change.

        This is a separate callback (rather than more inputs on the engine)
        so its rebuild cannot be superseded by an in-flight engine tick: the
        renderer only keeps the newest response per callback, and an idle
        tick's empty response would otherwise swallow the rebuild.

        :param robot_name: The newly selected robot name.
        :param visibility: The checked visibility flags.
        :return: The rebuilt figure.
        """
        if state.resetting:
            raise PreventUpdate
        selected = commands.resolve_robot(world, robot_name)
        return figure.make_figure(
            world, selected_robot=selected, **_visibility_kwargs(visibility or [])
        )

    @app.callback(
        Output("goal-input", "value"),
        Input("rand-goal", "n_clicks"),
        Input("rand-obj", "n_clicks"),
        prevent_initial_call=True,
    )
    def _randomize_goal(_goal_clicks: int, _obj_clicks: int) -> Any:
        """
        Fills the goal input with a random goal or object name.

        :param _goal_clicks: Click counter (unused; the click is the trigger).
        :param _obj_clicks: Click counter (unused; the click is the trigger).
        :return: The new goal text, or ``no_update`` if there was nothing to
            sample.
        """
        if ctx.triggered_id == "rand-goal":
            name = commands.random_goal(world)
        else:
            name = commands.random_object(world)
        return name if name is not None else no_update

    def _run_action(action: str, robot_name: str, goal: str) -> None:
        """
        Runs the world/robot command for an action button.

        :param action: The clicked button ID.
        :param robot_name: The selected robot name, or ``"world"`` for none.
        :param goal: The current goal query text.
        """
        robot = commands.resolve_robot(world, robot_name)
        if action == "open":
            commands.open_location(world, robot, goal)
        elif action == "close":
            commands.close_location(world, robot, goal)
        elif robot is None:
            return  # The remaining actions require a robot.
        elif action == "navigate":
            commands.navigate(robot, goal)
        elif action == "pick":
            commands.pick(robot, goal)
        elif action == "place":
            commands.place(robot)
        elif action == "detect":
            commands.detect(robot, goal)
        elif action == "rand-pose":
            commands.randomize_pose(world, robot)
        elif action == "reset-planner":
            commands.reset_path_planner(robot)
        elif action == "cancel":
            commands.cancel_action(robot)
        else:
            raise ValueError(f"Unhandled action button: {action}")

    def _reset_world() -> None:
        """Reloads the world, then unblocks and refreshes the view."""
        try:
            world.reset()
        finally:
            state.resetting = False
            state.force = True

    @app.callback(
        inputs={"_clicks": [Input(button, "n_clicks") for button in _ACTION_BUTTONS]},
        state={
            "robot_name": State("robot-select", "value"),
            "goal": State("goal-input", "value"),
        },
        prevent_initial_call=True,
    )
    def _dispatch(_clicks: list[int], robot_name: str, goal: str) -> None:
        """
        Runs the command for whichever action button was clicked. This callback
        has no outputs: commands only mutate the world, and the engine picks up
        the result on its next tick.

        :param _clicks: Click counters (unused; the triggering button is read
            from the callback context instead).
        :param robot_name: The selected robot name, or ``"world"`` for none.
        :param goal: The current goal query text.
        """
        action = ctx.triggered_id
        if action == "reset-world":
            # Reloading the world from YAML is slow; run it off-thread so the
            # UI stays responsive, and refresh once it completes.
            if not state.resetting:
                state.resetting = True
                commands.run_async(_reset_world)
        else:
            _run_action(action, robot_name, goal or "")
            state.force = True

    return app


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
    # Silence the per-request access log, which is noisy given the refresh
    # timer fires continuously.
    logging.getLogger("werkzeug").setLevel(logging.ERROR)
    app = create_app(world, title=title)
    try:
        app.run(host=host, port=port, debug=debug)
    finally:
        # The server returns on Ctrl+C, but non-daemon threads (e.g., sensors)
        # would keep the interpreter from exiting without this.
        world.shutdown()
