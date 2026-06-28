"""
Build a Plotly figure from a PyRoboSim world, mirroring the matplotlib GUI.

Rendering is split into two layers:

* **Static geometry** (rooms, hallways, locations, object spawns, resting
  objects, optional collision polygons) is drawn as ``layout.shapes``. These
  polygons often have interior holes (e.g. a room's wall ring), so they use SVG
  ``path`` shapes with an even-odd fill rule.
* **Dynamic content** (per robot: planner path, lidar rays, body, orientation
  line, and any held object) is drawn as Plotly traces at stable indices, so it
  can be updated each frame with :func:`dynamic_patch` instead of rebuilding the
  whole figure. The selected robot's planner graphs are drawn as traces too
  (efficient for many nodes/edges).
"""

import math
from typing import Any, Sequence

import plotly.graph_objects as go
from dash import Patch
from shapely.geometry import MultiPolygon, Polygon

from ..core.robot import Robot
from ..core.world import World
from ..utils.polygon import transform_polygon

# Multiplier of robot radius for the orientation line, matching the GUI.
ROBOT_DIR_LINE_FACTOR = 3.0
# Number of segments used to approximate a robot's circular body.
ROBOT_CIRCLE_SEGMENTS = 40
# Fraction of the world extent added as a margin around the view.
VIEW_MARGIN = 0.05
# Color used for collision-polygon overlays, matching the GUI's magenta.
COLLISION_COLOR = "rgb(255, 0, 255)"

# Number of dynamic traces emitted per robot (path, lidar, body, dir, held).
TRACES_PER_ROBOT = 5
_PATH, _LIDAR, _BODY, _DIR, _HELD = range(TRACES_PER_ROBOT)


def color_to_css(color: Sequence[float]) -> str:
    """
    Converts a PyRoboSim RGB color to a CSS ``rgb(...)`` string.

    :param color: An (R, G, B) sequence with each channel in the range (0.0, 1.0),
        as produced by :func:`pyrobosim.utils.general.parse_color`.
    :return: A CSS color string, e.g. ``"rgb(204, 0, 204)"``.
    """
    r, g, b = (int(round(255 * channel)) for channel in color)
    return f"rgb({r}, {g}, {b})"


# ---------------------------------------------------------------------------
# Geometry helpers
# ---------------------------------------------------------------------------
def _rgba(rgb_str: str, alpha: float) -> str:
    """Converts a CSS ``rgb(...)`` string into ``rgba(...)`` with the given alpha."""
    inner = rgb_str[rgb_str.index("(") + 1 : rgb_str.index(")")]
    return f"rgba({inner}, {alpha})"


def _ring_path(coords: Sequence[Sequence[float]]) -> str:
    """Builds an SVG sub-path string (``M ... L ... Z``) for one polygon ring."""
    if not coords:
        return ""
    commands = [
        f"{'M' if i == 0 else 'L'}{x:.5f},{y:.5f}" for i, (x, y) in enumerate(coords)
    ]
    return " ".join(commands) + " Z"


def _svg_path(geom: Polygon | MultiPolygon | None) -> str:
    """Converts a Shapely polygon (with holes / parts) to an SVG path string."""
    if geom is None or geom.is_empty:
        return ""
    polys = geom.geoms if isinstance(geom, MultiPolygon) else [geom]
    parts = []
    for poly in polys:
        parts.append(_ring_path(poly.exterior.coords))
        for interior in poly.interiors:
            parts.append(_ring_path(interior.coords))
    return " ".join(part for part in parts if part)


def _polygon_shape(
    geom: Polygon | MultiPolygon | None,
    line_color: str,
    *,
    fillcolor: str | None = None,
    width: float = 2.0,
    dash: str | None = None,
    layer: str = "below",
) -> dict[str, Any] | None:
    """Builds a Plotly ``path`` shape for a polygon, or None if it is empty."""
    path = _svg_path(geom)
    if not path:
        return None
    line: dict[str, Any] = {"color": line_color, "width": width}
    if dash is not None:
        line["dash"] = dash
    return {
        "type": "path",
        "path": path,
        "xref": "x",
        "yref": "y",
        "fillcolor": fillcolor or "rgba(0,0,0,0)",
        "fillrule": "evenodd",
        "line": line,
        "layer": layer,
    }


def _polygon_xy(
    geom: Polygon | MultiPolygon | None,
) -> tuple[list[float | None], list[float | None]]:
    """Returns None-separated x/y coordinate lists for a polygon's exterior ring(s)."""
    xs: list[float | None] = []
    ys: list[float | None] = []
    if geom is None or geom.is_empty:
        return xs, ys
    polys = geom.geoms if isinstance(geom, MultiPolygon) else [geom]
    for poly in polys:
        for x, y in poly.exterior.coords:
            xs.append(x)
            ys.append(y)
        xs.append(None)
        ys.append(None)
    return xs, ys


def _bounds(world: World) -> tuple[float, float, float, float]:
    """Returns the world's (xmin, ymin, xmax, ymax) extent, with a small margin."""
    if world.x_bounds is None or world.y_bounds is None:
        return (-1.0, -1.0, 1.0, 1.0)
    xmin, xmax = world.x_bounds
    ymin, ymax = world.y_bounds
    pad = VIEW_MARGIN * max(xmax - xmin, ymax - ymin)
    return (xmin - pad, ymin - pad, xmax + pad, ymax + pad)


def _label(x: float, y: float, text: str, color: str, size: int) -> dict[str, Any]:
    """Builds a Plotly annotation for an entity name."""
    return {
        "x": x,
        "y": y,
        "text": text,
        "showarrow": False,
        "font": {"color": color, "size": size},
        "xanchor": "center",
        "yanchor": "top",
    }


# ---------------------------------------------------------------------------
# Dynamic per-robot trace data
# ---------------------------------------------------------------------------
def _circle_xy(cx: float, cy: float, radius: float) -> tuple[list[float], list[float]]:
    """Returns the x/y points of a closed circle for a robot body."""
    angles = [
        2.0 * math.pi * i / ROBOT_CIRCLE_SEGMENTS
        for i in range(ROBOT_CIRCLE_SEGMENTS + 1)
    ]
    return (
        [cx + radius * math.cos(a) for a in angles],
        [cy + radius * math.sin(a) for a in angles],
    )


def _lidar_xy(robot: Robot) -> tuple[list[float | None], list[float | None]]:
    """Collects None-separated ray segments from a robot's active lidar sensors."""
    xs: list[float | None] = []
    ys: list[float | None] = []
    for sensor in robot.sensors.values():
        if not getattr(sensor, "is_active", False):
            continue
        for segment in getattr(sensor, "lidar_coords", []):
            for point in segment:
                xs.append(point[0])
                ys.append(point[1])
            xs.append(None)
            ys.append(None)
    return xs, ys


def _path_to_render(robot: Robot) -> Any:
    """
    Returns the path to display for a robot.

    A path handed to the GUI (via ``show_planner_and_path``) takes precedence:
    PDDLStream ``navigate`` actions carry their own path produced by a stream
    rather than planned at navigation time, so the path planner never records it
    and ``get_latest_path`` would miss it. Otherwise fall back to the planner's
    latest plan (the usual plan-on-the-spot case).
    """
    world = robot.world
    gui = getattr(world, "gui", None) if world is not None else None
    paths = getattr(getattr(gui, "canvas", None), "displayed_paths", None)
    if paths is not None and robot.name in paths:
        return paths[robot.name]
    if robot.path_planner is not None:
        return robot.path_planner.get_latest_path()
    return None


def _robot_dynamic_xy(robot: Robot) -> dict[int, tuple[list[Any], list[Any]]]:
    """
    Returns the x/y data for each of a robot's dynamic traces, keyed by the
    trace offset within the robot's block.
    """
    pose = robot.get_pose()

    # Path (planned on the spot or supplied by an action, see _path_to_render):
    # shown while navigating, or when a fresh path has not been traversed yet
    # (the robot is still at its start) -- e.g. the planner demos that plan but
    # never navigate. It clears once the robot has reached the goal (no longer
    # moving and no longer at the path start).
    path_x: list[Any] = []
    path_y: list[Any] = []
    path = _path_to_render(robot)
    if path is not None and path.num_poses > 1:
        at_start = pose.get_linear_distance(path.poses[0]) < 0.05
        if robot.is_moving() or at_start:
            path_x = [p.x for p in path.poses]
            path_y = [p.y for p in path.poses]

    body_x, body_y = _circle_xy(pose.x, pose.y, robot.radius)

    length = ROBOT_DIR_LINE_FACTOR * robot.radius
    yaw = pose.get_yaw()
    dir_x = [pose.x, pose.x + length * math.cos(yaw)]
    dir_y = [pose.y, pose.y + length * math.sin(yaw)]

    held_x: list[Any] = []
    held_y: list[Any] = []
    if robot.manipulated_object is not None:
        obj = robot.manipulated_object
        held_x, held_y = _polygon_xy(transform_polygon(obj.raw_polygon, obj.pose))

    lidar_x, lidar_y = _lidar_xy(robot)

    return {
        _PATH: (path_x, path_y),
        _LIDAR: (lidar_x, lidar_y),
        _BODY: (body_x, body_y),
        _DIR: (dir_x, dir_y),
        _HELD: (held_x, held_y),
    }


def _robot_traces(robot: Robot) -> list[go.Scatter]:
    """Builds the (styled) dynamic traces for a single robot, in block order."""
    color = color_to_css(robot.color)
    data = _robot_dynamic_xy(robot)
    held_color = (
        color_to_css(robot.manipulated_object.viz_color)
        if robot.manipulated_object is not None
        else color
    )
    common = {"hoverinfo": "skip", "showlegend": False}
    return [
        go.Scatter(
            x=data[_PATH][0],
            y=data[_PATH][1],
            mode="lines",
            line={"color": color, "width": 3},
            opacity=0.5,
            **common,
        ),
        go.Scatter(
            x=data[_LIDAR][0],
            y=data[_LIDAR][1],
            mode="lines",
            line={"color": color, "width": 0.5},
            opacity=0.5,
            **common,
        ),
        go.Scatter(
            x=data[_BODY][0],
            y=data[_BODY][1],
            mode="lines",
            line={"color": color, "width": 2},
            fill="toself",
            fillcolor="white",
            **common,
        ),
        go.Scatter(
            x=data[_DIR][0],
            y=data[_DIR][1],
            mode="lines",
            line={"color": color, "width": 2},
            **common,
        ),
        go.Scatter(
            x=data[_HELD][0],
            y=data[_HELD][1],
            mode="lines",
            line={"color": held_color, "width": 2},
            **common,
        ),
    ]


# ---------------------------------------------------------------------------
# Planner graphs (shown only for the selected robot)
# ---------------------------------------------------------------------------
def _graphs_visible(robot: Robot) -> bool:
    """
    Whether planner graphs should be shown for a robot.

    Graphs are only meaningful when the planner actually planned during the
    action; when a path is supplied ready-made (e.g. a PDDLStream ``navigate``
    action), ``show_planner_and_path`` was emitted with ``show_graphs=False`` and
    we suppress the graphs, showing just the path. Defaults to True when nothing
    has been recorded yet (the planner demos that plan on the spot).
    """
    world = robot.world
    gui = getattr(world, "gui", None) if world is not None else None
    visible = getattr(getattr(gui, "canvas", None), "graphs_visible", None)
    if visible is not None and robot.name in visible:
        return bool(visible[robot.name])
    return True


def _graphs_for(selected_robot: Robot | None) -> list[Any]:
    """Returns the selected robot's planner search graphs (empty if none)."""
    if selected_robot is None or selected_robot.path_planner is None:
        return []
    if not _graphs_visible(selected_robot):
        return []
    return list(selected_robot.path_planner.get_graphs())


def num_graph_traces(selected_robot: Robot | None) -> int:
    """Number of traces emitted for planner graphs (two per graph: edges, nodes)."""
    return 2 * len(_graphs_for(selected_robot))


def _graph_traces(selected_robot: Robot | None) -> list[go.Scatter]:
    """Builds edge and node traces for the selected robot's planner graphs."""
    traces: list[go.Scatter] = []
    for graph in _graphs_for(selected_robot):
        color = color_to_css(graph.color)
        alpha = graph.color_alpha

        edge_x: list[float | None] = []
        edge_y: list[float | None] = []
        for edge in graph.edges:
            edge_x += [edge.nodeA.pose.x, edge.nodeB.pose.x, None]
            edge_y += [edge.nodeA.pose.y, edge.nodeB.pose.y, None]
        traces.append(
            go.Scatter(
                x=edge_x,
                y=edge_y,
                mode="lines",
                line={"color": color, "width": 0.7, "dash": "dash"},
                opacity=alpha,
                hoverinfo="skip",
                showlegend=False,
            )
        )
        traces.append(
            go.Scatter(
                x=[n.pose.x for n in graph.nodes],
                y=[n.pose.y for n in graph.nodes],
                mode="markers",
                marker={"color": color, "size": 4},
                opacity=alpha,
                hoverinfo="skip",
                showlegend=False,
            )
        )
    return traces


# ---------------------------------------------------------------------------
# Static geometry
# ---------------------------------------------------------------------------
def _static_shapes_and_labels(
    world: World,
    selected_robot: Robot | None,
    show_room_names: bool,
    show_location_names: bool,
    show_object_names: bool,
    show_collision_polygons: bool,
) -> tuple[list[dict[str, Any]], list[dict[str, Any]]]:
    """Builds the static geometry shapes and name annotations for the world."""
    shapes: list[dict[str, Any]] = []
    annotations: list[dict[str, Any]] = []

    # Rooms: colored wall rings.
    for room in world.rooms:
        color = color_to_css(room.viz_color)
        shape = _polygon_shape(room.viz_polygon, color, fillcolor=_rgba(color, 0.85))
        if shape:
            shapes.append(shape)
        if show_room_names:
            annotations.append(
                _label(room.centroid[0], room.centroid[1], room.name, color, 12)
            )

    # Hallways: wall rings, plus a filled block when known to be closed.
    if selected_robot is not None:
        known_closed = set(selected_robot.get_known_closed_hallways())
    else:
        known_closed = {h for h in world.hallways if not h.is_open}
    for hall in world.hallways:
        color = color_to_css(hall.viz_color)
        shape = _polygon_shape(hall.viz_polygon, color, fillcolor=_rgba(color, 0.75))
        if shape:
            shapes.append(shape)
        if hall in known_closed:
            closed = _polygon_shape(
                hall.closed_polygon, color, fillcolor=_rgba(color, 0.5)
            )
            if closed:
                shapes.append(closed)

    # Locations: outline, filled if closed; object spawns as dashed outlines.
    for loc in world.locations:
        color = color_to_css(loc.viz_color)
        fill = None if loc.is_open else _rgba(color, 0.5)
        shape = _polygon_shape(loc.polygon, color, fillcolor=fill)
        if shape:
            shapes.append(shape)
        if show_location_names:
            annotations.append(_label(loc.pose.x, loc.pose.y, loc.name, color, 10))
        for spawn in loc.children:
            spawn_shape = _polygon_shape(spawn.polygon, color, width=1, dash="dash")
            if spawn_shape:
                shapes.append(spawn_shape)

    # Resting objects (held objects are drawn as dynamic robot traces instead).
    held = {r.manipulated_object for r in world.robots if r.manipulated_object}
    known_objects = (
        selected_robot.get_known_objects()
        if selected_robot is not None
        else world.objects
    )
    for obj in known_objects:
        if obj in held:
            continue
        color = color_to_css(obj.viz_color)
        polygon = transform_polygon(obj.raw_polygon, obj.pose)
        # Objects render above robots (GUI zorder 4 > 3).
        shape = _polygon_shape(polygon, color, layer="above")
        if shape:
            shapes.append(shape)
        if show_object_names:
            xmin, ymin, xmax, ymax = polygon.bounds
            annotations.append(
                _label(
                    obj.pose.x + (xmax - xmin),
                    obj.pose.y + (ymax - ymin),
                    obj.name,
                    color,
                    8,
                )
            )

    # Collision polygons (debug overlay).
    if show_collision_polygons:
        for entity in [*world.rooms, *world.hallways]:
            shape = _polygon_shape(
                entity.internal_collision_polygon,
                COLLISION_COLOR,
                fillcolor=_rgba(COLLISION_COLOR, 0.5),
            )
            if shape:
                shapes.append(shape)

    return shapes, annotations


# ---------------------------------------------------------------------------
# Public API
# ---------------------------------------------------------------------------
def status_text(robot: Robot | None) -> str:
    """
    Builds the selected robot's status string (battery / location / holding),
    as shown in the GUI title. It is rendered in an HTML element beside the
    graph rather than the figure title, so updating it every frame does not
    relayout the plot (which would disrupt an in-progress pan/zoom).
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


def make_figure(
    world: World,
    *,
    selected_robot: Robot | None = None,
    show_room_names: bool = True,
    show_location_names: bool = True,
    show_object_names: bool = True,
    show_robot_names: bool = True,
    show_collision_polygons: bool = False,
) -> go.Figure:
    """
    Builds a complete Plotly figure rendering a world, mirroring the GUI.

    Trace layout (used by :func:`dynamic_patch`): index 0 is an invisible bounds
    trace, followed by planner-graph traces, followed by ``TRACES_PER_ROBOT``
    traces for each robot in order.
    """
    shapes, annotations = _static_shapes_and_labels(
        world,
        selected_robot,
        show_room_names,
        show_location_names,
        show_object_names,
        show_collision_polygons,
    )

    if show_robot_names:
        for robot in world.robots:
            pose = robot.get_pose()
            annotations.append(
                _label(
                    pose.x,
                    pose.y - 2.0 * robot.radius,
                    robot.name,
                    color_to_css(robot.color),
                    10,
                )
            )

    xmin, ymin, xmax, ymax = _bounds(world)
    bounds_trace = go.Scatter(
        x=[xmin, xmax, xmin, xmax],
        y=[ymin, ymin, ymax, ymax],
        mode="markers",
        marker={"opacity": 0.0},
        hoverinfo="skip",
        showlegend=False,
    )

    traces: list[go.Scatter] = [bounds_trace]
    traces += _graph_traces(selected_robot)
    for robot in world.robots:
        traces += _robot_traces(robot)

    fig = go.Figure(data=traces)
    fig.update_layout(
        shapes=shapes,
        annotations=annotations,
        showlegend=False,
        dragmode="pan",
        margin={"l": 20, "r": 20, "t": 20, "b": 20},
        plot_bgcolor="white",
        uirevision="world",
    )
    fig.update_xaxes(showgrid=False, zeroline=False, constrain="domain")
    fig.update_yaxes(showgrid=False, zeroline=False, scaleanchor="x", scaleratio=1)
    return fig


def dynamic_patch(world: World, selected_robot: Robot | None) -> Patch:
    """
    Builds a Dash ``Patch`` updating only the dynamic per-robot trace data,
    leaving static shapes, planner graphs, and the layout untouched. Touching
    only trace data keeps each frame a pure restyle (no relayout), so motion
    does not disrupt an in-progress pan/zoom. Valid only while the trace layout
    is unchanged (same robots and same number of graphs).
    """
    patch = Patch()
    base = 1 + num_graph_traces(selected_robot)
    for i, robot in enumerate(world.robots):
        data = _robot_dynamic_xy(robot)
        for offset, (xs, ys) in data.items():
            index = base + TRACES_PER_ROBOT * i + offset
            patch["data"][index]["x"] = xs
            patch["data"][index]["y"] = ys
    return patch
