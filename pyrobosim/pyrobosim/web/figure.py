"""
Builds a Plotly figure from a PyRoboSim world.

Rendering is split into two layers:

* **Static geometry** (rooms, hallways, locations, object spawns, resting
  objects, optional collision polygons) is drawn as ``layout.shapes``. These
  polygons often have interior holes (e.g., a room's wall ring), so they use
  SVG ``path`` shapes with an even-odd fill rule.
* **Dynamic content** (planner graphs, and per robot: planner path, sensor
  data, body, orientation line, and any held object) is drawn as traces at
  stable indices, so it can be updated each frame with :func:`dynamic_patch`
  instead of rebuilding the whole figure.
"""

import math
from typing import Any, NamedTuple, Sequence

import plotly.graph_objects as go
from dash import Patch
from shapely.geometry import MultiPolygon, Polygon

from ..core.robot import Robot
from ..core.world import World
from ..utils.polygon import transform_polygon

# Multiplier of robot radius for the orientation line.
ROBOT_DIR_LINE_FACTOR = 3.0
# Fraction of the world extent added as a margin around the view.
VIEW_MARGIN = 0.05
# Color used for collision-polygon overlays.
COLLISION_COLOR = "rgb(255, 0, 255)"

# None-separated x and y coordinate lists for one scatter trace.
TraceXY = tuple[list[Any], list[Any]]


class RobotTraceData(NamedTuple):
    """The (x, y) data for one robot's dynamic traces, in trace order."""

    path: TraceXY
    sensor_fill: TraceXY
    sensors: TraceXY
    body: TraceXY
    direction: TraceXY
    held_object: TraceXY


# Number of dynamic traces emitted per robot.
TRACES_PER_ROBOT = len(RobotTraceData._fields)


def color_to_css(color: Sequence[float]) -> str:
    """
    Converts a PyRoboSim RGB color to a CSS ``rgb(...)`` string.

    :param color: An (R, G, B) sequence with each channel in the range (0.0, 1.0),
        as produced by :func:`pyrobosim.utils.general.parse_color`.
    :return: A CSS color string, e.g., ``"rgb(204, 0, 204)"``.
    """
    r, g, b = (int(round(255 * channel)) for channel in color)
    return f"rgb({r}, {g}, {b})"


# ---------------------------------------------------------------------------
# Static shapes
# ---------------------------------------------------------------------------
def _rgba(rgb_str: str, alpha: float) -> str:
    """
    Converts a CSS ``rgb(...)`` string into ``rgba(...)`` with the given alpha.

    :param rgb_str: A CSS color string, e.g., ``"rgb(204, 0, 204)"``.
    :param alpha: The alpha channel value, in the range (0.0, 1.0).
    :return: A CSS color string, e.g., ``"rgba(204, 0, 204, 0.5)"``.
    """
    inner = rgb_str[rgb_str.index("(") + 1 : rgb_str.index(")")]
    return f"rgba({inner}, {alpha})"


def _ring_path(coords: Sequence[Sequence[float]]) -> str:
    """
    Builds an SVG sub-path string (``M ... L ... Z``) for one polygon ring.

    :param coords: The (x, y) coordinates along the ring.
    :return: The SVG sub-path string, or an empty string for no coordinates.
    """
    if not coords:
        return ""
    commands = [
        f"{'M' if i == 0 else 'L'}{x:.5f},{y:.5f}" for i, (x, y) in enumerate(coords)
    ]
    return " ".join(commands) + " Z"


def _svg_path(geom: Polygon | MultiPolygon | None) -> str:
    """
    Converts a Shapely polygon (with holes / parts) to an SVG path string.

    :param geom: The polygon to convert.
    :return: The SVG path string, or an empty string for a missing/empty polygon.
    """
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
    """
    Builds a Plotly ``path`` shape for a polygon.

    :param geom: The polygon to render.
    :param line_color: The CSS color for the polygon outline.
    :param fillcolor: The CSS fill color. If None, the shape is not filled.
    :param width: The outline width, in pixels.
    :param dash: The outline dash style (e.g., ``"dash"``). If None, a solid line.
    :param layer: The Plotly shape layer (``"below"`` or ``"above"`` traces).
    :return: The shape dictionary, or None if the polygon is missing/empty.
    """
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


def _polygon_xy(geom: Polygon | MultiPolygon | None) -> TraceXY:
    """
    Extracts None-separated x/y coordinate lists for a polygon's exterior ring(s).

    :param geom: The polygon whose exterior ring(s) to extract.
    :return: The x and y coordinate lists, with None separating each ring.
    """
    xs: list[Any] = []
    ys: list[Any] = []
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


def _label(x: float, y: float, text: str, color: str, size: int) -> dict[str, Any]:
    """
    Builds a Plotly annotation for an entity name.

    :param x: The x position of the label.
    :param y: The y position of the label.
    :param text: The label text.
    :param color: The CSS text color.
    :param size: The font size, in points.
    :return: The annotation dictionary.
    """
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
# Dynamic per-robot traces
# ---------------------------------------------------------------------------
def _robot_trace_data(robot: Robot) -> RobotTraceData:
    """
    Computes the (x, y) data for a robot's dynamic traces.

    :param robot: The robot to compute trace data for.
    :return: The per-trace (x, y) data.
    """
    pose = robot.get_pose()

    # Render only the path snapshotted by the robot; the planner's live state
    # may be mid-replan. The path persists after navigation until the next
    # plan replaces it or the planner is reset.
    path = robot.displayed_path

    path_x: list[Any] = []
    path_y: list[Any] = []
    if path is not None and path.num_poses > 1:
        path_x = [p.x for p in path.poses]
        path_y = [p.y for p in path.poses]

    # Sensor data, as None-separated line segments (e.g., lidar rays) and
    # filled polygon rings (e.g., FOV cones).
    sensor_x: list[Any] = []
    sensor_y: list[Any] = []
    fill_x: list[Any] = []
    fill_y: list[Any] = []
    for sensor in robot.sensors.values():
        for segment in sensor.get_display_coords():
            for point in segment:
                sensor_x.append(point[0])
                sensor_y.append(point[1])
            sensor_x.append(None)
            sensor_y.append(None)
        for ring in sensor.get_display_polygons():
            for point in ring:
                fill_x.append(point[0])
                fill_y.append(point[1])
            fill_x.append(None)
            fill_y.append(None)

    length = ROBOT_DIR_LINE_FACTOR * robot.radius
    yaw = pose.get_yaw()
    dir_x = [pose.x, pose.x + length * math.cos(yaw)]
    dir_y = [pose.y, pose.y + length * math.sin(yaw)]

    held_x: list[Any] = []
    held_y: list[Any] = []
    if robot.manipulated_object is not None:
        obj = robot.manipulated_object
        held_polygon = transform_polygon(obj.raw_polygon, obj.pose)
        held_x, held_y = _polygon_xy(held_polygon)
        # Extra point anchoring the held object's name label, offset like the
        # resting-object labels. It follows the None ring separator, so no
        # line connects to it.
        xmin, ymin, xmax, ymax = held_polygon.bounds
        held_x.append(obj.pose.x + (xmax - xmin))
        held_y.append(obj.pose.y + (ymax - ymin))

    return RobotTraceData(
        path=(path_x, path_y),
        sensor_fill=(fill_x, fill_y),
        sensors=(sensor_x, sensor_y),
        body=_polygon_xy(robot.polygon),
        direction=(dir_x, dir_y),
        held_object=(held_x, held_y),
    )


def _robot_traces(robot: Robot, show_object_names: bool = True) -> list[go.Scatter]:
    """
    Builds the styled dynamic traces for a single robot.

    :param robot: The robot to build traces for.
    :param show_object_names: If True, labels any held object with its name.
    :return: The list of traces, in the field order of :class:`RobotTraceData`.
    """
    data = _robot_trace_data(robot)
    color = color_to_css(robot.color)
    held_obj = robot.manipulated_object
    held_color = color_to_css(held_obj.viz_color) if held_obj is not None else color
    styles: dict[str, dict[str, Any]] = {
        "path": {"line": {"color": color, "width": 3}, "opacity": 0.5},
        "sensor_fill": {
            "line": {"color": color, "width": 1},
            "opacity": 0.5,
            "fill": "toself",
            "fillcolor": _rgba(color, 0.3),
        },
        "sensors": {"line": {"color": color, "width": 0.5}, "opacity": 0.5},
        "body": {
            "line": {"color": color, "width": 2},
            "fill": "toself",
            "fillcolor": "white",
        },
        "direction": {"line": {"color": color, "width": 2}},
        "held_object": {"line": {"color": held_color, "width": 2}},
    }
    if held_obj is not None and show_object_names:
        # The held object's name renders as trace text on its label anchor
        # point (the last one), so it keeps following the object during smooth
        # motion, which restyles trace data without touching the layout.
        styles["held_object"].update(
            {
                "mode": "lines+text",
                "text": [""] * (len(data.held_object[0]) - 1) + [held_obj.name],
                "textposition": "bottom center",
                "textfont": {"color": held_color, "size": 8},
            }
        )
    return [
        go.Scatter(
            x=xs,
            y=ys,
            hoverinfo="skip",
            showlegend=False,
            **{"mode": "lines", **styles[field]},
        )
        for field, (xs, ys) in zip(RobotTraceData._fields, data)
    ]


# ---------------------------------------------------------------------------
# Planner graphs (shown only for the selected robot)
# ---------------------------------------------------------------------------
def num_graph_traces(selected_robot: Robot | None) -> int:
    """
    Counts the traces emitted for planner graphs (two per graph: edges, nodes).

    :param selected_robot: The robot whose planner graphs are displayed, if any.
    :return: The number of planner-graph traces at the start of the figure.
    """
    if selected_robot is None:
        return 0
    return 2 * len(selected_robot.displayed_graphs)


def _graph_traces(selected_robot: Robot | None) -> list[go.Scatter]:
    """
    Builds edge and node traces for the selected robot's planner graphs.

    Only the graphs snapshotted by the robot are rendered, for the same reason
    as the path in :func:`_robot_trace_data`.

    :param selected_robot: The robot whose planner graphs to render, if any.
    :return: The list of traces, two (edges, nodes) per graph.
    """
    traces: list[go.Scatter] = []
    if selected_robot is None:
        return traces
    for graph in selected_robot.displayed_graphs:
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
# Public API
# ---------------------------------------------------------------------------
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
    Builds a complete Plotly figure rendering a world.

    Trace layout (used by :func:`dynamic_patch`): planner-graph traces first,
    followed by ``TRACES_PER_ROBOT`` traces for each robot in order. The view
    extent comes from explicit axis ranges (not autorange), so redrawing trace
    data never re-fits the view.

    :param world: The world to render.
    :param selected_robot: The robot whose knowledge (closed hallways, known
        objects) and planner graphs are displayed, or None for full world state.
    :param show_room_names: If True, shows room name labels.
    :param show_location_names: If True, shows location name labels.
    :param show_object_names: If True, shows object name labels.
    :param show_robot_names: If True, shows robot name labels.
    :param show_collision_polygons: If True, overlays the collision polygons.
    :return: The complete Plotly figure for the world.
    """
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
        # Objects render above robots.
        shape = _polygon_shape(obj.polygon, color, layer="above")
        if shape:
            shapes.append(shape)
        if show_object_names:
            oxmin, oymin, oxmax, oymax = obj.polygon.bounds
            annotations.append(
                _label(
                    obj.pose.x + (oxmax - oxmin),
                    obj.pose.y + (oymax - oymin),
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

    traces: list[go.Scatter] = _graph_traces(selected_robot)
    for robot in world.robots:
        traces += _robot_traces(robot, show_object_names=show_object_names)

    # View extent: the world bounds plus a small margin.
    if world.x_bounds is not None and world.y_bounds is not None:
        (xmin, xmax), (ymin, ymax) = world.x_bounds, world.y_bounds
        pad = VIEW_MARGIN * max(xmax - xmin, ymax - ymin)
        xmin, ymin, xmax, ymax = xmin - pad, ymin - pad, xmax + pad, ymax + pad
    else:
        xmin, ymin, xmax, ymax = -1.0, -1.0, 1.0, 1.0

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
    fig.update_xaxes(
        range=[xmin, xmax], showgrid=False, zeroline=False, constrain="domain"
    )
    fig.update_yaxes(
        range=[ymin, ymax],
        showgrid=False,
        zeroline=False,
        scaleanchor="x",
        scaleratio=1,
    )
    return fig


def dynamic_patch(world: World, selected_robot: Robot | None) -> Patch:
    """
    Builds a Dash ``Patch`` updating only the dynamic per-robot trace data,
    leaving static shapes, planner graphs, and the layout untouched. Touching
    only trace data keeps each frame a pure restyle (no relayout), so motion
    does not disrupt an in-progress pan/zoom. Valid only while the trace
    layout is unchanged (same robots and same number of graphs).

    :param world: The world whose robots to update.
    :param selected_robot: The robot whose planner graphs are displayed, if any.
        Used to offset the per-robot trace indices past the graph traces.
    :return: A Dash patch updating the dynamic trace data in place.
    """
    patch = Patch()
    base = num_graph_traces(selected_robot)
    for i, robot in enumerate(world.robots):
        for offset, (xs, ys) in enumerate(_robot_trace_data(robot)):
            index = base + TRACES_PER_ROBOT * i + offset
            patch["data"][index]["x"] = xs
            patch["data"][index]["y"] = ys
    return patch
