"""
Tests for the web UI figure building, commands, and app creation.
"""

from shapely.geometry import Polygon

from pyrobosim.core import World
from pyrobosim.web import commands
from pyrobosim.web.app import create_app, status_text
from pyrobosim.web.figure import (
    TRACES_PER_ROBOT,
    _svg_path,
    color_to_css,
    dynamic_patch,
    make_figure,
    num_graph_traces,
)


def test_color_to_css() -> None:
    """The RGB float color is converted to a CSS rgb string."""
    assert color_to_css((0.0, 0.0, 0.0)) == "rgb(0, 0, 0)"
    assert color_to_css((1.0, 1.0, 1.0)) == "rgb(255, 255, 255)"
    assert color_to_css((0.8, 0.0, 0.8)) == "rgb(204, 0, 204)"


def test_svg_path_handles_holes() -> None:
    """A polygon with a hole produces a multi-subpath SVG string (for even-odd fill)."""
    donut = Polygon(
        [(0, 0), (4, 0), (4, 4), (0, 4)],
        holes=[[(1, 1), (1, 3), (3, 3), (3, 1)]],
    )
    path = _svg_path(donut)
    # Exterior ring + one hole => two closed sub-paths.
    assert path.count("Z") == 2
    assert path.count("M") == 2
    assert _svg_path(None) == ""
    assert _svg_path(Polygon()) == ""


def test_make_figure(test_world: World) -> None:
    """Static geometry renders as shapes; dynamic content as traces at known indices."""
    fig = make_figure(test_world)

    # Static geometry: at least the room wall rings are shapes.
    assert len(fig.layout.shapes) >= len(test_world.rooms) > 0
    assert fig.layout.yaxis.scaleanchor == "x"

    # The view extent is set explicitly (autorange would re-fit the view every
    # time trace data is redrawn, e.g., mid pan/zoom).
    assert fig.layout.xaxis.range is not None
    assert fig.layout.yaxis.range is not None

    # Traces: planner-graph traces (none without a selected robot) + a fixed
    # block per robot.
    expected_traces = num_graph_traces(None) + TRACES_PER_ROBOT * len(test_world.robots)
    assert len(fig.data) == expected_traces

    # The status is rendered outside the figure (no plot title), so updating it
    # every frame does not relayout the plot.
    assert fig.layout.title.text in (None, "")


def test_status_text(test_world: World) -> None:
    """The status string reports the selected robot's name and battery."""
    assert status_text(None) == ""
    text = status_text(test_world.robots[0])
    assert test_world.robots[0].name in text
    assert "Battery" in text


def test_collision_polygons_add_shapes(test_world: World) -> None:
    """Enabling collision polygons adds extra shapes."""
    base = len(make_figure(test_world).layout.shapes)
    with_collision = len(
        make_figure(test_world, show_collision_polygons=True).layout.shapes
    )
    assert with_collision > base


def test_dynamic_patch(test_world: World) -> None:
    """The dynamic patch builds for the world's robots."""
    patch = dynamic_patch(test_world, test_world.robots[0])
    assert patch is not None


def test_held_object_label(test_world: World) -> None:
    """A held object's name renders as trace text, so it follows the robot."""
    robot = test_world.robots[0]
    obj = test_world.objects[0]
    robot.manipulated_object = obj
    try:
        fig = make_figure(test_world, selected_robot=robot)
        assert any(
            trace.text is not None and obj.name in trace.text for trace in fig.data
        )

        # The label hides when object names are toggled off.
        fig = make_figure(test_world, selected_robot=robot, show_object_names=False)
        assert not any(
            trace.text is not None and obj.name in trace.text for trace in fig.data
        )
    finally:
        robot.manipulated_object = None


def test_resolve_robot(test_world: World) -> None:
    """Robot names resolve to robots; 'world'/None resolve to no robot."""
    robot = test_world.robots[0]
    assert commands.resolve_robot(test_world, robot.name) is robot
    assert commands.resolve_robot(test_world, "world") is None
    assert commands.resolve_robot(test_world, None) is None


def test_create_app(test_world: World) -> None:
    """The interactive Dash app builds with a layout and registered callbacks."""
    app = create_app(test_world)
    assert app.layout is not None
    # The engine, goal-randomizer, and dispatch callbacks register.
    assert len(app.callback_map) >= 3
