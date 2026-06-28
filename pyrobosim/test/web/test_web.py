"""
Tests for the web frontend serialization, figure building, and commands.

The whole module is skipped if the optional web dependencies (plotly/dash) are
not installed.
"""

import pathlib

import pytest

pytest.importorskip("plotly")
pytest.importorskip("dash")

from shapely.geometry import Polygon

from pyrobosim.core import World, WorldYamlLoader
from pyrobosim.utils.general import get_data_folder
from pyrobosim.web import commands
from pyrobosim.web.figure import (
    TRACES_PER_ROBOT,
    _svg_path,
    color_to_css,
    dynamic_patch,
    make_figure,
    num_graph_traces,
    status_text,
)


@pytest.fixture(scope="module")
def web_world() -> World:
    """Loads a world from YAML for the web tests."""
    return WorldYamlLoader().from_file(
        pathlib.Path(get_data_folder()) / "test_world.yaml"
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


def test_make_figure(web_world: World) -> None:
    """Static geometry renders as shapes; dynamic content as traces at known indices."""
    fig = make_figure(web_world)

    # Static geometry: at least the room wall rings are shapes.
    assert len(fig.layout.shapes) >= len(web_world.rooms) > 0
    assert fig.layout.yaxis.scaleanchor == "x"

    # Traces: bounds + planner-graph traces (none without a selected robot) +
    # a fixed block per robot.
    expected_traces = (
        1 + num_graph_traces(None) + TRACES_PER_ROBOT * len(web_world.robots)
    )
    assert len(fig.data) == expected_traces

    # The status is rendered outside the figure (no plot title), so updating it
    # every frame does not relayout the plot.
    assert fig.layout.title.text in (None, "")


def test_status_text(web_world: World) -> None:
    """The status string reports the selected robot's name and battery."""
    assert status_text(None) == ""
    text = status_text(web_world.robots[0])
    assert web_world.robots[0].name in text
    assert "Battery" in text


def test_collision_polygons_add_shapes(web_world: World) -> None:
    """Enabling collision polygons adds extra shapes."""
    base = len(make_figure(web_world).layout.shapes)
    with_collision = len(
        make_figure(web_world, show_collision_polygons=True).layout.shapes
    )
    assert with_collision > base


def test_dynamic_patch(web_world: World) -> None:
    """The dynamic patch builds for the world's robots."""
    patch = dynamic_patch(web_world, web_world.robots[0])
    assert patch is not None


def test_resolve_robot(web_world: World) -> None:
    """Robot names resolve to robots; 'world'/None resolve to no robot."""
    robot = web_world.robots[0]
    assert commands.resolve_robot(web_world, robot.name) is robot
    assert commands.resolve_robot(web_world, "world") is None
    assert commands.resolve_robot(web_world, None) is None


def test_create_app(web_world: World) -> None:
    """The interactive Dash app builds with a layout and registered callbacks."""
    from pyrobosim.web.app import create_app

    app = create_app(web_world)
    assert app.layout is not None
    # The engine (figure + buttons + interval) and dispatch callbacks register.
    assert len(app.callback_map) >= 2
