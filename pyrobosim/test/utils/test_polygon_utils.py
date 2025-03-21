#!/usr/bin/env python3

"""
Unit tests for polygon utilities.
"""

import pytest
from pytest import LogCaptureFixture
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon as PolygonPatch
from scipy.spatial import ConvexHull
from shapely.geometry import Point, Polygon

from pyrobosim.utils.polygon import (
    add_coords,
    box_to_coords,
    convhull_to_rectangle,
    get_polygon_centroid,
    inflate_polygon,
    polygon_and_height_from_footprint,
    sample_from_polygon,
    transform_polygon,
)
from pyrobosim.utils.pose import Pose

##################################
# Utilities and helper functions #
##################################
square_coords = [(0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (0.0, 1.0), (0.0, 0.0)]
rectangle_coords = [(0.0, 0.0), (1.0, 0.0), (1.0, 2.0), (0.0, 2.0), (0.0, 0.0)]


def coords_approx_equal(
    coords: list[tuple[float, float]], expected_coords: list[tuple[float, float]]
) -> None:
    """Checks whether coordinate lists of tuples are approximately equal."""
    assert len(coords) == len(expected_coords)
    for i in range(len(coords)):
        assert coords[i] == pytest.approx(expected_coords[i])


##############
# Unit Tests #
##############


def test_add_coords() -> None:
    offset = (1.0, -2.0)
    offset_coords = add_coords(square_coords, offset)
    expected_coords = [(1.0, -2.0), (2.0, -2.0), (2.0, -1.0), (1.0, -1.0), (1.0, -2.0)]
    coords_approx_equal(offset_coords, expected_coords)


def test_box_to_coords() -> None:
    # Dimensions only
    coords = box_to_coords(dims=[1.0, 2.0])
    expected_coords = [(-0.5, -1.0), (0.5, -1.0), (0.5, 1.0), (-0.5, 1.0), (-0.5, -1.0)]
    coords_approx_equal(coords, expected_coords)

    # Dimensions and origin
    coords = box_to_coords(dims=(1.0, 2.0), origin=(0.5, 1.0))
    expected_coords = [(0.0, 0.0), (1.0, 0.0), (1.0, 2.0), (0.0, 2.0), (0.0, 0.0)]
    coords_approx_equal(coords, expected_coords)

    # Dimensions, origin, and rotation
    coords = box_to_coords(dims=(1.0, 2.0), origin=(1.0, 0.5), ang=np.pi / 2.0)
    expected_coords = [(2.0, 0.0), (2.0, 1.0), (0.0, 1.0), (0.0, 0.0), (2.0, 0.0)]
    coords_approx_equal(coords, expected_coords)


def test_get_polygon_centroid() -> None:
    square_poly = Polygon(square_coords)
    assert get_polygon_centroid(square_poly) == pytest.approx([0.5, 0.5])

    rectangle_poly = Polygon(rectangle_coords)
    assert get_polygon_centroid(rectangle_poly) == pytest.approx([0.5, 1.0])


def test_inflate_polygon() -> None:
    empty_poly = Polygon()
    square_poly = Polygon(square_coords)

    # Empty polygon
    inflated_poly = inflate_polygon(empty_poly, 0.0)
    assert inflated_poly == empty_poly

    # Zero inflation radius
    inflated_poly = inflate_polygon(square_poly, 0.0)
    inflated_poly_coords = list(inflated_poly.exterior.coords)
    coords_approx_equal(inflated_poly_coords, square_coords)

    # Positive inflation radius
    inflated_poly = inflate_polygon(square_poly, 0.25)
    inflated_poly_coords = list(inflated_poly.exterior.coords)
    expected_coords = [
        (-0.25, -0.25),
        (1.25, -0.25),
        (1.25, 1.25),
        (-0.25, 1.25),
        (-0.25, -0.25),
    ]
    coords_approx_equal(inflated_poly_coords, expected_coords)

    # Negative inflation radius
    inflated_poly = inflate_polygon(square_poly, -0.25)
    inflated_poly_coords = list(inflated_poly.exterior.coords)
    expected_coords = [
        (0.25, 0.25),
        (0.75, 0.25),
        (0.75, 0.75),
        (0.25, 0.75),
        (0.25, 0.25),
    ]
    coords_approx_equal(inflated_poly_coords, expected_coords)


def test_transform_polygon() -> None:
    empty_poly = Polygon()
    square_poly = Polygon(box_to_coords(dims=[1.0, 2.0]))

    # Empty polygon
    transformed_poly = transform_polygon(empty_poly, None)
    assert transformed_poly == empty_poly

    # No transformation, using None as pose
    transformed_poly = transform_polygon(square_poly, None)
    assert transformed_poly == square_poly

    # No transformation, using an identity transform pose
    transformed_poly = transform_polygon(square_poly, Pose())
    assert transformed_poly == square_poly

    # Translation only
    transformed_poly = transform_polygon(square_poly, Pose(x=1.0, y=-2.0))
    transformed_poly_coords = list(transformed_poly.exterior.coords)
    expected_coords = [(0.5, -3.0), (1.5, -3.0), (1.5, -1.0), (0.5, -1.0), (0.5, -3.0)]
    coords_approx_equal(transformed_poly_coords, expected_coords)

    # Translation and rotatoin
    transformed_poly = transform_polygon(
        square_poly, Pose(x=1.0, y=-2.0, yaw=np.pi / 2.0)
    )
    transformed_poly_coords = list(transformed_poly.exterior.coords)
    expected_coords = [(2.0, -2.5), (2.0, -1.5), (0.0, -1.5), (0.0, -2.5), (2.0, -2.5)]
    coords_approx_equal(transformed_poly_coords, expected_coords)


def test_sample_from_polygon(caplog: LogCaptureFixture) -> None:
    # Regular polygon
    poly = Polygon(square_coords)
    out = sample_from_polygon(poly)
    assert poly.contains(Point(out[0], out[1]))

    # Adversarial polygon that is just a straight line
    poly = Polygon([(0.0, 0.0), (0.0, 0.0), (1.0, 1.0), (1.0, 1.0)])
    out = sample_from_polygon(poly)
    assert out == (None, None)
    assert "Exceeded max polygon samples" in caplog.text


def test_polygon_from_footprint(caplog: LogCaptureFixture) -> None:
    # Box type
    footprint = {
        "type": "box",
        "dims": (1.0, 2.0),
        "offset": (0.5, 1.0),
        "height": 0.25,
    }
    polygon, height = polygon_and_height_from_footprint(footprint)
    poly_coords = list(polygon.exterior.coords)
    expected_coords = [(0.0, 0.0), (1.0, 0.0), (1.0, 2.0), (0.0, 2.0), (0.0, 0.0)]
    coords_approx_equal(poly_coords, expected_coords)
    assert height == pytest.approx(0.25)

    # Circle type
    footprint = {
        "type": "circle",
        "radius": (1.0),
        "offset": (0.5, 1.0),
        "height": 0.25,
    }
    polygon, height = polygon_and_height_from_footprint(footprint)
    poly_coords = list(polygon.exterior.coords)
    expected_polygon = Point(0.5, 1.0).buffer(1.0)
    expected_coords = list(expected_polygon.exterior.coords)
    coords_approx_equal(poly_coords, expected_coords)
    assert height == pytest.approx(0.25)

    # Polygon type
    footprint = {
        "type": "polygon",
        "coords": rectangle_coords,
        "offset": (0.5, 1.0),
        "height": 0.25,
    }
    polygon, height = polygon_and_height_from_footprint(footprint)
    poly_coords = list(polygon.exterior.coords)
    expected_coords = [(0.5, 1.0), (1.5, 1.0), (1.5, 3.0), (0.5, 3.0), (0.5, 1.0)]
    coords_approx_equal(poly_coords, expected_coords)
    assert height == pytest.approx(0.25)

    # Mesh type
    footprint = {
        "type": "mesh",
        "model_path": "$DATA/sample_models/coke_can",
        "mesh_path": "meshes/coke_can.dae",
        "height": 0.25,
    }
    polygon, height = polygon_and_height_from_footprint(footprint)
    assert isinstance(polygon, Polygon)
    assert height == pytest.approx(0.25)

    # Parent type
    parent_polygon = Polygon(rectangle_coords)
    footprint = {
        "type": "parent",
        "offset": (0.5, 1.0),
        "height": 0.25,
    }
    polygon, height = polygon_and_height_from_footprint(
        footprint, parent_polygon=parent_polygon
    )
    poly_coords = list(polygon.exterior.coords)
    expected_coords = [(0.5, 1.0), (1.5, 1.0), (1.5, 3.0), (0.5, 3.0), (0.5, 1.0)]
    coords_approx_equal(poly_coords, expected_coords)
    assert height == pytest.approx(0.25)

    # Using the pose argument to additionally transform the polygon
    footprint = {
        "type": "box",
        "dims": (1.0, 2.0),
        "offset": (0.5, 1.0),
        "height": 0.25,
    }
    polygon, height = polygon_and_height_from_footprint(
        footprint, pose=Pose(x=10.0, y=-10.0, yaw=np.pi / 2.0)
    )
    poly_coords = list(polygon.exterior.coords)
    expected_coords = [
        (10.0, -10.0),
        (10.0, -9.0),
        (8.0, -9.0),
        (8.0, -10.0),
        (10.0, -10.0),
    ]
    coords_approx_equal(poly_coords, expected_coords)
    assert height == pytest.approx(0.25)

    # Invalid type
    footprint = {"type": "invalid"}
    with pytest.raises(ValueError) as exc_info:
        polygon_and_height_from_footprint(footprint, parent_polygon=parent_polygon)
    assert str(exc_info.value) == "Invalid footprint type: invalid"


def test_convhull_to_rectangle(display: bool = False) -> None:
    # Create a cross-shaped object
    xy_pts = np.array(
        [
            [-0.05, -0.15],
            [0.05, -0.15],
            [0.05, -0.05],
            [0.15, -0.05],
            [0.15, 0.05],
            [0.05, 0.05],
            [0.05, 0.15],
            [-0.05, 0.15],
            [-0.05, 0.05],
            [-0.15, 0.05],
            [-0.15, -0.05],
            [-0.05, -0.05],
            [-0.05, -0.15],
        ]
    )

    # Then, gets its convex hull and a best-fit rectangle
    hull = ConvexHull(xy_pts)
    hull_pts = np.vstack([xy_pts[hull.vertices, :], xy_pts[hull.vertices[0], :]])
    (rect_pose, rect_dims, rect_pts) = convhull_to_rectangle(hull_pts)

    # Check the rectangle origin and dimension
    assert rect_pose.x == pytest.approx(0.0)
    assert rect_pose.y == pytest.approx(0.0)
    assert rect_dims == pytest.approx([0.2 * np.sqrt(2), 0.2 * np.sqrt(2)])

    # Display the outputs
    if display:
        fig, ax = plt.subplots()
        poly = PolygonPatch(xy_pts)
        ax.add_patch(poly)
        plt.plot(hull_pts[:, 0], hull_pts[:, 1], "r--", lw=2)
        plt.plot(rect_pts[:, 0], rect_pts[:, 1], "b:", lw=2)
        plt.legend(["Polygon", "Convex Hull", "Rectangle Fit"])
        plt.axis("equal")
        plt.show()
