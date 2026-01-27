#!/usr/bin/env python3

"""Unit tests for occupancy grid tools"""

import pathlib
import tempfile

import numpy as np
import pytest
import yaml
from PIL import Image

from pyrobosim.navigation.occupancy_grid import OccupancyGrid

# Create test occupancy grid
# Note: The np.rot90() call is useful to visually align the numpy array below with how the grid is displayed.
TEST_DATA = np.rot90(
    np.array(
        [
            [1, 1, 1, 1, 1],
            [1, 0, 0, 0, 1],
            [1, 0, 0, 0, 1],
            [1, 0, 1, 0, 1],
            [1, 0, 1, 0, 1],
            [1, 1, 1, 1, 1],
        ],
    ),
    k=-1,
)


def test_create_occupancy_grid_default_args() -> None:
    grid = OccupancyGrid(TEST_DATA, 1.0)

    assert grid.data.shape == (5, 6)
    assert grid.width == 5
    assert grid.height == 6
    assert grid.resolution == 1.0
    assert grid.origin == (0.0, 0.0)
    assert grid.occ_thresh == 0.65
    assert grid.free_thresh == 0.2


def test_create_occupancy_grid_nondefault_args() -> None:
    grid = OccupancyGrid(
        TEST_DATA,
        0.25,
        origin=(-1.0, 2.0),
        occ_thresh=0.8,
        free_thresh=0.12,
    )

    assert grid.data.shape == (5, 6)
    assert grid.width == 5
    assert grid.height == 6
    assert grid.resolution == 0.25
    assert grid.origin == (-1.0, 2.0)
    assert grid.occ_thresh == 0.8
    assert grid.free_thresh == 0.12


def test_is_in_bounds() -> None:
    grid = OccupancyGrid(TEST_DATA, 1.0)

    assert grid.is_in_bounds((0, 0))  # Lower left
    assert grid.is_in_bounds((4, 5))  # Upper right
    assert grid.is_in_bounds((2, 3))  # Middle
    assert not grid.is_in_bounds((-1, -1))
    assert not grid.is_in_bounds((5, 4))
    assert not grid.is_in_bounds((10, 2))


def test_world_to_grid_conversion() -> None:
    grid = OccupancyGrid(TEST_DATA, 1.0, origin=(1.0, 2.0))

    orig_grid_pt = (2, 3)
    world_pt = grid.grid_to_world(orig_grid_pt)
    assert world_pt == pytest.approx((3.0, 5.0))

    new_grid_pt = grid.world_to_grid(world_pt)
    assert new_grid_pt == orig_grid_pt


def test_is_occupied() -> None:
    grid = OccupancyGrid(TEST_DATA, 1.0)

    # Check occupancy of known points in the grid.
    assert grid.is_occupied((0, 0))
    assert grid.is_occupied((4, 5))
    assert grid.is_occupied((5, 4))
    assert not grid.is_occupied((1, 1))
    assert not grid.is_occupied((3, 4))

    # Modify the occupancy grid data in place and see that the results change accordingly.
    grid.data[3][4] = 1
    assert grid.is_occupied((3, 4))
    grid.data[3][4] = 0
    assert not grid.is_occupied((3, 4))


def test_connectable() -> None:
    grid = OccupancyGrid(TEST_DATA, 1.0)

    # Connectable in a vertical line
    is_connectable, last_point = grid.has_straight_line_connection((1, 1), (1, 4))
    assert is_connectable and last_point == (1, 4)

    # Connectable in a horizontal line
    is_connectable, last_point = grid.has_straight_line_connection((1, 4), (3, 4))
    assert is_connectable and last_point == (3, 4)

    # Connectable in a diagonal line
    is_connectable, last_point = grid.has_straight_line_connection((1, 2), (3, 4))
    assert is_connectable and last_point == (3, 4)

    # Points are in a straight line, but there is an occupied cell
    is_connectable, last_point = grid.has_straight_line_connection((1, 1), (3, 3))
    assert not is_connectable

    # Start point and goal point are in collision
    is_connectable, last_point = grid.has_straight_line_connection((0, 0), (4, 4))
    assert not is_connectable

    # Free path, but it's not a straight line
    is_connectable, last_point = grid.has_straight_line_connection((1, 2), (2, 4))


def test_save_load_to_file() -> None:
    grid = OccupancyGrid(
        TEST_DATA,
        0.25,
        origin=(-1.0, 2.0),
        occ_thresh=0.8,
        free_thresh=0.12,
    )

    output_folder = tempfile.mkdtemp()
    output_filename = "test_grid"
    grid.save_to_file(output_folder, output_filename)

    # Check that the right files were written
    image_path = pathlib.Path(output_folder) / "test_grid.pgm"
    assert image_path.is_file()
    yaml_path = pathlib.Path(output_folder) / "test_grid.yaml"
    assert yaml_path.is_file()

    # Verify the contents of the YAML file
    with yaml_path.open("r") as f:
        yaml_dict = yaml.load(f, yaml.FullLoader)
        assert isinstance(
            yaml_dict, dict
        ), f"Loaded YAML is not a dict. type={type(yaml_dict)}, content={yaml_dict!r}"

        # Ensure required keys exist
        for key in ("image", "resolution", "origin", "occupied_thresh", "free_thresh"):
            assert (
                key in yaml_dict
            ), f"Missing key '{key}' in YAML. keys_present={list(yaml_dict.keys())!r}"

        # Image path: compare as strings to avoid Path vs str mismatch
        got_image = yaml_dict["image"]
        expected_image = str(image_path.as_posix())
        assert got_image == expected_image, (
            "YAML 'image' value mismatch:\n"
            f"  got  ({type(got_image)}): {got_image!r}\n"
            f"  want ({type(expected_image)}): {expected_image!r}"
        )

        # Resolution: use approx for float comparison
        got_resolution = yaml_dict["resolution"]
        assert got_resolution == pytest.approx(grid.resolution), (
            "YAML 'resolution' value mismatch:\n"
            f"  got  ({type(got_resolution)}): {got_resolution!r}\n"
            f"  want ({type(grid.resolution)}): {grid.resolution!r}"
        )

        # Origin: explicit expected format and detailed message
        got_origin = yaml_dict["origin"]
        expected_origin = list(grid.origin) + [0]
        assert got_origin == expected_origin, (
            "YAML 'origin' value mismatch:\n"
            f"  got  ({type(got_origin)}): {got_origin!r}\n"
            f"  want ({type(expected_origin)}): {expected_origin!r}"
        )

        # Thresholds: use approx for float comparison with clear messages
        got_occ = yaml_dict["occupied_thresh"]
        assert got_occ == pytest.approx(grid.occ_thresh), (
            "YAML 'occupied_thresh' value mismatch:\n"
            f"  got  ({type(got_occ)}): {got_occ!r}\n"
            f"  want ({type(grid.occ_thresh)}): {grid.occ_thresh!r}"
        )

        got_free = yaml_dict["free_thresh"]
        assert got_free == pytest.approx(grid.free_thresh), (
            "YAML 'free_thresh' value mismatch:\n"
            f"  got  ({type(got_free)}): {got_free!r}\n"
            f"  want ({type(grid.free_thresh)}): {grid.free_thresh!r}"
        )

    # Verify the contents of the image file
    image = Image.open(image_path)
    assert image.size == (5, 6)
    assert image.getpixel((0, 0)) == 0
    assert image.getpixel((1, 1)) == 254

    # Now load back the occupancy grid
    loaded_grid = OccupancyGrid.from_file(output_folder)
    assert isinstance(
        loaded_grid, OccupancyGrid
    ), f"Loaded object is not OccupancyGrid: {type(loaded_grid)}"

    # Use numpy's testing helpers to give detailed diffs for array mismatches
    np.testing.assert_array_equal(
        loaded_grid.data,
        grid.data,
        err_msg=(
            f"Loaded grid data does not match saved grid data. "
            f"loaded shape={loaded_grid.data.shape}, expected shape={grid.data.shape}"
        ),
    )

    assert (
        loaded_grid.resolution == grid.resolution
    ), f"Resolution mismatch: loaded={loaded_grid.resolution}, expected={grid.resolution}"
    assert (
        loaded_grid.origin == grid.origin
    ), f"Origin mismatch: loaded={loaded_grid.origin}, expected={grid.origin}"
    assert (
        loaded_grid.occ_thresh == grid.occ_thresh
    ), f"Occupied threshold mismatch: loaded={loaded_grid.occ_thresh}, expected={grid.occ_thresh}"
    assert (
        loaded_grid.free_thresh == grid.free_thresh
    ), f"Free threshold mismatch: loaded={loaded_grid.free_thresh}, expected={grid.free_thresh}"


def test_occupancy_grid_from_world() -> None:
    from pyrobosim.core import World

    world = World()
    world.add_room(name="room1", footprint=[(0, 0), (1, 0), (1, 1), (0, 1)])
    world.add_room(name="room2", footprint=[(2, 0), (3, 0), (3, 1), (2, 1)])
    world.add_hallway(room_start="room1", room_end="room2", width=0.25)

    grid = OccupancyGrid.from_world(world, 0.1, 0.0)

    # Check some known unoccupied points
    assert not grid.is_occupied(grid.world_to_grid((0.5, 0.5)))
    assert not grid.is_occupied(grid.world_to_grid((2.25, 0.75)))
    assert not grid.is_occupied(grid.world_to_grid((1.5, 0.5)))

    # Check some known occupied points
    assert grid.is_occupied(grid.world_to_grid((0.0, 0.0)))
    assert grid.is_occupied(grid.world_to_grid((2.8, 1.1)))
    assert grid.is_occupied(grid.world_to_grid((1.5, 0.75)))

    # Check out of bounds points
    assert grid.is_occupied(grid.world_to_grid((-20.0, 0.0)))
    assert grid.is_occupied(grid.world_to_grid((2.0, 10.0)))
