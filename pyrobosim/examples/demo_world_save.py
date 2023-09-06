#!/usr/bin/env python3

"""
Tests Gazebo world saving and occupancy grid export capabilities.
"""

import argparse
import os

from pyrobosim.core import WorldGazeboExporter, WorldYamlLoader
from pyrobosim.navigation import OccupancyGrid
from pyrobosim.utils.general import get_data_folder


def parse_args():
    parser = argparse.ArgumentParser(
        description="Test Gazebo world saving and occupancy grid export."
    )
    parser.add_argument(
        "--world-file",
        default="test_world.yaml",
        help="YAML file name (should be in the pyrobosim/data folder). "
        + "Defaults to test_world.yaml",
    )
    parser.add_argument(
        "--out-folder", default=None, help="Output folder for exporting the world"
    )
    parser.add_argument(
        "--classic", action="store_true", help="Enable to export to Gazebo Classic"
    )
    parser.add_argument("--save-grid", action="store_true", help="Save occupancy grid")
    parser.add_argument(
        "--grid-resolution",
        type=float,
        default=0.05,
        help="Occupancy grid resolution (meters)",
    )
    parser.add_argument(
        "--grid-inflation-radius",
        type=float,
        default=0.05,
        help="Occupancy grid inflation radius (meters)",
    )
    parser.add_argument(
        "--show-grid", action="store_true", help="Show the occupancy grid"
    )
    return parser.parse_args()


def main():
    args = parse_args()

    # Load a test world from YAML file.
    data_folder = get_data_folder()
    loader = WorldYamlLoader()
    world = loader.from_yaml(os.path.join(data_folder, args.world_file))

    # Export a Gazebo world.
    exp = WorldGazeboExporter(world)
    world_folder = exp.export(classic=args.classic, out_folder=args.out_folder)

    # Save an occupancy grid to the world folder and show it, if enabled.
    if args.save_grid:
        occ_grid = OccupancyGrid.from_world(
            world,
            resolution=args.grid_resolution,
            inflation_radius=args.grid_inflation_radius,
        )
        occ_grid.save_to_file(world_folder)
        if args.show_grid:
            occ_grid.show()


if __name__ == "__main__":
    main()
