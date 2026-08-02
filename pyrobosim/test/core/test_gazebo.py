#!/usr/bin/env python3

"""Unit tests for world Gazebo exporting utilities."""

import pathlib
import tempfile

from pyrobosim.core import World, WorldGazeboExporter
from pyrobosim.utils.general import get_data_folder


def test_export_gazebo_default_folder(test_world: World) -> None:
    """Exports a test world to Gazebo using the default folder."""

    exporter = WorldGazeboExporter(test_world)
    world_folder = exporter.export()
    assert world_folder == get_data_folder() / "worlds" / test_world.name


def test_export_gazebo(test_world: World) -> None:
    """Exports a test world to Gazebo using a provided output folder."""
    output_folder = tempfile.mkdtemp()

    exporter = WorldGazeboExporter(test_world)
    world_folder = exporter.export(out_folder=output_folder)
    assert world_folder == pathlib.Path(output_folder) / test_world.name
