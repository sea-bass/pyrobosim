#!/usr/bin/env python3

"""Unit tests for world Gazebo exporting utilities."""

import os
import pytest
import tempfile

from pyrobosim.core import WorldGazeboExporter, WorldYamlLoader
from pyrobosim.utils.general import get_data_folder


def load_world():
    """Load a test world."""
    world_file = os.path.join(get_data_folder(), "test_world.yaml")
    return WorldYamlLoader().from_yaml(world_file)


def test_export_gazebo_default_folder():
    """Exports a test world to Gazebo using the default folder."""
    world = load_world()

    exporter = WorldGazeboExporter(world)
    world_folder = exporter.export()
    assert world_folder == os.path.join(get_data_folder(), "worlds", world.name)


@pytest.mark.parametrize("classic", [True, False])
def test_export_gazebo(classic):
    """Exports a test world to Gazebo using a provided output folder."""
    world = load_world()
    output_folder = tempfile.mkdtemp()

    exporter = WorldGazeboExporter(world)
    world_folder = exporter.export(classic=classic, out_folder=output_folder)
    assert world_folder == os.path.join(output_folder, world.name)
