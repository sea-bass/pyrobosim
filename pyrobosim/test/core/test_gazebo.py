#!/usr/bin/env python3

"""Unit tests for world Gazebo exporting utilities."""

import pathlib
import tempfile

import pytest

from pyrobosim.core import WorldGazeboExporter
from pyrobosim.utils.general import get_data_folder
from test.conftest import WorldFactoryProtocol


def test_export_gazebo_default_folder(world: WorldFactoryProtocol) -> None:
    """Exports a test world to Gazebo using the default folder."""

    exporter = WorldGazeboExporter(world)
    world_folder = exporter.export()
    assert world_folder == get_data_folder() / "worlds" / world.name


@pytest.mark.parametrize("classic", [True, False])  # type: ignore[misc]
def test_export_gazebo(classic: bool, world: WorldFactoryProtocol) -> None:
    """Exports a test world to Gazebo using a provided output folder."""
    output_folder = tempfile.mkdtemp()

    exporter = WorldGazeboExporter(world)
    world_folder = exporter.export(classic=classic, out_folder=output_folder)
    assert world_folder == pathlib.Path(output_folder) / world.name
