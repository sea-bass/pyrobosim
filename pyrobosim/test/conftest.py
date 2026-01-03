import pathlib
from typing import Any

import pytest

from pyrobosim.core import WorldYamlLoader
from pyrobosim.utils.general import get_data_folder


@pytest.fixture(autouse=False, scope="module")
def world(w: str = "test_world.yaml") -> Any:
    """Create a reusable test world factory for sensors tests.

    Usage:
        w = world()  # loads default test_world.yaml
        w = world("other_world.yaml")  # loads another YAML in data folder
    """

    # Create a default world to proxy attributes to when the fixture
    # is used directly (so tests that expect `world.name` continue to work).
    default_world = WorldYamlLoader().from_file(
        pathlib.Path(get_data_folder()) / w,
    )

    return default_world
