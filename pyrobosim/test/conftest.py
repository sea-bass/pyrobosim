import pathlib

import pytest

from pyrobosim.core import World, WorldYamlLoader
from pyrobosim.utils.general import get_data_folder


@pytest.fixture(autouse=False, scope="module")
def test_world(world_config_file: str = "test_world.yaml") -> World:
    """Create a reusable test world factory for sensors tests.

    Usage:
        w = world()  # loads default test_world.yaml
        w = world("other_world.yaml")  # loads another YAML in data folder

    To use with a non-default filename, parametrize the fixture or pass the argument directly if supported.
    Example:
        @pytest.mark.parametrize("world_config_file", ["test_world_multirobot.yaml"])
        def test_example(world: World) -> None:
            # world is loaded from test_world_multirobot.yaml
            pass
    """

    # Create a default world to proxy attributes to when the fixture
    # is used directly (so tests that expect `world.name` continue to work).
    default_world = WorldYamlLoader().from_file(
        pathlib.Path(get_data_folder()) / world_config_file,
    )

    return default_world
