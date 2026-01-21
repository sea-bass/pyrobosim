import pathlib

import pytest

from pyrobosim.core import World, WorldYamlLoader
from pyrobosim.utils.general import get_data_folder


@pytest.fixture(autouse=False, scope="module")
def test_world(world_config_file: str = "test_world.yaml") -> World:
    """Create a reusable test world for tests.

    Usage:
        def test_example(test_world: World) -> None:
            # test_world is loaded from test_world.yaml (default)
            assert test_world.name == "test_world"

    To use with a non-default filename, parametrize the fixture with indirect=True:
        @pytest.mark.parametrize("test_world", ["test_world_multirobot.yaml"], indirect=True)
        def test_example(test_world: World) -> None:
            # test_world is loaded from test_world_multirobot.yaml
            assert test_world.name == "test_world_multirobot"
    """

    # Create a default world to proxy attributes to when the fixture
    # is used directly (so tests that expect `world.name` continue to work).
    default_world = WorldYamlLoader().from_file(
        pathlib.Path(get_data_folder()) / world_config_file,
    )

    return default_world
