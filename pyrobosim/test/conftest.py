import pathlib

import pytest

from pyrobosim.core import World, WorldYamlLoader
from pyrobosim.utils.general import get_data_folder


@pytest.fixture(autouse=False, scope="module")
def world() -> World:
    """Create a reusable test world factory for sensors tests.

    Usage:
        w = world()  # loads default test_world.yaml
        w = world("other_world.yaml")  # loads another YAML in data folder
    """

    # Create a default world to proxy attributes to when the fixture
    # is used directly (so tests that expect `world.name` continue to work).
    default_world = WorldYamlLoader().from_file(
        pathlib.Path(get_data_folder()) / "test_world.yaml",
    )

    class _WorldFactory:
        """Factory class to create World instances from YAML files.

        Allows attribute access to be proxied to a default World instance.

        :param default: The default World instance to proxy attributes to.
        :return: A new WorldFactory instance.
        """
        def __init__(self, default: World) -> None:
            """Creates a WorldFactory instance.

            :param default: The default World instance to proxy attributes to.
            :return: None
            """
            self._default = default

        def __call__(self, filename: str = "test_world.yaml") -> World:
            """Load a World instance from a YAML file in the data folder.

            :param filename: The YAML file name to load.
            :return: The loaded World instance.
            """
            return WorldYamlLoader().from_file(
                pathlib.Path(get_data_folder()) / filename,
            )

        def __getattr__(self, name: str) -> object:
            """Proxy attribute access to the default World instance.

            :param name: The attribute name to access.
            :return: The attribute value from the default World instance.
            """
            return getattr(self._default, name)

        def __repr__(self) -> str:
            """Return brief description of the WorldFactory.

            :return: String representation of the WorldFactory.
            """
            return f"<WorldFactory default={self._default.name}>"

    return _WorldFactory(default_world)
