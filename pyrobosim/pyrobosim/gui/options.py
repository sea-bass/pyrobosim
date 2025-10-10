"""Options helpers for PyRoboSim GUI."""

from dataclasses import dataclass


@dataclass
class WorldCanvasOptions:
    """Options dataclass for WorldCanvas."""

    dpi: int = 100
    """The DPI for the MatPlotLib figure."""

    animation_dt: float = 0.1
    """ The animation timer period, in seconds."""

    show_collision_polygons: bool = False
    """ Whether to show collision polygons. """

    show_room_names: bool = True
    """Whether to show room names."""

    show_object_names: bool = True
    """ Whether to show object names."""

    show_location_names: bool = True
    """Whether to show location names."""

    show_robot_names: bool = True
    """Whether to show robot names."""
