import pytest
from matplotlib.colors import CSS4_COLORS, to_rgb

from pyrobosim.utils.general import parse_color


def test_parse_color():
    """Testing parse color with different input color formats"""
    # Test with RGB list
    color_rgb_list = (1.0, 0.0, 0.0)
    assert parse_color([1.0, 0.0, 0.0]) == color_rgb_list

    # Test with RGB tuple
    color_rgb_tuple = (0.0, 1.0, 0.0)
    assert parse_color((0.0, 1.0, 0.0)) == color_rgb_tuple

    # Test with named color
    color_by_name = "red"
    assert parse_color("red") == to_rgb(CSS4_COLORS[color_by_name])

    # Test with hexadecimal color format
    color_hex = "#00FFFF"
    assert parse_color("#00FFFF") == to_rgb(color_hex)

    # Test with invalid RGB list
    with pytest.raises(ValueError):
        parse_color([1.0, 0.0])

    # Test with invalid RGB tuple
    with pytest.raises(ValueError):
        parse_color((1.0, 0.0))

    # Test with invalid named color
    with pytest.raises(ValueError):
        parse_color("notavalidcolor")

    # Test with invalid hexadecimal color format
    with pytest.raises(ValueError):
        parse_color("#ZZZ")

    # Test with unsupported input type
    with pytest.raises(ValueError):
        parse_color(12345)
