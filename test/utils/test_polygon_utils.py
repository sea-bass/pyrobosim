#!/usr/bin/env python3

"""
Unit tests for polygon utilities.
"""

import pytest
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
from scipy.spatial import ConvexHull

from pyrobosim.utils.polygon import convhull_to_rectangle


def test_rectangle_from_object_footprint(display=False):
    # Create a cross-shaped object
    xy_pts = np.array(
        [
            [-0.05, -0.15],
            [0.05, -0.15],
            [0.05, -0.05],
            [0.15, -0.05],
            [0.15, 0.05],
            [0.05, 0.05],
            [0.05, 0.15],
            [-0.05, 0.15],
            [-0.05, 0.05],
            [-0.15, 0.05],
            [-0.15, -0.05],
            [-0.05, -0.05],
            [-0.05, -0.15],
        ]
    )

    # Then, gets its convex hull and a best-fit rectangle
    hull = ConvexHull(xy_pts)
    hull_pts = np.vstack([xy_pts[hull.vertices, :], xy_pts[hull.vertices[0], :]])
    (rect_pose, rect_dims, rect_pts) = convhull_to_rectangle(hull_pts)

    # Check the rectangle origin and dimension
    assert rect_pose.x == pytest.approx(0.0)
    assert rect_pose.y == pytest.approx(0.0)
    assert rect_dims == pytest.approx([0.2 * np.sqrt(2), 0.2 * np.sqrt(2)])

    # Display the outputs
    if display:
        fig, ax = plt.subplots()
        poly = Polygon(xy_pts)
        ax.add_patch(poly)
        plt.plot(hull_pts[:, 0], hull_pts[:, 1], "r--", lw=2)
        plt.plot(rect_pts[:, 0], rect_pts[:, 1], "b:", lw=2)
        plt.legend(["Polygon", "Convex Hull", "Rectangle Fit"])
        plt.axis("equal")
        plt.show()


if __name__ == "__main__":
    test_rectangle_from_object_footprint(display=True)
