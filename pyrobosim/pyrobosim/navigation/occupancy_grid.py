"""Occupancy grid utilities."""

import math
import numpy as np
import matplotlib.pyplot as plt
import os
from PIL import Image
from typing import Sequence
from typing_extensions import Self  # For compatibility with Python <= 3.10
import yaml

from ..core.robot import Robot
from ..core.world import World
from ..utils.logging import get_global_logger
from ..utils.world_collision import check_occupancy


class OccupancyGrid:
    """Lightweight wrapper containing occupancy grid information."""

    def __init__(
        self,
        data: np.ndarray,
        resolution: float,
        origin: Sequence[float] = (0.0, 0.0),
        occ_thresh: float = 0.65,
        free_thresh: float = 0.2,
    ) -> None:
        """
        Creates an occupancy grid.

        :param data: 2D numeric array containing the occupancy data.
        :param resolution: Grid resolution, in meters.
        :param origin: XY position of the grid origin, in meters,
            defaults to (0, 0).
        :param occ_thresh: Probability threshold for a cell being occupied
            (0 to 1), defaults to 0.65.
        :param free_thresh: Probability threshold for a cell being free
            (0 to 1), defaults to 0.2.
        """
        self.data = data
        self.width, self.height = self.data.shape
        self.resolution = resolution
        self.origin = tuple(origin)
        self.occ_thresh = occ_thresh
        self.free_thresh = free_thresh

    def show(self) -> None:
        """Displays the occupancy grid as an image."""
        rot_img = np.logical_not(np.rot90(self.data))
        plt.imshow(rot_img, cmap="gray", interpolation="nearest")
        plt.axis("equal")
        plt.title("Occupancy Grid")

        n_buckets = 8
        x_pos = np.linspace(0, self.width, n_buckets)
        plt.xticks(x_pos, [f"{p*self.resolution + self.origin[0]:.2}" for p in x_pos])
        y_pos = np.linspace(0, self.height, n_buckets)
        plt.yticks(
            y_pos, np.flip([f"{p*self.resolution + self.origin[1]:.2}" for p in y_pos])
        )
        plt.show()

    def is_in_bounds(self, pos: tuple[int, int]) -> bool:
        """
        Check if a given (x,y) position is within grid limits

        :param pos: The position to be validated.
        :return: True if the given coordinates are within bounds, else False.
        """
        x, y = pos
        x_bounds = (x >= 0) and (x < self.width)
        y_bounds = (y >= 0) and (y < self.height)
        return x_bounds and y_bounds

    def world_to_grid(self, pos: tuple[float, float]) -> tuple[int, int]:
        """
        Convert a given world position in world frame to grid frame.

        :param pos: The position to be transformed.
        :return: The coordinates in grid frame.
        """
        x_grid = math.floor((pos[0] - self.origin[0]) / self.resolution)
        y_grid = math.floor((pos[1] - self.origin[1]) / self.resolution)
        return (x_grid, y_grid)

    def grid_to_world(self, pos: tuple[int, int]) -> tuple[float, float]:
        """
        Convert a given world position in grid frame to world frame.

        :param pos: The position to be transformed.
        :return: The coordinates in world frame
        """
        x_world = (pos[0] * self.resolution) + self.origin[0]
        y_world = (pos[1] * self.resolution) + self.origin[1]
        return (x_world, y_world)

    def is_occupied(self, pos: tuple[int, int]) -> bool:
        """
        Check if a given position in the grid is occupied

        :param pos: The position to be checked.
        :return: True if the position is occupied, else False
        """
        return (not self.is_in_bounds(pos)) or (self.data[pos[0], pos[1]] == 1)

    def has_straight_line_connection(
        self, pointA: tuple[int, int], pointB: tuple[int, int]
    ) -> tuple[bool, tuple[int, int]]:
        """
        Checks if 2 points can be connected in a straight line.

        :param pointA: The source point in the grid
        :param pointB: The destination point in the grid
        :return: (True, last_point) if pointA can be connected to pointB, else (False, last_point),
                 last_point is the last point that can be reached from the source in a straight line
                 towards the destination.
                 If pointA and pointB are connectable last_point will be pointB.
        """
        # Note: Left shift operator `<<` is used as an optimization for multiplying by 2.

        x0, y0 = pointA
        x1, y1 = pointB

        dx = x1 - x0
        dy = y1 - y0

        xdir = 1 if dx > 0 else -1
        ydir = 1 if dy > 0 else -1

        dx = abs(dx)
        dy = abs(dy)

        # If we have to take more steps along y axis, switch x-axis with y-axis
        # This is done for easiness of computing in the loop
        axis_switched = False
        if dy > dx:
            dx, dy = dy, dx
            axis_switched = True

        y = 0
        x = 0
        decision = (dy << 1) - dx
        last_point = pointB
        can_connect = True
        for _ in range(dx + 1):
            # handles the increment the same way even if axis has been switched
            x_inc = xdir * (y if axis_switched else x)
            y_inc = ydir * (x if axis_switched else y)
            # Compute the next grid cell
            if self.is_occupied((x0 + x_inc, y0 + y_inc)):
                last_point = (x0 + x_inc, y0 + y_inc)
                can_connect = False
            x += 1
            if decision < 0:
                decision += dy << 1
            else:
                y += 1
                decision += (dy << 1) - (dx << 1)
        return can_connect, last_point

    def save_to_file(self, folder: str, filename: str = "world_map") -> None:
        """
        Save occupancy grid to PGM and YAML files compatible with ROS tools.

        :param folder: Path to output folder.
        :param filename: Name of PGM/YAML file, defaults to "world_map".
        """
        # Write the PGM file.
        pgm_file = os.path.join(folder, filename + ".pgm")
        with open(pgm_file, "wb") as f:
            # Create the PGM file header.
            file_header = f"P5\n# CREATOR: world {self.resolution:.3} m/pix"
            file_header += f"\n{self.width} {self.height}\n255\n"
            f.write(bytearray(file_header, "ascii"))

            # Write the occupancy grid information to the PGM file.
            for h in reversed(range(self.height)):
                for w in range(self.width):
                    occ = self.data[w, h]
                    if occ >= self.occ_thresh:
                        occ_val = 0
                    elif occ <= self.free_thresh:
                        occ_val = 254
                    else:
                        occ_val = 205
                    f.write(occ_val.to_bytes(1, "big"))

            f.close()

        # Write the YAML file.
        yaml_file = os.path.join(folder, filename + ".yaml")
        x_orig, y_orig = self.origin
        yaml_dict = {
            "image": pgm_file,
            "resolution": self.resolution,
            "origin": [x_orig, y_orig, 0],
            "negate": 0,
            "occupied_thresh": self.occ_thresh,
            "free_thresh": self.free_thresh,
        }
        with open(yaml_file, "w") as f:
            yaml.dump(yaml_dict, f, sort_keys=False, default_flow_style=None)

    @classmethod
    def from_file(cls, folder: str, filename: str | None = None) -> Self:
        """
        Loads an occupancy grid from a folder containing a PGM image file and a YAML file.

        :param folder: Path to folder.
        :param filename: Name of YAML file, defaults to None.
            If None is specified, this function will try to find the file using the .yaml extension.
        """
        all_files = os.listdir(folder)

        # Read the YAML file
        if filename:
            yaml_path = os.path.join(folder, f"{filename}.yaml")
        else:
            yaml_files = [
                file
                for file in all_files
                if file.endswith(".yaml") or file.endswith(".yml")
            ]
            if len(yaml_files) == 0:
                raise FileNotFoundError(
                    "No YAML files found. Cannot load occupancy grid."
                )
            elif len(yaml_files) > 1:
                get_global_logger().warning(
                    "Found multiple YAML files. Loading the first one."
                )
            yaml_path = os.path.join(folder, yaml_files[0])

        with open(yaml_path, "r") as f:
            yaml_dict = yaml.load(f, yaml.FullLoader)
            image_path = yaml_dict["image"]
            resolution = yaml_dict["resolution"]
            origin = yaml_dict["origin"][:2]
            occ_thresh = yaml_dict["occupied_thresh"]
            free_thresh = yaml_dict["free_thresh"]

        # Read the image file and process the data
        grid_data = np.array(Image.open(image_path), dtype=np.float64)
        grid_data = np.rot90(grid_data, k=-1)
        grid_data[grid_data == 0] = 1.0
        grid_data[grid_data == 254] = 0.0
        grid_data[grid_data == 205] = 0.5 * (occ_thresh + free_thresh)

        # Create and return an Occupancy grid from the data above
        return OccupancyGrid(grid_data, resolution, origin, occ_thresh, free_thresh)  # type: ignore[return-value]

    @classmethod
    def from_world(
        cls,
        world: World,
        resolution: float,
        inflation_radius: float = 0.0,
        robot: Robot | None = None,
        xlim: tuple[float, float] | None = None,
        ylim: tuple[float, float] | None = None,
        auto_lim_padding_ratio: float = 0.05,
    ) -> Self:
        """
        Generates an occupancy grid of a world at a given resolution.

        Can optionally specify (min, max) x and y limits. If they are
        left unspecified, the extents will be calculated automatically.

        :param world: World object from which to create an occupancy grid.
        :param resolution: Grid resolution, in meters.
        :param inflation_radius: Inflation radius, in meters.
        :param robot: Robot instance used for checking collisions.
        :param xlim: X coordinate limits, in meters.
        :param ylim: Y coordinate limits, in meters.
        :param auto_lim_padding_ratio: Additional padding ratio outside world
            limits if automatically computed, defaults to 0.05.
        :return: Occupancy grid of the world.
        """
        # If limits are not specified, use the world limits, but slightly padded.
        if xlim is None:
            if world.x_bounds is None:
                raise RuntimeError("World bounds are not yet set.")
            x_padding = (world.x_bounds[1] - world.x_bounds[0]) * auto_lim_padding_ratio
            x_limits = (world.x_bounds[0] - x_padding, world.x_bounds[1] + x_padding)
        else:
            x_limits = xlim

        if ylim is None:
            if world.y_bounds is None:
                raise RuntimeError("World bounds are not yet set.")
            y_padding = (world.y_bounds[1] - world.y_bounds[0]) * auto_lim_padding_ratio
            y_limits = (world.y_bounds[0] - y_padding, world.y_bounds[1] + y_padding)
        else:
            y_limits = ylim

        # Temporarily change collision polygons to specified inflation radius
        orig_inflation_radius = world.inflation_radius
        world.set_inflation_radius(inflation_radius)

        # Generate the occupancy grid
        xrange = np.arange(x_limits[0], x_limits[1] + resolution, resolution)
        yrange = np.arange(y_limits[0], y_limits[1] + resolution, resolution)
        nx = xrange.shape[0]
        ny = yrange.shape[0]
        occupancy_grid_data = np.zeros((nx, ny))
        for i in range(nx):
            x = xrange[i]
            for j in range(ny):
                y = yrange[j]
                occupancy_grid_data[i, j] = check_occupancy((x, y), world, robot)
        origin = (x_limits[0], y_limits[0])

        # Reset collision polygons to original inflation radius
        world.set_inflation_radius(orig_inflation_radius)

        return OccupancyGrid(occupancy_grid_data, resolution, origin)  # type: ignore[return-value]


def reduce_waypoints_grid(
    grid: OccupancyGrid, positions: list[tuple[int, int]]
) -> list[tuple[int, int]]:
    """
    Reduces the number of waypoints in a generated path from a grid-based planner.

    :param grid: The occupancy grid associated with the generated path.
    :param positions: The list of positions that make up the path.
    :return: The optimized list of waypoints.
    """
    waypoints = []
    start = positions[0]
    waypoints.append(start)
    positions = positions[1:]
    i = len(positions) - 1
    while positions and (i >= 0):
        current = positions[i]
        if grid.has_straight_line_connection(start, current)[0]:
            waypoints.append(current)
            start = current
            positions = positions[i + 1 :]
            i = len(positions) - 1
        else:
            i -= 1
    return waypoints
