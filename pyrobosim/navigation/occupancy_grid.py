""" Occupancy map utilities """

import numpy as np
import matplotlib.pyplot as plt

class OccupancyGrid:
    """ Lightweight wrapper containing occupancy grid information """
    def __init__(self, data, resolution, origin):
        self.data = data
        self.resolution = resolution
        self.origin = origin

    def show(self):
        """ Displays the occupancy grid as an image """
        rot_img = np.logical_not(np.rot90(self.data))
        plt.imshow(rot_img, 
                   cmap="gray", interpolation="nearest")
        plt.axis("equal")
        plt.title("Occupancy Grid")

        n_buckets = 8
        x_pos = np.linspace(0, rot_img.shape[1], n_buckets)
        plt.xticks(
            x_pos, [f"{p*self.resolution + self.origin[0]:.2}" for p in x_pos])
        y_pos = np.linspace(0, rot_img.shape[0], n_buckets)
        plt.yticks(y_pos, np.flip(
            [f"{p*self.resolution + self.origin[1]:.2}" for p in y_pos]))
        plt.show()

    def save_to_file(self, path):
        """ Save occupancy grid to file """
        raise NotImplementedError("Occupancy grid saving not implemented yet")


def occupancy_grid_from_world(world, resolution,
                              inflation_radius=0.0,
                              xlim=None, ylim=None):
    """
    Generates an occupancy grid of a world at a given resolution.

    Can optionally specify (min, max) x and y limits. If they are 
    left unspecified, the extents will be calculated automatically.
    """
    # Use the world limits if not specified, but slightly padded
    auto_lim_padding_ratio = 0.05
    if xlim is None:
        x_padding = (world.x_bounds[1] - world.x_bounds[0]) * auto_lim_padding_ratio
        xlim = (world.x_bounds[0] - x_padding, world.x_bounds[1] + x_padding)
    if ylim is None:
        y_padding = (world.y_bounds[1] - world.y_bounds[0]) * auto_lim_padding_ratio
        ylim = (world.y_bounds[0] - y_padding, world.y_bounds[1] + y_padding)

    # Temporarily change collision polygons to specified inflation radius
    orig_inflation_radius = world.inflation_radius
    world.set_inflation_radius(inflation_radius)

    # Generate the occupancy grid
    xrange = np.arange(xlim[0], xlim[1] + resolution, resolution)
    yrange = np.arange(ylim[0], ylim[1] + resolution, resolution)
    nx = xrange.shape[0]
    ny = yrange.shape[0]
    occupancy_grid_data = np.zeros((nx, ny))
    for i in range(nx):
        x = xrange[i]
        for j in range(ny):
            y = yrange[j]
            occupancy_grid_data[i, j] = world.check_occupancy((x, y))
    origin = (xlim[0], ylim[0])

    # Reset collision polygons to original inflation radius
    world.set_inflation_radius(orig_inflation_radius)

    return OccupancyGrid(occupancy_grid_data, resolution, origin)
