""" Occupancy map utilities """

import os
import yaml
import numpy as np
import matplotlib.pyplot as plt

class OccupancyGrid:
    """ Lightweight wrapper containing occupancy grid information """

    def __init__(self, data, resolution, origin=(0, 0), occ_thresh=0.65, free_thresh=0.2):
        self.data = data
        self.width, self.height = self.data.shape
        self.resolution = resolution
        self.origin = origin
        self.occ_thresh = occ_thresh
        self.free_thresh = free_thresh

    def show(self):
        """ Displays the occupancy grid as an image """
        rot_img = np.logical_not(np.rot90(self.data))
        plt.imshow(rot_img, 
                   cmap="gray", interpolation="nearest")
        plt.axis("equal")
        plt.title("Occupancy Grid")

        n_buckets = 8
        x_pos = np.linspace(0, self.width, n_buckets)
        plt.xticks(
            x_pos, [f"{p*self.resolution + self.origin[0]:.2}" for p in x_pos])
        y_pos = np.linspace(0, self.height, n_buckets)
        plt.yticks(y_pos, np.flip(
            [f"{p*self.resolution + self.origin[1]:.2}" for p in y_pos]))
        plt.show()

    def save_to_file(self, folder, filename="world_map"):
        """ Save occupancy grid to PGM and YAML files compatible with ROS tools. """

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
        yaml_dict = {"image": pgm_file,
                     "resolution": self.resolution,
                     "origin": [x_orig, y_orig, 0],
                     "negate": 0,
                     "occupied_thresh": self.occ_thresh,
                     "free_thresh": self.free_thresh}
        with open(yaml_file, "w") as f:
            yaml.dump(yaml_dict, f, sort_keys=False, default_flow_style=None)


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
