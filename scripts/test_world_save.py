#!/usr/bin/env python

""" Tests Gazebo world saving capabilities """

import os
from pyrobosim.navigation.occupancy_grid import occupancy_grid_from_world
from pyrobosim.utils.general import get_data_folder
from pyrobosim.world.gazebo import WorldGazeboExporter
from pyrobosim.world.yaml import WorldYamlLoader

# Load a test world from YAML file.
data_folder = get_data_folder()
loader = WorldYamlLoader()
w = loader.from_yaml(os.path.join(data_folder, "test_world.yaml"))

# Export a Gazebo world.
exp = WorldGazeboExporter(w)
out_folder = exp.export()

# Save an occupancy grid to the world folder.
occ_grid = occupancy_grid_from_world(w, resolution=0.05, inflation_radius=0.05)
occ_grid.save_to_file(out_folder)
occ_grid.show()
