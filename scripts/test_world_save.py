#!/usr/bin/env python

""" Tests Gazebo world saving capabilities """

import os
from pyrobosim.navigation.occupancy_grid import occupancy_grid_from_world
from pyrobosim.utils.general import get_data_folder
from pyrobosim.world.gazebo import WorldGazeboExporter
from pyrobosim.world.yaml import WorldYamlLoader

data_folder = get_data_folder()
loader = WorldYamlLoader()
w = loader.from_yaml(os.path.join(data_folder, "test_world.yaml"))

exp = WorldGazeboExporter(w)
exp.export()

occ_grid = occupancy_grid_from_world(w, resolution=0.05, inflation_radius=0.05)
occ_grid.show()
