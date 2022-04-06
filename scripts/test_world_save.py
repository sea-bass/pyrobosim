#!/usr/bin/env python

""" Tests Gazebo world saving capabilities """

import os
from pyrobosim.utils.general import get_data_folder
from pyrobosim.world.gazebo import WorldGazeboExporter
from pyrobosim.world.yaml import WorldYamlLoader

data_folder = get_data_folder()
loader = WorldYamlLoader()
w = loader.from_yaml(os.path.join(data_folder, "test_world.yaml"))

exp = WorldGazeboExporter(w)
exp.export()
