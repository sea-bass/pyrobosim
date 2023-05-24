World Schema
============
Worlds can be specified programmatically or loaded from YAML files as follows:

.. code-block:: python

   from pyrobosim.core import WorldYamlLoader
   loader = WorldYamlLoader()
   world = loader.from_yaml("/path/to/world.yaml")

The world schema looks as follows, where ``<angle brackets>`` are placeholders:

.. code-block:: yaml

   # General parameters
   params:
      name: <world_name>
      inflation_radius: <value>  # Collision padding radius around rooms/hallways/locations
      object_radius: <value>  # Collision padding radius around objects
      wall_height: <value>  # Vertical (Z) height of walls, can override with individual entities

   # Metadata
   metadata:
      locations: </path/to/location_data_file.yaml>
      objects: </path/to/object_data_file.yaml>

   # Robots: Each robot contains basic properties, plus other add-ons such as path planners and grasp generators
   robots:
     - name: <name>
       radius: <value>  # Robot radius
       location: <loc_name>  # Initial location
       pose: [<x>, <y>, <z>, <yaw>]  # Initial pose, if not specified will sample
       path_planner:  # Local robot path planner -- generally refers to single-query planners
         type: rrt  # Supported types -- astar, rrt, prm, world_graph
         <property>: <planner_property>
       path_executor:  # For following a path
         type: constant_velocity  # Supported types -- constant_velocity
         <property>: <path_executor_property>
       grasp_generator:  # For object grasp generation
         type: parallel_grasp  # Supported types -- parallel_grasp
         <property>: <grasp_generator_property>
     - ...
     - ...

   # Rooms
   rooms:
     - name: <room_name>  # If not specified, will be automatic
       footprint:
         type: <footprint_type>
         <property>: <footprint_property>
       nav_poses:
         - [<x1>, <y1>, <z1>, <yaw1>]
         - ...
         - [<xN>, <yN>, <zN>, <yawN>]
       wall_width: <value>
       color: [<r>, <g>, <b>]
     - ...
     - ...

   # Hallways (refer to Hallways API)
   hallways:
     - room_start: <room1>
       room_end: <room2>
       width: <value>
       conn_method: <type>
       <conn_property>: <value>
     - ...
     - ...

   # Locations
   locations:
     - name: <loc_name>  # If not specified, will be automatic
       category: <loc_category>  # From location YAML file
       parent: <room_name>
       pose: [<x>, <y>, <z>, <yaw>]  # If not specified, will sample
     - ...
     - ...

   # Objects
   objects:
     - name: <obj_name>  # If not specified, will be automatic
       category: <obj_category>  # From object YAML file
       parent: <loc_name>
       pose: [<x>, <y>, <z>, <yaw>]  # If not specified, will sample
