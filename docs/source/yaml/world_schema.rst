World Schema
============
Worlds can be specified programmatically or loaded from YAML files as follows:

.. code-block:: python

   from pyrobosim.core.yaml import WorldYamlLoader
   loader = WorldYamlLoader()
   world = loader.from_yaml("/path/to/world.yaml")

The world schema looks as follows, where ``<angle brackets>`` are placeholders:

.. code-block:: yaml

   # General parameters
   params:
      name: <world_name>
      inflation_radius: <value> # Collision padding radius around rooms/hallways/locations
      object_radius: <value> # Collision padding radius around objects
      wall_height: <value> # Vertical (Z) height of walls, can override with individual entities

   # Metadata
   metadata: 
      locations: </path/to/location_data_file.yaml>
      objects: </path/to/object_data_file.yaml>

   # Robots: For now, we only support a single robot
   robots: 
     - id: <value>
       radius: <value> # Robot radius
       location: <name> # Initial location
       pose: [<x>, <y>, <yaw>] # Initial pose, if not specified will sample
       path_planner: # Local robot path planner -- generally refers to single-query planners
         type: rrt # Supported types -- rrt
         <property>: <planner_property>

   # Global path planner -- generally refers to multi-query planners
   global_path_planner:
     type: search_graph # Supported types -- search_graph, prm
     <property>: <planner_property>

   # Rooms
   rooms:
     - name: <room_name> # If not specified, will be automatic
       footprint:
         type: <footprint_type>
         <property>: <footprint_property>
       nav_poses:
         - [<x1>, <y1>, <yaw1>]
         - ...
         - [<xN>, <yN>, <yawN>]
       wall_width: <value>
       color: [<r>, <g>, <b>]
     - ...
     - ...

   # Hallways (refer to Hallways API)
   hallways:
     - from: <room1>
       to: <room2>
       width: <value>
       conn_method: <type>
       <conn_property>: <value>
     - ...
     - ...

   # Locations
   locations:
     - name: <loc_name> # If not specified, will be automatic
       type: <loc_category> # From location YAML file
       room: <room_name>
       pose: [<x>, <y>, <yaw>] # If not specified, will sample
     - ...
     - ...

   # Objects
   objects:
     - name: <obj_name> # If not specified, will be automatic
       type: <obj_category> # From object YAML file
       location: <loc_name>
       pose: [<x>, <y>, <yaw>] # If not specified, will sample

