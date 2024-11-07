World Schema
============

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
       height: <value>  # Robot height
       location: <loc_name>  # Initial location
       pose:  # Initial pose, if not specified will sample
         position:
           x: <x>
           y: <y>
         rotation_eul:
           yaw: <yaw>
       initial_battery_level: 50.0
       # Dynamics limits
       max_linear_velocity: <value>
       max_angular_velocity: <value>
       max_linear_acceleration: <value>
       max_angular_acceleration: <value>
       # Specialized capabilities
       path_planner:  # Local robot path planner -- generally refers to single-query planners
         type: rrt  # Supported types -- astar, rrt, prm, world_graph
         <property>: <planner_property>
       path_executor:  # For following a path
         type: constant_velocity  # Supported types -- constant_velocity
         <property>: <path_executor_property>
       grasp_generator:  # For object grasp generation
         type: parallel_grasp  # Supported types -- parallel_grasp
         <property>: <grasp_generator_property>
       partial_observability: False  # If True, robot starts with no detected objects
       # Options to control the execution of actions
       action_execution_options:
         navigate:
           delay: 0.1
           success_probability: 0.9
           rng_seed: 42
           battery_usage: 1.0
         pick:
           delay: 1.0
           battery_usage: 5.0
         place:
           success_probability: 0.75
           battery_usage: 5.0
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

   # Hallways
   hallways:
     - room_start: <room1>
       room_end: <room2>
       width: <value>
       conn_method: <type>
       <conn_property>: <value>
       is_open: true  # Can only navigate through hallway if open
       is_locked: false  # Can only open and close if unlocked
     - ...
     - ...

   # Locations
   locations:
     - name: <loc_name>  # If not specified, will be automatic
       category: <loc_category>  # From location YAML file
       parent: <room_name>
       pose:  # If not specified, will sample
        position:
          x: <x>
          y: <y>
        rotation_eul:
          yaw: <yaw>
       is_open: true  # Can only pick, place, and detect if open
       is_locked: true  # Can only open and close if unlocked
       is_charger: false  # Robots can charge at this location
     - ...
     - ...

   # Objects
   objects:
     - name: <obj_name>  # If not specified, will be automatic
       category: <obj_category>  # From object YAML file
       parent: <loc_name>
       pose:  # If not specified, will sample
         position:
           x: <x>
           y: <y>
         rotation_quat:
           w: <w>
           x: <x>
           y: <y>
           z: <z>

Note that you can use both Euler angles and quaternions to specify poses.
Any unspecified values will default to ``0.0``.
