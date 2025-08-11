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
      locations:
       - <path/to/location_data1_file.yaml>
       - <path/to/location_data2_file.yaml>
       - ...
       - <path/to/location_dataN_file.yaml>
      objects:
       - </path/to/object_data1_file.yaml>
       - </path/to/object_data2_file.yaml>
       - ...
       - </path/to/object_dataN_file.yaml>

   # Robots: Each robot contains basic properties, plus other add-ons such as path planners and grasp generators
   robots:
     - name: <name>
       radius: <value>  # Robot radius
       height: <value>  # Robot height
       location: <loc_name>  # Initial location (can also be a list)
       pose:  # Initial pose, if not specified will sample
         position:
           x: <x>
           y: <y>
         rotation_eul:
           yaw: <yaw>
           angle_units: <units>  # Can be "radians" (default) or "degrees"
       initial_battery_level: 50.0
       partial_obs_objects: False  # If True, robot starts with no detected objects
       partial_obs_hallways: False  # If True, robot starts with all hallways detected open
       # Dynamics limits
       max_linear_velocity: <value>
       max_angular_velocity: <value>
       max_linear_acceleration: <value>
       max_angular_acceleration: <value>
       # Specialized capabilities
       path_planner:  # Path planners for robot navigation
         type: rrt  # Supported types -- astar, rrt, prm, world_graph
         <property>: <planner_property>
       path_executor:  # For following a path
         type: constant_velocity  # Supported types -- constant_velocity
         <property>: <path_executor_property>
       grasp_generator:  # For object grasp generation
         type: parallel_grasp  # Supported types -- parallel_grasp
         <property>: <grasp_generator_property>
       sensors:  # Dictionary of named sensors
         lidar:
           type: lidar
           update_rate_s: 0.1
           angle_units: degrees
           min_angle: -120.0
           max_angle: 120.0
           angular_resolution: 5.0
           max_range_m: 2.0
         # ... other named sensors can go here
       start_sensor_threads: true  # If True, automatically starts sensor threads
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
       pose:  # If not specified, will use the room's centroid
         position:
           x: <x>
           y: <y>
         rotation_eul:
           yaw: <yaw>
       nav_poses:
         - [<x1>, <y1>, <z1>, <yaw1>]
         - ...
         - [<xN>, <yN>, <zN>, <yawN>]
       wall_width: <value>
       color: [<r>, <g>, <b>] or <"color_name"> or <"hexadecimalcode">
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
          angle_units: <units>  # Can be "radians" (default) or "degrees"
        relative_to: <room_name>  # If not specified, uses absolute pose
       is_open: true  # Can only pick, place, and detect if open
       is_locked: true  # Can only open and close if unlocked
       is_charger: false  # Robots can charge at this location
     - ...
     - ...

   # Objects
   objects:
     - name: <obj_name>  # If not specified, will be automatic
       category: <obj_category>  # From object YAML file
       parent: <loc_name>  # Initial location (can also be a list)
       pose:  # If not specified, will sample
         position:
           x: <x>
           y: <y>
         rotation_quat:
           w: <w>
           x: <x>
           y: <y>
           z: <z>
         relative_to: <loc_name>  # If not specified, uses absolute pose


Specifying Poses
----------------

There are a few ways to specify poses in PyRoboSim YAML files: lists and dictionaries.

.. code-block:: yaml

   # Valid list formats
   pose: [<x>, <y>]
   pose: [<x>, <y>, <z>]
   pose: [<x>, <y>, <z>, <yaw>]  # Angle units always in radians

If possible, you should use the dictionary format, as the list format is at this point only around for backward compatibility.
Anything below this line is only supported in dictionary format.

Note that you can use both Euler angles and quaternions to specify poses.
If specifying rotation using Euler angles, you can specify angle either in radians or degrees.
Any unspecified values will default to ``0.0``.

.. code-block:: yaml

   # Euler angles in radians (default), fully specified
   pose:
     position:
       x: 1.0
       y: 2.0
       z: 3.0
     rotation_eul:
       yaw: 0.1
       pitch: 0.2
       roll: 0.3

   # Euler angles in degrees, partially specified
   pose:
     position:
       x: 1.0
       y: 2.0
     rotation_eul:
       yaw: 45.0
       angle_units: "degrees"

   # Quaternion
   pose:
     position:
       x: 1.0
       y: 2.0
     rotation_quat:
       w: 0.707
       x: 0.0
       y: 0.0
       z: -0.707

You can also use the ``relative_to`` field when specifying poses.
This makes it easier to specify poses relative to other entities in the world (rooms, locations, objects, etc.).

.. code-block:: yaml

   pose:
     position:
       x: 1.0
       y: 2.0
     rotation_eul:
       yaw: 45.0
       angle_units: "degrees"
     relative_to: "table0"
