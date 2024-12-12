Location Schema
===============

Locations in a world come from a library of categories defined in a YAML file.

The generic location schema, where ``<angle brackets>`` are placeholders, is:

.. code-block:: yaml

   <category_name>:
     footprint:
       type: <footprint_type>
       <property>: <footprint_property>
     nav_poses:
       - position:
           x: <x1>
           y: <y1>
         rotation_eul:
           yaw: <yaw1>
           angle_units: "degrees"
       - position:
           x: <x2>
           y: <y2>
         rotation_quat:
           w: <w2>
           x: <x2>
           y: <y2>
           z: <z2>
       -  ...
       - position:
           x: <xN>
           y: <yN>
           z: <zN>
         rotation_eul:
           yaw: <yawN>
           angle_units: <units>
     locations:
       - name: <location_name>
         footprint:
           type: <footprint_type>
           <property>: <footprint_property>
       - ...
       - name: <other_location_name>
         footprint:
           type: <footprint_type>
           <property>: <footprint_property>
     color: [<r>, <g>, <b>] or <"color_name"> or <"hexadecimalcode">
     is_open: True
     is_locked: False
     is_charger: False


Examples
--------

A simple object with a box footprint and one object spawn that inherits parent geometry.

.. code-block:: yaml

   table:                       # Category name is "table"
     footprint:                 # The overall location footprint is this box
       type: box
       dims: [0.9, 1.2]
       height: 0.5              # 0.5 m off the ground
     nav_poses:                 # List of navigation poses around the location origin
       - [-0.75, 0, 0]          # (the children below will inherit the parent's poses)
       - [0.75, 0, 0.0, 3.14]
     locations:                 # List of locations
       - name: "tabletop"       # The location name will be "<loc_name>_tabletop"
         footprint:
           type: parent         # "parent" footprint means we inherit parent's box geometry
           padding: 0.1         # 10 cm padding relative to the parent geometry
     color: [0.2, 0, 0]         # Dark red

A more complex object with a box footprint and two separate object spawns.

.. code-block:: yaml

   counter:                     # Category name is "counter"
     footprint:                 # The overall location footprint is this box
       type: box
       dims: [1.2, 0.6]
       height: 0.75             # 0.75 m off the ground
     locations:                 # List of locations with their own geometries and navigation poses
       - name: "left"           # The location name will be "<loc_name>_left"
         footprint:
           type: polygon
           coords:
             - [-0.25, -0.25]
             - [0.25, -0.25]
             - [0.25, 0.25]
             - [-0.25, 0.25]
           offset: [0.3, 0]
         nav_poses:
           - [0, 0.5, 0.0, -1.57]
           - [0, -0.5, 0.0, 1.57]
       - name: "right"         # The location name will be "<loc_name>_right"
         footprint:
           type: polygon
           coords:
             - [-0.25, -0.25]
             - [0.25, -0.25]
             - [0.25, 0.25]
             - [-0.25, 0.25]
           offset: [-0.3, 0]
         nav_poses:
           - [0, 0.5, 0.0, -1.57]
           - [0, -0.5, 0.0, 1.57]
     color: "#003300"        # Dark green

A location with a footprint read from a mesh file.
Note that the literal ``$DATA`` resolves to the ``pyrobosim/data`` folder, but you can specify an absolute path as well or create your own tokens.

.. code-block:: yaml

   trash_can:               # Category name is "trash_can"
     footprint:
       type: mesh           # Mesh footprint takes the 2D convex hull
       model_path: $DATA/sample_models/first_2015_trash_can
       mesh_path: meshes/trash_can.dae
     locations:             # Single location inherits from parent with 5 cm padding
       - name: "top"        # The location name will be "<loc_name>_top"
         footprint:
           type: parent
           padding: 0.05
     nav_poses:             # Navigation poses must still be specified manually
       - [0.5, 0.0, 0.0, 3.14]
       - [-0.5, 0.0, 0.0, 0.0]
     color: [0, 0.35, 0.2]  # Greenish-blue
