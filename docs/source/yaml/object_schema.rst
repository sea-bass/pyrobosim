Object Schema
=============

Objects in a world come from a library of categories defined in a YAML file.

The generic object schema, where ``<angle brackets>`` are placeholders, is:

.. code-block:: yaml

   <category_name>:
     footprint:
       type: <footprint_type>
       <property>: <footprint_property>
     color: [<r>, <g>, <b>] or <"color_name"> or <"hexadecimalcode">

Examples
--------

A simple object with a circular footprint.

.. code-block:: yaml

   apple:                   # The category name is "apple"
     footprint:
       type: circle         # Circular footprint
       radius: 0.06         # 6 cm radius
     color: [1, 0, 0]       # Red

A simple object with a box footprint.

.. code-block:: yaml

   banana:                  # Category name is "banana"
     footprint:
       type: box            # Box footprint
       dims: [0.05, 0.2]    # 5 cm by 20 cm
       offset: [0.0, 0.1]   # 10 cm Y offset from origin
     color: "yellow"   # Yellow

An object with a generic polygon footprint.

.. code-block:: yaml

   water:                   # Category name is "water"
     footprint:
       type: polygon        # Generic polygon footprint
       coords:              # List of X-Y coordinates
         - [0.035, -0.075]
         - [0.035, 0.075]
         - [0.0, 0.1]
         - [-0.035, 0.075]
         - [-0.035, -0.075]
       offset: [0.02, 0.0]  # 2 cm X offset from origin
       height: 0.25         # 25 cm extruded height (Z dimension)
     color: [0.0, 0.1, 0.9] # Blue

An object with a footprint read from a mesh file.
Note that the literal ``$DATA`` resolves to the ``pyrobosim/data`` folder, but you can specify an absolute path as well or create your own tokens.

.. code-block:: yaml

   coke:                    # Category name is "coke"
     footprint:
       type: mesh           # Geometry from mesh
       model_path: $DATA/sample_models/coke_can
       mesh_path: meshes/coke_can.dae
     color: "#CC0000"     # Red color (for viewing in pyrobosim)
