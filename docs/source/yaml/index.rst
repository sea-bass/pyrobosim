.. _yaml_schemas:

YAML Schemas
============

PyRoboSim worlds can be created using the Python API, but can also loaded from YAML files.

Specifically, each world draws from a set of **location** and **object** metadata files.

Worlds themselves can be created programmatically, or defined using world YAML files.

For the programmatic approach, you can create a world as follows either using ``set_metadata`` or ``add_metadata``.

.. code-block:: python

   from pyrobosim.core import Robot, World

   world = World()

   # locations and objects can accept either a single file path or a list of file paths.
   world.add_metadata(
      locations=[
         "/path/to/location_data1.yaml",
         "/path/to/location_data2.yaml"
      ],
      objects=[
         "/path/to/object_data1.yaml",
         "/path/to/object_data2.yaml"
      ]
   )

   # Then, you can add the other entities
   world.add_robot(...)
   world.add_room(...)
   world.add_hallway(...)
   world.add_location(...)
   world.add_object(...)


For the YAML approach, you can directly point to the location and object metadata in the world YAML file itself.

.. code-block:: yaml

    metadata:
      locations: /path/to/location_data.yaml
      objects: /path/to/object_data.yaml

Then, the world can be loaded from file as follows.

.. code-block:: python

   from pyrobosim.core import WorldYamlLoader

   world = WorldYamlLoader().from_file("/path/to/world_file.yaml")

Refer to the following sections for more details on the schemas.

.. toctree::
   :maxdepth: 1

   world_schema
   location_schema
   object_schema
