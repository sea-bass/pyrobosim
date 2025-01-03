.. _yaml_schemas:

YAML Schemas
============

``pyrobosim`` relies on YAML files to define entities in the world.

Specifically, each world draws from a set of **location** and **object** metadata files.

Worlds themselves can be created programmatically, or defined using world YAML files.

For the programmatic approach, you can create a world as follows.

**Option 1** : Using ``set_metadata`` : Supports specifying metadata with a single YAML file for locations and one for objects.

.. code-block:: python

   from pyrobosim.core import Robot, World

   world = World()
   world.set_metadata(locations="/path/to/location_data.yaml",
                      objects="/path/to/object_data.yaml")

   # Then, you can add the other entities
   world.add_robot(...)
   world.add_room(...)
   world.add_hallway(...)
   world.add_location(...)
   world.add_object(...)

**Option 2** : Using ``add_metadata`` : Supports specifying metadata with multiple YAML files for locations and for objects.

.. code-block:: python

   from pyrobosim.core import Robot, World

   world = World()
   world.add_metadata(locations="/path/to/location_data1.yaml",
                      objects="/path/to/object_data1.yaml")

   world.add_metadata(locations=["/path/to/location_data2.yaml"],
                      objects=[
                        "/path/to/object_data2.yaml",
                        "/path/to/object_data3.yaml",
                      ])

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
