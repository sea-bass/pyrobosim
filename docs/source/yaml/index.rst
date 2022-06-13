YAML Schemas
============
pyrobosim relies on YAML files to define entities in the world.

Specifically, each world draws from a set of **locations** and **objects**. 
Worlds themselves can be created programmatically, or defined using their own 
YAML files.

For the programmatic approach, 

.. code-block:: python

   world.set_metadata(locations="location_data.yaml",
                      objects="object_data.yaml")

For the YAML approach, you can define this in the schema,

.. code-block:: yaml

    metadata:
      locations: </path/to/location_data_file.yaml>
      objects: </path/to/object_data_file.yaml>

Refer to the following sections for more details on the schemas

.. toctree::
   :maxdepth: 1

   world_schema
   location_schema
   object_schema
