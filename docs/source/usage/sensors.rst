.. _sensors:

Sensors
=======

This section explains how to use (and optionally extend) sensor models in PyRoboSim.

Sensor Definitions
------------------

The ``pyrobosim/sensors`` module contains all sensor model implementations.

In the ``sensor_registry.py`` file available, you will see a list of implemented planner classes that looks like this:

.. code-block:: python

    # Sensors
    from .lidar import Lidar2D

    SENSORS_MAP = {
        "lidar": Lidar2D,
    }

When loading sensors from YAML, the ``sensor.type`` entry will correspond to one of these entries in the ``SENSORS_MAP`` dictionary.
As such, if you would like to add your own sensor, you can do so in this file.

.. code-block:: python

    # Sensors
    from .lidar import Lidar2D
    from my_module.my_file import MyNewSensor

    SENSORS_MAP = {
        "lidar": Lidar2D,
        "my_sensor": MyNewSensor,
    }


What to Implement in a Sensor
------------------------------

The sensors implemented in PyRoboSim provide general functionality to simulate a sensor, as well as data for visualization in the UI.

TODO: Fill this in!

If you would like to implement your own sensor, it is highly recommended to look at the existing sensor implementations as a reference.
You can also always ask the maintainers through a Git issue!
