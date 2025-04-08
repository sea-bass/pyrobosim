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

First, you want subclass from the ``Sensor`` class and create a constructor as follows.

.. code-block:: python

    from pyrobosim.sensors.types import Sensor

    class MyNewSensor(Sensor):

        def __init__(
            self,
            *,
            update_rate_s: float,
            initial_value: float,
        ) -> None:
            super().__init__()
            self.update_rate_s = update_rate_s
            self.latest_value = initial_value


Next, you should implement ``update()`` and ``get_measurement()`` functions.
These update the internals of the sensor and return the latest measurement, respectively.

.. code-block:: python

        def update(self) -> None:
            # Increments the sensor value by 1.
            self.latest_value += 1.0

        def get_measurement(self) -> float:
            return self.latest_value


If you want to run the sensor automatically in the background, you can also implement a ``thread_function()`` function.
This will only take effect if your robot is created with the ``start_sensor_threads`` argument set to ``True``.

.. code-block:: python

    def thread_function(self) -> None:
        if self.robot is None:  # This is created in the constructor!
            return

        # The `sensors_active` attribute of the robot should be used to cleanly
        # stop this thread when the robot is shut down.
        while self.robot.sensors_active:
            t_start = time.time()
            self.update()
            t_end = time.time()
            time.sleep(max(0.0, self.update_rate_s - (t_end - t_start)))


For visualization, you can provide ``setup_artists()`` and ``update_artists()`` methods.

.. code-block:: python

    from matplotlib.artist import Artist
    from matplotlib.patches import Circle
    from matplotlib.transforms import Affine2D

    def setup_artists(self) -> list[Artist]:
        """Executes when the sensor is first visualized."""
        pose = self.robot.get_pose()
        self.circle = Circle(
            (pose.x, pose.y),
            radius=1.0,
            color="r",
        )
        return [self.circle]

    def update_artists(self) -> None:
        """Updates the artist as needed."""
        pose = self.robot.get_pose()
        new_tform = Affine2D().translate(pose.x, pose.y)
        self.circle.set_transform(new_tform)


To serialize to file, which is needed to reset the world, you should also implement the ``to_dict()`` method.
Note the ``get_sensor_string()`` helper function, which extracts the name of the sensor you defined in ``SENSORS_MAP`` earlier on.

.. code-block:: python

        def to_dict(self) -> dict[str, Any]:
            from pyrobosim.sensors.sensor_registry import get_sensor_string

            return {
                "type": get_sensor_string(self),
                "update_rate_s": self.update_rate_s,
                "initial_value": self.initial_value,
            }


If you would like to implement your own sensor, it is highly recommended to look at the existing sensor implementations as a reference.
You can also always ask the maintainers through a Git issue!
