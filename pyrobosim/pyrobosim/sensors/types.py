"""
Basic types for sensor simulation.
"""

from matplotlib.artist import Artist
from threading import Thread
from typing import Any


class Sensor:
    """
    Generic sensor class that helps with type hinting.

    When implementing a new sensor, you should subclass from this class.
    """

    def __init__(self) -> None:
        from ..core.robot import Robot

        self.robot: Robot | None = None
        self.thread = Thread(target=self.thread_function)

    def update(self) -> None:
        """
        Performs the sensor calculation.

        Must be implemented in your sensor implementation.
        """
        raise NotImplementedError("Must implement update()!")

    def get_measurement(self) -> Any:
        """
        Returns the latest measurement from the sensor.

        Must be implemented in your sensor implementation.

        :return: The latest measurement.
        """
        raise NotImplementedError("Must implement get_measurement()!")

    def thread_function(self) -> None:
        """
        Defines the sensor update function to run in a background thread.

        If the sensor has nothing to do in the background, you can leave this unimplemented in your sensor.
        """
        return

    def start_thread(self) -> None:
        """
        Starts the thread defined by your sensor's implementation of `thread_function()`.
        """
        if self.thread is not None:
            self.thread.start()

    def stop_thread(self) -> None:
        """
        Stops the actively running sensor thread.

        You should not need to override this function, so long as your sensor's
        implementation of `thread_function()` has a way to stop when the
        robot's `sensors_active` attribute becomes `False` on deletion.
        """
        if (self.thread is not None) and self.thread.is_alive():
            self.thread.join()

    def setup_artists(self) -> list[Artist]:
        """
        Sets up and returns the artists for visualizing the sensor.

        :return: The list of MatPlotLib artists for the sensor.
        """
        return []

    def update_artists(self) -> None:
        """
        Updates the artists.

        These should have been originally returned by `setup_artists()`.
        """

    def to_dict(self) -> dict[str, Any]:
        """
        Serializes the sensor to a dictionary.

        :return: A dictionary containing the sensor information.
        """
        raise NotImplementedError("Must implement to_dict()!")
