""" ROS 2 enabled 2D mobile robot simulator for behavior prototyping. """

import logging

# Define a general PyRoboSim logger.
global_logger = logging.getLogger("pyrobosim")
global_logger.setLevel(logging.INFO)
log_formatter = logging.Formatter("[%(name)s] %(levelname)s: %(message)s")
console_handler = logging.StreamHandler()
console_handler.setFormatter(log_formatter)
global_logger.addHandler(console_handler)
global_logger.propagate = True


def get_global_logger():
    """
    Returns a global logger for PyRoboSim.

    :return: The PyRoboSim global logger instance.
    :rtype: :class:`logging.Logger`.
    """
    return global_logger
