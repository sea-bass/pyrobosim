"""
Logging utilities for PyRoboSim.
"""

import logging


def create_logger(name, level=logging.INFO):
    """
    Defines a general logger for use with PyRoboSim.

    :param name: The name of the logger.
    :type name: str
    :param level: The level of the logger.
    :type level: int
    :return: A logger instance.
    :rtype: :class:`logging.Logger`.
    """
    logger = logging.getLogger(name)
    logger.setLevel(level)

    # TODO: Consider configuring console vs. file logging at some point.
    if not logger.hasHandlers():  # Prevents adding duplicate handlers
        log_formatter = logging.Formatter("[%(name)s] %(levelname)s: %(message)s")
        console_handler = logging.StreamHandler()
        console_handler.setFormatter(log_formatter)
        logger.addHandler(console_handler)

    # Needed to propagate to unit tests via the caplog fixture.
    logger.propagate = True

    return logger


PYROBOSIM_GLOBAL_LOGGER = create_logger("pyrobosim")


def get_global_logger():
    """
    Returns the global logger for PyRoboSim.

    :return: The PyRoboSim global logger instance.
    :rtype: :class:`logging.Logger`.
    """
    return PYROBOSIM_GLOBAL_LOGGER


def set_global_logger(logger):
    """
    Sets the global PyRoboSim logger to another specified logger.

    :param logger: The logger to use as the new global logger.
    :type logger: :class:`logging.Logger`
    """
    global PYROBOSIM_GLOBAL_LOGGER
    PYROBOSIM_GLOBAL_LOGGER = logger
