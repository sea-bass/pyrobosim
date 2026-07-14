"""
Web frontend for PyRoboSim.

This subpackage renders a PyRoboSim world in the browser using Plotly/Dash,
as a lightweight alternative to the PySide6/matplotlib GUI. It has no
dependency on PySide6 or ROS, and does not modify the core world model: it
only reads the geometry already stored on world entities.
"""
