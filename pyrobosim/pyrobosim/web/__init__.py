"""
Web UI for PyRoboSim.

This subpackage renders a PyRoboSim world in the browser using Plotly Dash.
It does not modify the core world model: it only reads the geometry already
stored on world entities, and drives the world through its regular APIs.
"""

from .app import create_app, start_ui
