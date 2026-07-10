# Configuration file for the Sphinx documentation builder.
#
# This file only contains a selection of the most common options. For a full
# list see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html

# -- Path setup --------------------------------------------------------------

# If extensions (or modules to document with autodoc) are in another directory,
# add these directories to sys.path here. If the directory is relative to the
# documentation root, use os.path.abspath to make it absolute, like shown here.
#
# import os
# import sys
# sys.path.insert(0, os.path.abspath('.'))


# -- Project information -----------------------------------------------------

project = "pyrobosim"
copyright = "2022-2026, Sebastian Castro"
author = "Sebastian Castro"

# The full version, including alpha/beta/rc tags
version = release = "4.3.4"


# -- General configuration ---------------------------------------------------

# Add any Sphinx extension module names here, as strings. They can be
# extensions coming with Sphinx (named 'sphinx.ext.*') or your custom
# ones.
extensions = [
    "sphinx.ext.autodoc",
    "sphinx.ext.autosummary",
    "sphinx_rtd_theme",
    "sphinx_copybutton",
    "sphinxcontrib.youtube",
]

# Add any paths that contain templates here, relative to this directory.
templates_path = ["_templates"]

# List of patterns, relative to source directory, that match files and
# directories to ignore when looking for source files.
# This pattern also affects html_static_path and html_extra_path.
exclude_patterns: list[str] = []

# Mock imports for external dependencies.
autodoc_mock_imports = [
    "action_msgs",
    "geometry_msgs",
    "pddlstream",
    "pyrobosim_msgs",
    "rclpy",
    "std_srvs",
]

# Importing PySide6 installs a global import hook (shibokensupport) that calls
# inspect.getsource() on every subsequently imported module. Sphinx's mocked
# modules (see autodoc_mock_imports above) have a self-referential __wrapped__
# attribute, so inspect.unwrap() raises "ValueError: wrapper loop" and the
# import fails, leaving those API pages empty. Make inspect.unwrap() return the
# object unchanged in that case; getsource() then fails with a regular error
# that the hook already handles.
import inspect

_original_unwrap = inspect.unwrap


def _safe_unwrap(func, *, stop=None):  # type: ignore[no-untyped-def]
    try:
        if stop is None:
            return _original_unwrap(func)
        return _original_unwrap(func, stop=stop)
    except ValueError:
        return func


inspect.unwrap = _safe_unwrap


# -- Options for HTML output -------------------------------------------------

# The theme to use for HTML and HTML Help pages.  See the documentation for
# a list of builtin themes.
#
html_theme = "sphinx_rtd_theme"
