"""
Basic checks for external dependencies.
"""

import pytest

def test_import_pddlstream():
    import pddlstream
    assert("__init__.py" in pddlstream.__file__)
