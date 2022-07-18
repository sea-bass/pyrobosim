"""
Basic checks for external dependencies.
"""

import sys
import pytest


xfail = pytest.mark.xfail('pandas' not in sys.modules)
docutils = pytest.importorskip("pddlstream")
def test_import_pddlstream():
    import pddlstream
    assert("__init__.py" in pddlstream.__file__)
