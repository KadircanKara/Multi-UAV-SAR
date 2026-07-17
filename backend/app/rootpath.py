"""
Inserts the repo root into sys.path so that flat root modules
(PathInfo, PathOptimizationModel, etc.) can be imported by name.

Import this module FIRST in any file that needs root modules:
    import app.rootpath   # or: from app import rootpath
The side-effect is idempotent: the path is only inserted once.
"""
import os
import sys

REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))

if REPO_ROOT not in sys.path:
    sys.path.insert(0, REPO_ROOT)

# Bootstrap side-effect: make legacy 2-tuple StringArray pickles (the seeded
# Results/ files) loadable under the installed pandas. Every module that reads
# those pickles imports this module first, so the patch is always in place
# before the first read_pickle. Idempotent.
from app.pandas_compat import install_legacy_string_pickle_compat  # noqa: E402

install_legacy_string_pickle_compat()
