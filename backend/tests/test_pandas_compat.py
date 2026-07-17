"""Guard for the legacy-pickle compat shim (app.pandas_compat).

The seeded Results/ pickles were written by a pandas whose StringArray state is
a 2-tuple (dtype, ndarray); pandas 2.3.x expects a 3-tuple and raises
NotImplementedError otherwise, which silently emptied the whole library. These
tests forge a legacy 2-tuple StringArray pickle and assert it round-trips once
the shim is installed.
"""
import pickle

import numpy as np
import pandas as pd
import pytest

from app.pandas_compat import install_legacy_string_pickle_compat


def _legacy_pickle_bytes(values):
    """Pickle a StringArray forcing the legacy 2-tuple __setstate__ payload."""
    from pandas.core.arrays.string_ import StringArray

    arr = pd.array(values, dtype=pd.StringDtype(storage="python", na_value=np.nan))
    original_reduce = StringArray.__reduce__

    def legacy_reduce(self):
        func, args, state = original_reduce(self)
        # current pandas emits a 3-tuple state; drop the trailing attrs dict to
        # reproduce the older 2-tuple that the seeded pickles carry.
        if isinstance(state, tuple) and len(state) == 3:
            state = (state[0], state[1])
        return func, args, state

    StringArray.__reduce__ = legacy_reduce
    try:
        return pickle.dumps(arr)
    finally:
        StringArray.__reduce__ = original_reduce


def test_legacy_two_tuple_pickle_loads_after_install():
    install_legacy_string_pickle_compat()
    blob = _legacy_pickle_bytes(["Mission Time", "Max Mean TBV", None])
    restored = pickle.loads(blob)
    assert list(restored[:2]) == ["Mission Time", "Max Mean TBV"]
    assert pd.isna(restored[2])


def test_install_is_idempotent():
    # Installing twice must not double-wrap or change behavior.
    install_legacy_string_pickle_compat()
    install_legacy_string_pickle_compat()
    blob = _legacy_pickle_bytes(["a", "b"])
    assert list(pickle.loads(blob)) == ["a", "b"]


def test_current_three_tuple_pickle_still_loads():
    # A pickle written by the installed pandas (native 3-tuple state) is
    # delegated through untouched.
    install_legacy_string_pickle_compat()
    arr = pd.array(["x", "y"], dtype=pd.StringDtype(storage="python", na_value=np.nan))
    assert list(pickle.loads(pickle.dumps(arr))) == ["x", "y"]
