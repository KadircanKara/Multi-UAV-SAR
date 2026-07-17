"""
Compatibility shim for reading legacy pandas pickles.

The seeded ``Results/`` pickles (``Objectives/*-ObjectiveValues.pkl``,
``Solutions/*-SolutionObjects.pkl``) were written by a pandas whose
``StringArray.__setstate__`` serialized its state as a 2-tuple
``(dtype, ndarray)``.  pandas 2.3.x expects a 3-tuple
``(dtype, ndarray, attrs_dict)`` and its ``NDArrayBacked.__setstate__`` raises
``NotImplementedError`` on the 2-tuple.  A single string-typed column (the
objective names live in the DataFrame's column Index) is enough to make the
whole ``read_pickle`` fail, and ``library_service`` swallows that exception —
so the entire precomputed library silently shows up empty.

``install_legacy_string_pickle_compat`` wraps ``StringArray.__setstate__`` with
an idempotent function that pads the legacy 2-tuple to the current 3-tuple and
delegates to the native implementation.  The current 3-tuple path is passed
through untouched, so freshly-written pickles are unaffected.
"""

_INSTALLED = False


def install_legacy_string_pickle_compat() -> None:
    """Patch pandas so legacy 2-tuple StringArray pickles load. Idempotent."""
    global _INSTALLED
    if _INSTALLED:
        return
    try:
        from pandas.core.arrays.string_ import StringArray
    except Exception:
        # pandas internals moved; nothing to patch (fail open, not loud).
        _INSTALLED = True
        return

    native_setstate = StringArray.__setstate__

    def _compat_setstate(self, state):
        # Legacy pickle: (dtype, ndarray). Current pandas: (dtype, ndarray, attrs).
        if isinstance(state, tuple) and len(state) == 2:
            state = (state[0], state[1], {})
        return native_setstate(self, state)

    StringArray.__setstate__ = _compat_setstate
    _INSTALLED = True
