"""
Model-key / scenario display aliasing for the Max Mean TBV objective.

The legacy seeded models coded Max Mean TBV as the letter ``T`` (e.g. the
five-objective model ``TCDT`` = Time · Connectivity · Disconnected · **Tbv**),
which collides visually with Mission Time's ``T``. The canonical code for Max
Mean TBV is ``V`` — this is what the optimizer synthesizer already emits (a
custom Time+Conn+TBV run downloads as ``TCV``).

Rather than rename 216 seeded data files on disk (and their pickled internals),
this module aliases the ONE seeded TBV model that has data and is browsable —
``TCDT`` — to its display form ``TCDV`` at the API boundary:

  * OUTBOUND  (storage → display): every ``model_key`` / ``scenario`` the API
    returns is passed through :func:`to_display`, so the UI shows ``TCDV``.
  * INBOUND   (display → storage): every ``model_key`` / ``scenario`` the API
    receives is passed through :func:`to_storage`, so data lookups still resolve
    the real ``…TCDT…`` filenames.

Scope is deliberately ``TCDT`` only:
  * ``TT`` / ``TCT`` have NO seeded data (they never appear in the UI), and
  * their display forms ``TV`` / ``TCV`` would COLLIDE with the synthesizer's
    codes for the same objective sets — aliasing them would misroute a saved
    custom run. ``TCDT``'s five-objective synth code is ``TCDxDnV`` (distinct
    from ``TCDV``), so ``TCDT ↔ TCDV`` is collision-free.

The Exp code appears only as an underscore-delimited token in both key shapes
(``TCDT_MOO_NSGA2``, ``TCDT_WS``, ``MOO_NSGA2_TCDT_g_8_…``), so a token-boundary
substitution is unambiguous. Both functions are idempotent.
"""
from __future__ import annotations

import re
from typing import Optional

_STORAGE = "TCDT"
_DISPLAY = "TCDV"

_STORAGE_RE = re.compile(rf"(^|_){_STORAGE}(_|$)")
_DISPLAY_RE = re.compile(rf"(^|_){_DISPLAY}(_|$)")


def to_display(key: Optional[str]) -> Optional[str]:
    """Storage → display (``TCDT`` → ``TCDV``). Pass-through for anything else."""
    if not key:
        return key
    return _STORAGE_RE.sub(rf"\g<1>{_DISPLAY}\g<2>", key)


def to_storage(key: Optional[str]) -> Optional[str]:
    """Display → storage (``TCDV`` → ``TCDT``). Pass-through for anything else."""
    if not key:
        return key
    return _DISPLAY_RE.sub(rf"\g<1>{_STORAGE}\g<2>", key)
