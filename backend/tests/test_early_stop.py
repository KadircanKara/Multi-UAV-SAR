"""
Unit tests for the optimizer's convergence early-stop rule (_EarlyStop).

The rule runs in SIGNED objective space (pymoo minimizes actual×polarity), so
'better' is always 'smaller' and objective direction is already baked in:
Percentage Connectivity is maximized, so its signed optimum is negative and
improving means going more negative.
"""
from app.optimizer_worker import _EarlyStop


def test_first_update_sets_reference_and_does_not_stop():
    es = _EarlyStop(patience=3, threshold=0.10)
    assert es.update([10.0]) is False


def test_keeps_running_while_any_objective_improves_over_threshold():
    es = _EarlyStop(patience=2, threshold=0.10)
    assert es.update([10.0]) is False  # reference
    assert es.update([8.0]) is False   # 20% better → reset, no stall
    assert es.update([7.0]) is False   # 12.5% better → reset
    assert es.update([6.0]) is False   # 14% better → reset


def test_stops_after_patience_generations_of_plateau():
    es = _EarlyStop(patience=3, threshold=0.10)
    assert es.update([10.0]) is False   # reference
    assert es.update([9.95]) is False   # 0.5% → stall 1
    assert es.update([9.95]) is False   # stall 2
    assert es.update([9.95]) is True    # stall 3 == patience → STOP


def test_any_single_objective_improving_prevents_stop():
    es = _EarlyStop(patience=2, threshold=0.10)
    assert es.update([10.0, 10.0]) is False  # reference
    assert es.update([10.0, 8.0]) is False   # B improved 20% → reset
    assert es.update([10.0, 7.0]) is False   # B improved 12.5% → reset
    assert es.update([10.0, 6.95]) is False  # B ~0.7% → stall 1
    assert es.update([10.0, 6.95]) is True   # neither improved → stall 2 → STOP


def test_connectivity_direction_handled_via_signed_values():
    # Maximized objective: signed = actual × -1. Improving = more negative.
    es = _EarlyStop(patience=2, threshold=0.10)
    assert es.update([-0.50]) is False  # actual connectivity 0.50 (reference)
    assert es.update([-0.60]) is False  # 0.60: 20% better (signed) → reset
    assert es.update([-0.61]) is False  # ~1.7% → stall 1
    assert es.update([-0.61]) is True   # stall 2 → STOP


def test_slow_steady_sub_threshold_gains_accumulate_and_keep_running():
    # Each step is < 10% but cumulative crosses 10% before patience is hit.
    es = _EarlyStop(patience=3, threshold=0.10)
    assert es.update([10.0]) is False   # reference
    assert es.update([9.6]) is False    # 4% vs ref → stall 1
    assert es.update([9.3]) is False    # 7% vs ref → stall 2
    assert es.update([9.0]) is False    # 10% vs ref → reset, stall 0
    assert es.update([8.7]) is False    # 3.3% vs new ref → stall 1


def test_non_finite_values_do_not_crash_or_falsely_improve():
    es = _EarlyStop(patience=2, threshold=0.10)
    assert es.update([float("inf")]) is False  # reference
    assert es.update([float("inf")]) is False  # skipped → stall 1
    assert es.update([float("inf")]) is True   # stall 2 → STOP
