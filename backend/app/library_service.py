"""
Library service: scans Results/ for precomputed scenarios and exposes
structured metadata without triggering any heavy algorithm imports.

Import-safety: only PathInfo, PathOptimizationModel (for AVAILABLE_MODELS),
and pandas are used here.  PathAlgorithm / PathUnitTest / main are never
imported.
"""
import math
import os
import re
import threading
from typing import Optional

import pandas as pd

import app.rootpath  # side-effect: inserts repo root into sys.path
from app import settings, models_registry
from app.model_aliases import to_display, to_storage
from PathOptimizationModel import AVAILABLE_MODELS


# ---------------------------------------------------------------------------
# Security: path-traversal containment
# ---------------------------------------------------------------------------

# Accept only known-safe chars: alphanumerics, underscore, dot, hyphen,
# parentheses (needed for "sqrt(8)").  Slashes, backslashes, and NUL bytes
# are caught by the explicit checks above before this regex runs.
_SAFE_SCENARIO_RE = re.compile(r"^[A-Za-z0-9_.()-]+$")


def _is_safe_scenario_name(scenario: str) -> bool:
    """Return True iff *scenario* is a safe, non-traversal scenario name."""
    if not scenario:
        return False
    if ".." in scenario:
        return False
    if "/" in scenario or "\\" in scenario:
        return False
    if os.sep in scenario:
        return False
    if "\x00" in scenario:
        return False
    return bool(_SAFE_SCENARIO_RE.match(scenario))


def _assert_path_in_results_root(path: str) -> bool:
    """Return True iff the resolved *path* is inside RESULTS_ROOT."""
    root = os.path.abspath(settings.RESULTS_ROOT) + os.sep
    return os.path.abspath(path).startswith(root)


# ---------------------------------------------------------------------------
# Model-key resolution
# ---------------------------------------------------------------------------

def resolve_model_key(scenario: str) -> str:
    """
    Map a scenario name to a key in AVAILABLE_MODELS.

    Examples
    --------
    MOO_NSGA2_TC_g_8_...   → TC_MOO_NSGA2
    WS_GA_TCDT_g_8_...     → TCDT_WS
    SOO_GA_MTSP_g_8_...    → MTSP
    SOO_NSGA2_MTSP_g_8_... → MTSP
    """
    prefix = scenario.split("_g_")[0]          # e.g. "MOO_NSGA2_TC", "WS_GA_TCDT"
    parts = prefix.split("_")
    typ, alg, exp = parts[0], parts[1], "_".join(parts[2:])

    if exp in ("MTSP", "CONN"):
        return exp                              # single registry entry; algorithm ignored

    if typ == "WS":
        return f"{exp}_WS"

    if typ == "MOO":
        return f"{exp}_MOO_{alg}"

    return exp


# ---------------------------------------------------------------------------
# Parameter parsing
# ---------------------------------------------------------------------------

_TAIL_RE = re.compile(
    r"_g_(?P<grid>\d+)"
    r"_a_(?P<cell>[^_]+)"
    r"_n_(?P<ndrones>\d+)"
    r"_v_(?P<speed>[^_]+)"
    r"_r_(?P<range>[^_]+)"
    r"_(?P<variant>nvisits|ntours)_(?P<vval>\d+)"
)


def parse_scenario_params(scenario: str) -> dict:
    """
    Best-effort parse of the ``_g_..._a_..._n_..._v_..._r_..._(nvisits|ntours)_...``
    tail into a structured dict.

    Never raises: missing fields are simply omitted from the returned dict.
    comm_range is kept as a raw string (may be "sqrt(8)").
    """
    params: dict = {}
    try:
        m = _TAIL_RE.search(scenario)
        if m is None:
            return params
        g = m.groupdict()

        try:
            params["grid_size"] = int(g["grid"])
        except (ValueError, KeyError):
            pass

        try:
            cell = g["cell"]
            params["cell_side_length"] = int(cell) if "." not in cell else float(cell)
        except (ValueError, KeyError):
            pass

        try:
            params["number_of_drones"] = int(g["ndrones"])
        except (ValueError, KeyError):
            pass

        try:
            params["max_drone_speed"] = float(g["speed"])
        except (ValueError, KeyError):
            pass

        try:
            params["comm_range"] = g["range"]
        except KeyError:
            pass

        try:
            params["variant"] = g["variant"]
            params["variant_value"] = int(g["vval"])
        except (ValueError, KeyError):
            pass

    except Exception:
        # Absolute fallback — never propagate parsing errors
        pass

    return params


# ---------------------------------------------------------------------------
# Filesystem scan helpers
# ---------------------------------------------------------------------------

def _objectives_dir() -> str:
    return os.path.join(settings.RESULTS_ROOT, "Objectives")


def _solutions_dir() -> str:
    return os.path.join(settings.RESULTS_ROOT, "Solutions")


def _obj_path(scenario: str) -> str:
    return os.path.join(_objectives_dir(), f"{scenario}-ObjectiveValues.pkl")


def _sol_path(scenario: str) -> str:
    return os.path.join(_solutions_dir(), f"{scenario}-SolutionObjects.pkl")


# ---------------------------------------------------------------------------
# Memoization
# ---------------------------------------------------------------------------
# list_scenarios() and model_grid() each re-unpickle every Objectives/*.pkl on
# the box per call (hundreds of pd.read_pickle) — unthrottled, that is a cheap
# way to peg a core. The set of those files only changes when a run is saved (a
# new *-ObjectiveValues.pkl appears) or purged, and any such change bumps the
# Objectives/ directory's mtime. So we key a cache on that mtime and rebuild only
# when it moves: a save_run writing a new file busts it automatically, and
# nothing else has to remember to invalidate.

_cache_lock = threading.Lock()
# Single-entry caches: a changed signature drops the old value wholesale, so the
# cache can never grow unbounded.
_list_cache: dict = {"key": None, "value": None}
_grid_cache: dict = {"key": None, "value": {}}  # value: {storage_model_key: grid}


def _objectives_sig() -> tuple:
    """A cheap signature that changes whenever the Objectives/ file set changes.

    Uses the directory's nanosecond mtime — any create/delete of a pickle bumps
    the directory entry. Includes the directory PATH so a test that repoints
    RESULTS_ROOT never serves another root's cache. A missing/unreadable dir maps
    to a stable sentinel."""
    d = _objectives_dir()
    try:
        return (d, os.stat(d).st_mtime_ns)
    except OSError:
        return (d, None)


def _bust_scenario_memos() -> None:
    """Drop the list/grid memoized caches wholesale.

    The _objectives_sig signature only moves when a pickle is CREATED or DELETED
    in Objectives/ (that bumps the directory mtime). An in-place OVERWRITE of an
    existing pickle — possible only under ALLOW_LIBRARY_SAVE=1 via save_run —
    keeps the same filename, so the directory mtime may not change and the memo
    would keep serving the old stats. save_run calls this explicitly after
    copying the new pickles in, mirroring its _load_selector.cache_clear()."""
    with _cache_lock:
        _list_cache["key"] = None
        _list_cache["value"] = None
        _grid_cache["key"] = None
        _grid_cache["value"] = {}


# ---------------------------------------------------------------------------
# Private shared helper
# ---------------------------------------------------------------------------

def _read_scenario_meta(scenario: str) -> Optional[dict]:
    """
    Shared core logic used by both list_scenarios() and get_scenario().

    Resolves model key, reads the (small) objectives pickle, parses params.
    Returns a dict with keys:
        model_key, model_dict, objectives, n_solutions, result_kind, params
    Returns None if anything is missing/invalid.

    Caller must have already validated the scenario name with
    _is_safe_scenario_name() before calling this helper.
    """
    # Require matching solutions pickle
    if not os.path.isfile(_sol_path(scenario)):
        return None

    try:
        model_key = resolve_model_key(scenario)
    except Exception:
        return None
    if not models_registry.known(model_key):
        return None

    model_dict = models_registry.get_model(model_key)

    obj_path = _obj_path(scenario)
    if not _assert_path_in_results_root(obj_path):
        return None

    try:
        df: pd.DataFrame = pd.read_pickle(obj_path)
        n_solutions: int = int(df.shape[0])
        objectives: list[str] = list(df.columns)
    except Exception:
        return None

    result_kind = (
        "front"
        if model_dict["Type"] == "MOO" and n_solutions > 1
        else "single"
    )

    params = parse_scenario_params(scenario)

    return {
        "model_key": model_key,
        "model_dict": model_dict,
        "objectives": objectives,
        "n_solutions": n_solutions,
        "result_kind": result_kind,
        "params": params,
    }


# ---------------------------------------------------------------------------
# Public service functions
# ---------------------------------------------------------------------------

def list_scenarios() -> list[dict]:
    """
    Scan Results/Objectives/*-ObjectiveValues.pkl (excluding *Abs* files).

    For each file:
      - derive the scenario name
      - require a matching Solutions pickle (else skip)
      - resolve model_key → skip if not in AVAILABLE_MODELS
      - read the objectives DataFrame to get n_solutions and column names
      - determine result_kind ("front" or "single")
      - merge parsed params

    Returns a list of dicts matching ScenarioSummary.

    Memoized on the Objectives/ directory mtime: the heavy per-file unpickle only
    reruns when the file set changes (a save or a purge), so back-to-back library
    listings are served from cache.
    """
    sig = _objectives_sig()
    with _cache_lock:
        if _list_cache["key"] == sig and _list_cache["value"] is not None:
            # Return a shallow copy so a caller mutating the list (append/sort)
            # cannot corrupt the shared cached value for the next request.
            return list(_list_cache["value"])
    rows = _scan_scenarios()
    with _cache_lock:
        _list_cache["key"] = sig
        _list_cache["value"] = rows
    return list(rows)


def _scan_scenarios() -> list[dict]:
    """Uncached core of list_scenarios (see it for the contract)."""
    obj_dir = _objectives_dir()
    if not os.path.isdir(obj_dir):
        return []

    rows: list[dict] = []

    for fname in sorted(os.listdir(obj_dir)):
        # The endswith filter already excludes *-ObjectiveValuesAbs.pkl because
        # those end with "Abs.pkl", not "-ObjectiveValues.pkl".
        if not fname.endswith("-ObjectiveValues.pkl"):
            continue

        scenario = fname[: -len("-ObjectiveValues.pkl")]

        if not _is_safe_scenario_name(scenario):
            continue

        meta = _read_scenario_meta(scenario)
        if meta is None:
            continue

        params = meta["params"]

        # Build row — flat fields for ScenarioSummary. Both identifiers are
        # displayified (TCDT→TCDV) so the UI shows the TBV "V" code; inbound
        # requests normalize back to storage before touching disk.
        row: dict = {
            "scenario": to_display(scenario),
            "model_key": to_display(meta["model_key"]),
            "type": meta["model_dict"]["Type"],
            "algorithm": meta["model_dict"]["Alg"],
            "objectives": meta["objectives"],
            "n_solutions": meta["n_solutions"],
            "result_kind": meta["result_kind"],
            "grid_size": params.get("grid_size"),
            "cell_side_length": params.get("cell_side_length"),
            "max_drone_speed": params.get("max_drone_speed"),
            "number_of_drones": params.get("number_of_drones"),
            "comm_range": params.get("comm_range"),
            "variant": params.get("variant"),
            "variant_value": params.get("variant_value"),
            "has_solutions": True,
        }
        rows.append(row)

    # Sort by (type, number of objectives, number_of_drones)
    rows.sort(key=lambda r: (
        r["type"] or "",
        len(r["objectives"]),
        r["number_of_drones"] or 0,
    ))

    return rows


def _safe_float(v) -> Optional[float]:
    """Convert a numeric value to a JSON-safe float, returning None for inf/nan."""
    try:
        f = float(v)
        if math.isfinite(f):
            return f
        return None
    except (TypeError, ValueError):
        return None


def _parse_comm_range_value(raw: str) -> float:
    """
    Parse a raw comm_range string to a float.

    "2"       → 2.0
    "4"       → 4.0
    "sqrt(8)" → 2.8284271247461903
    """
    m = re.match(r"^sqrt\((\d+(?:\.\d+)?)\)$", raw)
    if m:
        return math.sqrt(float(m.group(1)))
    return float(raw)


def model_grid(model_key: str) -> Optional[dict]:
    """
    Return the parameter grid for a single model, with per-scenario objective
    summary statistics.

    Only reads Objectives/*-ObjectiveValues.pkl (never Solutions).  Each
    scenario row includes min/max/mean/best for every objective (ABS values).
    'best' = min for minimize objectives (+1 polarity) or max for maximize
    objectives (-1 polarity).

    Returns None if the model_key is unknown or no seeded scenarios exist.

    Memoized on the Objectives/ directory mtime, per resolved model key: the
    per-file unpickle only reruns when the file set changes (a save or a purge).
    """
    # A display key (TCDV) may arrive from the route; normalize to storage so the
    # cache key and the `resolved != model_key` filter both match on-disk names.
    model_key = to_storage(model_key)

    sig = _objectives_sig()
    with _cache_lock:
        if _grid_cache["key"] != sig:
            _grid_cache["key"] = sig
            _grid_cache["value"] = {}
        elif model_key in _grid_cache["value"]:
            return _grid_cache["value"][model_key]

    result = _compute_model_grid(model_key)

    with _cache_lock:
        # Store only under the signature we computed against — if the mtime moved
        # while we scanned, a newer reader already reset the bucket, so drop ours.
        if _grid_cache["key"] == sig:
            _grid_cache["value"][model_key] = result
    return result


def _compute_model_grid(model_key: str) -> Optional[dict]:
    """Uncached core of model_grid (see it for the contract). Expects a STORAGE
    model key (the public wrapper normalizes display → storage)."""
    # Lazy import to avoid circular; selector_service imports library_service,
    # so we import at call-time not at module load.
    from app.selector_service import get_polarities  # noqa: PLC0415

    if not models_registry.known(model_key):
        return None

    model_dict = models_registry.get_model(model_key)
    polarities = get_polarities(model_dict)

    obj_dir = _objectives_dir()
    if not os.path.isdir(obj_dir):
        return None

    rows: list[dict] = []

    for fname in sorted(os.listdir(obj_dir)):
        if not fname.endswith("-ObjectiveValues.pkl"):
            continue

        scenario = fname[: -len("-ObjectiveValues.pkl")]

        if not _is_safe_scenario_name(scenario):
            continue

        # Only keep scenarios that belong to this model
        try:
            resolved = resolve_model_key(scenario)
        except Exception:
            continue
        if resolved != model_key:
            continue

        # Only expose scenarios that also have a solutions pickle
        # (so the Explore deep-dive will work)
        if not os.path.isfile(_sol_path(scenario)):
            continue

        obj_path = _obj_path(scenario)
        if not _assert_path_in_results_root(obj_path):
            continue

        try:
            objdf: pd.DataFrame = pd.read_pickle(obj_path)
        except Exception:
            continue

        absdf = objdf.abs()
        n_solutions = int(absdf.shape[0])

        objective_stats: dict[str, dict] = {}
        for obj in model_dict["F"]:
            if obj not in absdf.columns:
                continue
            col = absdf[obj]
            pol = polarities.get(obj, 1)
            mn = _safe_float(col.min())
            mx = _safe_float(col.max())
            me = _safe_float(col.mean())
            best = mx if pol == -1 else mn
            objective_stats[obj] = {
                "min": mn,
                "max": mx,
                "mean": me,
                "best": best,
            }

        params = parse_scenario_params(scenario)
        number_of_drones: Optional[int] = params.get("number_of_drones")
        comm_range_raw: Optional[str] = params.get("comm_range")
        variant: Optional[str] = params.get("variant")
        variant_value: Optional[int] = params.get("variant_value")

        comm_range_value: Optional[float] = None
        if comm_range_raw is not None:
            try:
                comm_range_value = _parse_comm_range_value(comm_range_raw)
            except (ValueError, TypeError):
                comm_range_value = None

        n_visits: Optional[int] = None
        n_tours: Optional[int] = None
        if variant == "nvisits":
            n_visits = variant_value
        elif variant == "ntours":
            n_tours = variant_value

        result_kind = (
            "front"
            if model_dict["Type"] == "MOO" and n_solutions > 1
            else "single"
        )

        row: dict = {
            "scenario": to_display(scenario),
            "number_of_drones": number_of_drones,
            "comm_range": comm_range_raw,
            "comm_range_value": comm_range_value,
            "n_visits": n_visits,
            "n_solutions": n_solutions,
            "result_kind": result_kind,
            "objective_stats": objective_stats,
        }
        if n_tours is not None:
            row["n_tours"] = n_tours

        rows.append(row)

    if not rows:
        return None

    # Sort by (number_of_drones, comm_range_value, n_visits)
    def _sort_key(r: dict):
        return (
            r.get("number_of_drones") or 0,
            r.get("comm_range_value") or 0.0,
            r.get("n_visits") or 0,
        )

    rows.sort(key=_sort_key)

    return {
        "model_key": to_display(model_key),
        "type": model_dict["Type"],
        "algorithm": model_dict["Alg"],
        "objectives": list(model_dict["F"]),
        "polarities": polarities,
        "scenarios": rows,
    }


def get_scenario(scenario: str) -> Optional[dict]:
    """
    Return a detailed dict for a single scenario, or None if pickles are missing,
    the scenario name is unsafe, or the scenario cannot be resolved.
    """
    # A display scenario (…TCDV…) may arrive from the client; normalize to the
    # storage form so filesystem lookups hit the real …TCDT… pickles.
    scenario = to_storage(scenario)

    # Security: reject path-traversal attempts before any filesystem access
    if not _is_safe_scenario_name(scenario):
        return None

    sol_path = _sol_path(scenario)
    if not _assert_path_in_results_root(sol_path):
        return None

    meta = _read_scenario_meta(scenario)
    if meta is None:
        return None

    model_dict = meta["model_dict"]

    # Build ModelInfo-compatible sub-dict
    model_info = {
        "name": to_display(meta["model_key"]),
        "type": model_dict["Type"],
        "algorithm": model_dict["Alg"],
        "objectives": meta["objectives"],
        "constraints": model_dict.get("G", []),
    }

    return {
        "scenario": to_display(scenario),
        "model": model_info,
        "n_solutions": meta["n_solutions"],
        "result_kind": meta["result_kind"],
        "params": meta["params"],
    }
