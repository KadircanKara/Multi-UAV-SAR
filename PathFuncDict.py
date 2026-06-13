import math

from Distance import *
from Connectivity import *
from Time import *

from PathSolution import *
from PathOptimizationModel import obj_name_sol_attr_dict

model_metric_info = {
    "Objectives": {
        'Mission Time': (get_mission_time, 1, "seconds"),
        'Percentage Connectivity': (get_percentage_connectivity, -1, "%"),
        'Max Disconnected Time': (get_max_disconnected_time, 1, "timesteps"),
        'Mean Disconnected Time': (get_mean_disconnected_time, 1, "timesteps"),
        'Max Mean TBV': (get_max_mean_tbv, 1, "seconds"),
    },
    "Constraints": {
        'Max Mission Time': max_mission_time,
        'Min Percentage Connectivity': min_perc_conn_constraint,
        'Max Mean TBV Ceiling': max_mean_tbv_constraint,
        'Path Speed Violations as Constraint': path_speed_violations_as_constraint
    }
}


def compute_all_objectives(sol):
    """Ensure every objective metric is populated on a PathSolution, computing
    only the ones that are not already cached.

    Why this is needed: the getters in ``model_metric_info`` only READ cached
    attributes (e.g. ``get_max_mean_tbv`` is just ``return sol.max_mean_tbv``);
    the actual maths lives in ``PathSolution`` methods gated behind the
    ``calculate_*`` constructor flags. A solution produced for one model can
    therefore be missing the objectives that model did not optimise — most
    sharply ``max_disconnected_time`` / ``mean_disconnected_time``, which are not
    even initialised in ``PathSolution.__init__`` (accessing them raises
    ``AttributeError`` until computed). This makes a solution fully comparable
    across models by filling the gaps in dependency order.

    Efficiency: each block fires only when its metric is absent, so calling this
    on an already-complete solution (e.g. the back-filled ones in Results/) does
    no work. Mutates ``sol`` in place and returns it.
    """
    info = sol.info

    # 1. Path plan — prerequisite for every other metric (sets mission_time,
    #    real_time_path_matrix, drone_dict, time_slots, time_elapsed_at_steps).
    if (getattr(sol, "real_time_path_matrix", None) is None
            or sol.mission_time is None
            or not hasattr(sol, "drone_dict")):
        sol.get_drone_dict()
        sol.get_pathplan()

    # 2. Connectivity (percentage_connectivity + the connectivity_matrix).
    if sol.percentage_connectivity is None:
        sol.do_connectivity_calculations()

    # 3. Disconnectivity (max/mean_disconnected_time) — depends on the
    #    connectivity_matrix, so make sure that exists first.
    if (getattr(sol, "max_disconnected_time", None) is None
            or getattr(sol, "mean_disconnected_time", None) is None):
        if getattr(sol, "connectivity_matrix", None) is None:
            sol.do_connectivity_calculations()
        sol.do_disconnectivity_calculations()

    # 4. Time-between-visits (max_mean_tbv) — only defined for n_visits > 1;
    #    left at its default (0) otherwise, matching the rest of the codebase.
    if info.n_visits > 1 and getattr(sol, "mean_tbv", None) is None:
        sol.get_visit_times()
        sol.get_tbv()
        sol.get_mean_tbv()

    return sol


def objective_values(sol):
    """Return ``{objective_name: value}`` for ALL five objectives of a solution.

    Ensures every objective is populated first via :func:`compute_all_objectives`
    (which only computes what is not already cached), then reads each value off
    the solution via ``obj_name_sol_attr_dict``. This is model-agnostic: every
    objective is returned regardless of which ones the solution's model
    optimised, so solutions from different models become directly comparable.

    Values are JSON-safe floats; ``None`` where unavailable / non-finite.
    ``Max Mean TBV`` is 0 when ``n_visits == 1`` (undefined there — callers that
    care about that case should special-case it).
    """
    compute_all_objectives(sol)
    values = {}
    for name, attr in obj_name_sol_attr_dict.items():
        v = getattr(sol, attr, None)
        if v is None:
            values[name] = None
            continue
        try:
            fv = float(v)
        except (TypeError, ValueError):
            values[name] = None
            continue
        values[name] = fv if math.isfinite(fv) else None
    return values