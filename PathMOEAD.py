"""Constraint-capable MOEA/D.

pymoo's stock ``MOEAD`` hard-asserts ``not problem.has_constraints()``
(pymoo/algorithms/moo/moead.py:79). This subclass enables constraints by
applying Deb's parameter-less feasibility-first rule to the DECOMPOSED values
during neighborhood replacement — pymoo's own commented-out sketch at
moead.py:125-129, promoted to working code:

  * feasible vs feasible     -> lower decomposition value wins (vanilla MOEA/D)
  * feasible vs infeasible   -> the feasible one wins
  * infeasible vs infeasible -> lower total constraint violation (CV) wins

This matches the feasibility-first semantics NSGA2/NSGA3 use elsewhere in this
project, so fronts are comparable across engines.

Import-safety: this module imports only numpy/scipy/pymoo — it must NEVER
import ``main`` or ``PathAlgorithm`` (the optimizer worker imports it).

``_setup`` duplicates the parent's body minus its assert (pymoo inlines the
assert with no overridable hook). ``test_setup_override_matches_pymoo_minus_the_assert``
fails if a pymoo upgrade changes that body, so the copy cannot go stale silently.
"""
import numpy as np
from scipy.spatial.distance import cdist

from pymoo.algorithms.moo.moead import MOEAD, default_decomp
from pymoo.util.misc import parameter_less
from pymoo.util.reference_direction import default_ref_dirs

# Max neighbours a single offspring may replace in one step (MOEA/D-DE's `nr`).
MAX_REPLACEMENTS = 2


class ConstrainedMOEAD(MOEAD):

    def _setup(self, problem, **kwargs):
        # Parent's _setup minus `assert not problem.has_constraints()`.
        if self.ref_dirs is None:
            self.ref_dirs = default_ref_dirs(problem.n_obj)
        self.pop_size = len(self.ref_dirs)
        self.neighbors = np.argsort(cdist(self.ref_dirs, self.ref_dirs),
                                    axis=1, kind='quicksort')[:, :self.n_neighbors]
        if self.decomposition is None:
            self.decomposition = default_decomp(problem)

    def _replace(self, k, off):
        pop = self.pop
        N = self.neighbors[k]
        FV = self.decomposition.do(pop[N].get("F"), weights=self.ref_dirs[N, :],
                                   ideal_point=self.ideal)
        off_FV = self.decomposition.do(off.F[None, :], weights=self.ref_dirs[N, :],
                                       ideal_point=self.ideal)

        if self.problem.has_constraints():
            # parameter_less maps infeasible entries to fmax + CV, so any feasible
            # value beats any infeasible one and infeasible ties order by CV.
            CV, off_CV = pop[N].get("CV")[:, 0], np.full(len(off_FV), off.CV)
            fmax = max(FV.max(), off_FV.max())
            FV = parameter_less(FV, CV, fmax=fmax)
            off_FV = parameter_less(off_FV, off_CV, fmax=fmax)

        # MOEA/D-DE-style cap: one offspring may claim at most MAX_REPLACEMENTS of
        # its neighbours. Uncapped, a single offspring takes the whole
        # neighbourhood -- and while the population is infeasible, parameter_less
        # makes every comparison weight-INDEPENDENT (pure CV), so that happens
        # constantly: measured 52 distinct individuals collapsing to 8 by
        # generation 100. Capping holds ~30/52.
        I = np.where(off_FV < FV)[0][:MAX_REPLACEMENTS]
        pop[N[I]] = off
