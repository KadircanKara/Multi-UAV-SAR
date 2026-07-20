"""Constraint-capable MOEA/D.

pymoo's stock ``MOEAD`` hard-asserts ``not problem.has_constraints()``
(pymoo/algorithms/moo/moead.py:79). This subclass enables constraints by
applying Deb's parameter-less feasibility-first rule to the DECOMPOSED values
during neighborhood replacement — pymoo's own commented-out sketch at
moead.py:125-129, promoted to working code:

  * feasible vs feasible     -> lower decomposition value wins (vanilla MOEA/D)
  * feasible vs infeasible   -> the feasible one wins
  * infeasible vs infeasible -> lower total constraint violation (CV) wins

Objectives are normalised into the population's own [0,1] box before the
decomposition sees them, with the offspring measured against that same box;
without it, this project's raw scales (seconds beside fractions) let the large
objective dominate every subproblem.

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

        # Scalarise on NORMALISED objectives. PathProblem reports raw values --
        # Mission Time in seconds (~1e3) beside Percentage Connectivity as a
        # fraction (~1) -- and Tchebicheff/PBI apply the weights straight to those
        # magnitudes (pymoo's Decomposition stores a nadir_point but neither
        # decomposition uses it, so there is no hook to do this for us). Left raw,
        # the large objective dominates every subproblem, the weight vectors stop
        # discriminating, and MOEA/D's diversity mechanism goes inert: at raw
        # scales, pop=100/gen=150 over 3 seeds found ZERO feasible solutions.
        # Mapping F into the population's own [0,1] box, with the offspring
        # measured against that same box, first restores it. NSGA3 normalises via
        # its ideal/extreme points and NSGA2's rank+crowding is scale-invariant,
        # so this puts MOEA/D in line with the other engines rather than apart.
        # F is shifted so the ideal sits at the origin, hence ideal_point=zero.
        allF = pop.get("F")
        # `lo` folds in the offspring: Tchebicheff takes |F - utopian|, so a value
        # BELOW the shift origin folds back into a penalty. pymoo's MOEAD._next
        # already updates self.ideal with the offspring before calling us, but
        # taking the min here too keeps _replace correct on its own terms.
        lo = np.minimum(np.minimum(allF.min(axis=0), self.ideal), off.F)
        # `hi` deliberately EXCLUDES the offspring: the same affine map is applied
        # to incumbents and offspring, so an offspring worse than every incumbent
        # maps above 1 and simply scores worse. Letting it widen the range instead
        # would compress the incumbents into a sliver and randomise the comparison.
        span = allF.max(axis=0) - lo
        # A column with no spread carries no information; dividing by a tiny floor
        # would turn any deviation in it into the whole scalarisation. Leave it
        # unscaled, matching pymoo's own handling of zero-range objectives.
        span[span <= 0] = 1.0
        zero = np.zeros(allF.shape[1])

        FV = self.decomposition.do((pop[N].get("F") - lo) / span,
                                   weights=self.ref_dirs[N, :], ideal_point=zero)
        off_FV = self.decomposition.do((off.F[None, :] - lo) / span,
                                       weights=self.ref_dirs[N, :], ideal_point=zero)

        if self.problem.has_constraints():
            # parameter_less maps infeasible entries to fmax + CV, so any feasible
            # value beats any infeasible one and infeasible ties order by CV.
            CV, off_CV = pop[N].get("CV")[:, 0], np.full(len(off_FV), off.CV)
            fmax = max(FV.max(), off_FV.max())
            FV = parameter_less(FV, CV, fmax=fmax)
            off_FV = parameter_less(off_FV, off_CV, fmax=fmax)

        # MOEA/D-DE-style cap: one offspring may claim at most MAX_REPLACEMENTS of
        # its neighbours. Unlike Li & Zhang's `nr`, which permutes the neighbourhood
        # before selecting, this takes the closest winners deterministically.
        #
        # Uncapped, a single offspring takes the whole neighbourhood -- and while
        # the population is infeasible, parameter_less makes every comparison
        # weight-INDEPENDENT (pure CV), so that happens constantly: at raw scales
        # it held only 11-13 of 100 population members distinct. With normalised
        # objectives (above) it holds 53-60 of 100.
        I = np.where(off_FV < FV)[0][:MAX_REPLACEMENTS]
        pop[N[I]] = off
