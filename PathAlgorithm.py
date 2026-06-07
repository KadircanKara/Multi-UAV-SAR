from typing import Any
from PathSampling import *
from PathMutation import *
from PathCrossover import *
from PathRepair import PathRepair
from PathOutput import *
from pymoo.core.repair import NoRepair
from PathProblem import *

from PathFitness import ws_fitness_evaluation
from pymoo.operators.selection.tournament import compare, TournamentSelection
from pymoo.core.survival import Survival
from pymoo.algorithms.base.genetic import GeneticAlgorithm



from pymoo.operators.crossover.nox import NoCrossover
from PathCrossover import PathCrossover
# from pymoo.operators.crossover. import pmx
from pymoo.operators.mutation.nom import NoMutation
from PathMutation import PathMutation
from pymoo.core.duplicate import NoDuplicateElimination
from PathDuplicateElimination import PathDuplicateElimination

from pymoo.algorithms.moo.nsga2 import NSGA2
from pymoo.algorithms.moo.ctaea import CTAEA
from pymoo.util.ref_dirs import get_reference_directions
from pymoo.constraints.as_obj import ConstraintsAsObjective
from pymoo.algorithms.moo.moead import MOEAD
from pymoo.constraints.eps import AdaptiveEpsilonConstraintHandling
from pymoo.algorithms.moo.nsga3 import NSGA3
from pymoo.algorithms.moo.unsga3 import UNSGA3
# from pymoo.algorithms.moo.age import AGEMOEA
from pymoo.algorithms.moo.sms import SMSEMOA
from pymoo.algorithms.soo.nonconvex.ga import GA
from pymoo.algorithms.soo.nonconvex.pso import PSO

from pymoo.termination.default import DefaultSingleObjectiveTermination
from PathOptimizationModel import *
from PathInput import *

from main import scenario, pop_size, path_eliminate_duplicates, path_sampling, path_mutation, path_crossover, path_repair

# Create a Custom GA with novel weighted sum consideration
class FitnessSurvival(Survival):

    def __init__(self) -> None:
        super().__init__(filter_infeasible=False)

    def _do(self, problem, pop, n_survive=None, **kwargs):
        F, cv = pop.get("weighted_F", "cv")
        assert F.shape[1] == 1, "FitnessSurvival can only used for single objective single!"
        S = np.lexsort([F[:, 0], cv])
        pop.set("rank", np.argsort(S))
        return pop[S[:n_survive]]
    
def comp_by_cv_and_fitness(pop, P, **kwargs):
    S = np.full(P.shape[0], np.nan)

    for i in range(P.shape[0]):
        a, b = P[i, 0], P[i, 1]

        # if at least one solution is infeasible
        if pop[a].CV > 0.0 or pop[b].CV > 0.0:
            S[i] = compare(a, pop[a].CV, b, pop[b].CV, method='smaller_is_better', return_random_if_equal=True)

        # both solutions are feasible just set random
        else:
            S[i] = compare(a, pop[a].F, b, pop[b].F, method='smaller_is_better', return_random_if_equal=True)

    return S[:, None].astype(int)


class WeightedSumGA(GeneticAlgorithm):
    def __init__(self,
                 pop_size=pop_size,
                 sampling=PathSampling(),
                 selection=TournamentSelection(func_comp=comp_by_cv_and_fitness),
                 crossover=PathCrossover(),
                 mutation=PathMutation(),
                 survival=FitnessSurvival(),
                 eliminate_duplicates=True,
                 n_offsprings=None,
                 # output=PathOutput(),
                 **kwargs):
        super().__init__(pop_size=pop_size,
                         sampling=sampling,
                         selection=selection,
                         crossover=crossover,
                         mutation=mutation,
                         survival=survival,
                         eliminate_duplicates=eliminate_duplicates,
                         n_offsprings=n_offsprings,
                         # output=output,
                         **kwargs)

        self.termination = DefaultSingleObjectiveTermination()


algorithm_dict = {

    'PSO': PSO(
        pop_size=pop_size,
        sampling=path_sampling,
        # mutation=path_mutation,
        # crossover=path_crossover,
        # eliminate_duplicates=path_eliminate_duplicates,
        # repair=path_repair
    ),

    'GA': GA(
        pop_size=pop_size,
        sampling=path_sampling,
        mutation=path_mutation,
        crossover=path_crossover,
        eliminate_duplicates=path_eliminate_duplicates,
        repair=path_repair
    ),

    'Weighted Sum GA': WeightedSumGA(
        pop_size=pop_size,
        sampling=path_sampling,
        mutation=path_mutation,
        crossover=path_crossover,
        eliminate_duplicates=path_eliminate_duplicates,
        repair=path_repair
    ),


    'NSGA2' : NSGA2(
                        pop_size=pop_size,
                        sampling=path_sampling,
                        mutation=path_mutation,
                        crossover=path_crossover,
                        eliminate_duplicates=path_eliminate_duplicates,
                        repair=path_repair,
                ),

    'MOEAD' : MOEAD(
                    ref_dirs=get_reference_directions("das-dennis", len(model['F']), n_partitions=12),
                    # pop_size=len(get_reference_directions("das-dennis", len(model['F']), n_partitions=12)),
                    n_neighbors=15, # 5
                    prob_neighbor_mating=0.7, # 0.3
                    sampling=path_sampling,
                    mutation=path_mutation,
                    crossover=path_crossover,
                    # eliminate_duplicates=path_eliminate_duplicates,
                    repair=path_repair
                ),

    'NSGA3' : NSGA3(
                    ref_dirs=get_reference_directions("das-dennis", len(model['F']), n_partitions=12),
                    # pop_size=len(get_reference_directions("das-dennis", len(model['F']), n_partitions=12)),
                    pop_size=pop_size,
                    n_neighbors=15,
                    prob_neighbor_mating=0.7,
                    sampling=path_sampling,
                    mutation=path_mutation,
                    crossover=path_crossover,
                    eliminate_duplicates=path_eliminate_duplicates,
                    repair=path_repair,
                    # termination=path_termination
                        ),

    'USNGA3' : UNSGA3(
                    ref_dirs=np.array([[0.7, 0.9, 0.2, 0], [0.5, 0.7, 0.4, 0], [0.8, 0.9, 0.4, 0]]),
                    # pop_size=pop_size,
                    pop_size = len(np.array([[0.7, 0.9, 0.2, 0], [0.5, 0.7, 0.4, 0], [0.8, 0.9, 0.4, 0]])),
                    sampling=PathSampling(),
                    mutation=path_mutation,
                    crossover=path_crossover,
                    eliminate_duplicates=path_eliminate_duplicates,
                    # repair=path_repair
                        ),

    # 'AGEMOEAD' : AGEMOEA(
    #                 pop_size=126,
    #                 sampling=PathSampling(),
    #                 mutation=path_mutation,
    #                 crossover=path_crossover,
    #                 eliminate_duplicates=path_eliminate_duplicates,
    #                 repair=path_repair
    #                     ),

    'SMSEMOA' : SMSEMOA(
                    pop_size=pop_size,
                    sampling=PathSampling(),
                    mutation=path_mutation,
                    crossover=path_crossover,
                    eliminate_duplicates=path_eliminate_duplicates,
                    # repair=path_repair
                        )


}

class PathAlgorithm(object):

    def __init__(self, algorithm) -> None:

        self.algorithm = algorithm

    def __call__(self, *args: Any, **kwds: Any) -> Any:

        print("-->", self.algorithm)

        if self.algorithm == 'NSGA3':
            # print(len(algorithm_dict['NSGA3'].ref_dirs))
            return algorithm_dict['NSGA3']

        elif self.algorithm == 'MOEAD':
            return algorithm_dict['MOEAD']

        elif self.algorithm == 'NSGA2':
            return algorithm_dict['NSGA2']

        elif self.algorithm == 'GA':
            return algorithm_dict['GA']