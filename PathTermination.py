from pymoo.termination.default import MultiObjectiveSpaceTermination, DefaultMultiObjectiveTermination, MaximumGenerationTermination, MaximumFunctionCallTermination
from pymoo.termination.default import DefaultTermination, RobustTermination, DesignSpaceTermination, ConstraintViolationTermination

from PathInput import n_gen

from math import inf

from abc import abstractmethod

from pymoo.core.termination import Termination

class WeightedSumTermination(Termination):
    def __init__(self, max_gen=None):
        super().__init__()
        self.max_gen = max_gen

    def _update(self, algorithm):
        # Example: terminate based on max generations
        if self.max_gen is not None and algorithm.n_gen >= self.max_gen:
            return True
        return False





class PathTermination:

    def __init__(self) -> None:
        super().__init__()

        # the algorithm can be forced to terminate by setting this attribute to true
        self.force_termination = False

        # the value indicating how much perc has been made
        self.perc = 0.0

    def update(self, algorithm):
        """
        Provide the termination criterion a current status of the algorithm to update the perc.

        Parameters
        ----------
        algorithm : object
            The algorithm object which is used to determine whether a run has terminated.
        """

        if self.force_termination:
            progress = 1.0
        else:
            progress = self._update(algorithm)
            assert progress >= 0.0, "Invalid progress was set by the TerminationCriterion."

        self.perc = progress
        return self.perc

    def has_terminated(self):
        return self.perc >= 1.0

    def do_continue(self):
        return not self.has_terminated()

    def terminate(self):
        self.force_termination = True

    @abstractmethod
    def _update(self, algorithm):
        pass



class PathDefaultTermination(PathTermination):

    def __init__(self, x, cv, f, n_max_gen=n_gen, n_max_evals=inf) -> None:
        super().__init__()
        self.x = x
        self.cv = cv
        self.f = f

        self.max_gen = MaximumGenerationTermination(n_max_gen)
        self.max_evals = MaximumFunctionCallTermination(n_max_evals)

        self.criteria = [self.x, self.cv, self.f, self.max_gen, self.max_evals]

    def _update(self, algorithm):
        p = [criterion.update(algorithm) for criterion in self.criteria]
        return max(p)


class PathDefaultMultiObjectiveTermination(PathDefaultTermination):

    def __init__(self, xtol=0.0005, cvtol=1e-8, ftol=0.005, n_skip=5, period=50, **kwargs) -> None:
        x = RobustTermination(DesignSpaceTermination(tol=xtol, n_skip=n_skip), period)
        cv = RobustTermination(ConstraintViolationTermination(cvtol, terminate_when_feasible=False, n_skip=n_skip), period)
        f = RobustTermination(MultiObjectiveSpaceTermination(ftol, only_feas=True, n_skip=n_skip), period)
        super().__init__(x, cv, f, **kwargs)



path_termination = DefaultMultiObjectiveTermination()

'''termination = MultiObjectiveSpaceTermination(
    tol=0.0025,          # Tolerance for improvement in objective space
    # n_last=30,           # Look at the last 30 generations
    # nth_gen=5,           # Only check every 5 generations
    n_max_gen=1000,      # Maximum number of generations (acts as a safety net)
    n_max_evals=inf  # Maximum number of evaluations
)


class ParetoFrontTermination(Termination):
    def __init__(self, min_diff=0.001, n_last=20):
        super().__init__()
        self.min_diff = min_diff
        self.n_last = n_last
        self.hist = []

    def _do_continue(self, algorithm):
        # Get the size of the Pareto front
        front_size = len(algorithm.opt)

        # Keep track of front sizes over the generations
        self.hist.append(front_size)

        # If we have fewer than `n_last` generations, continue
        if len(self.hist) < self.n_last:
            return True

        # Compute the difference in front size over the last `n_last` generations
        diff = abs(self.hist[-1] - self.hist[-self.n_last])

        # Continue if the difference is larger than `min_diff`
        return diff > self.min_diff

# Use the custom termination
termination = ParetoFrontTermination(min_diff=0.01, n_last=30)'''