from pymoo.core.duplicate import DuplicateElimination
import numpy as np

class PathDuplicateElimination(DuplicateElimination):
    def __init__(self):
        super().__init__()

    def is_duplicate(self, X, others):
        """
        Eliminate solutions with the same mission time and percentage connectivity.
        
        Parameters:
        X : np.array
            New solutions to be checked (e.g., current generation individuals).
        others : np.array
            Existing solutions in the population.
        
        Returns:
        np.array
            A boolean array indicating if each solution in X is a duplicate.
        """
        duplicates = np.zeros(len(X), dtype=bool)

        for i, x in enumerate(X):
            for other in others:
                # Compare mission time and percentage connectivity
                if (x.mission_time == other.mission_time and
                    x.percentage_connectivity == other.percentage_connectivity):
                    duplicates[i] = True
                    break

        return duplicates