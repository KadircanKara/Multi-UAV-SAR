# from PathSolution import PathSolution
# from PathProblem import PathProblem
from pymoo.core.callback import Callback
from pymoo.util.normalization import normalize
from PathFuncDict import model_metric_info

import numpy as np

class PathCallback(Callback):
    def __init__(self, model):
        super().__init__()
        self.model = model

    def normalize_objectives(self, algorithm):
        # Extract population data
        X = algorithm.pop.get("X")
        objective_names = self.model["F"][0].split("-")[:-1]

        # Compute raw objective values
        F = np.empty((X.shape[0], len(objective_names)), dtype=float)
        for i in range(X.shape[0]):
            sol = X[i][0]
            for j, obj_name in enumerate(objective_names):
                obj_calc = model_metric_info[obj_name]
                F[i, j] = obj_calc(sol)

        # Normalize objectives
        F_norm = F.copy().T
        for i, objective_values in enumerate(F_norm):
            min_val = np.min(objective_values)
            max_val = np.max(objective_values)
            if max_val - min_val > 1e-6:  # Avoid division by zero
                F_norm[i] = (objective_values - min_val) / (max_val - min_val)
            else:
                F_norm[i] = objective_values  # Leave as is if range is too small

        # Compute weighted sum (example: weights could come from the model)
        # weights = self.model["weights"]  # Example: assume weights are stored in the model
        weighted_F = np.sum(F_norm.T, axis=1)

        # Store the weighted sum in a custom attribute
        algorithm.pop.set("weighted_F", weighted_F)
        algorithm.pop.set("F", weighted_F)
        # print(weighted_F.shape)

    def notify(self, algorithm):
        # print(self.model["F"])
        # Call normalization and weighted sum computation if required
        if "Weighted Sum" in self.model["F"][0]:
            # print("Callback!")
            print(self.algorithm.pop.get("F"))
            self.normalize_objectives(algorithm)
            # Debug output for verification
            # print("Normalized Weighted Objectives:", algorithm.pop.get("weighted_F"))

        
        


"""class PathCallback(Callback):
    def __init__(self, model):
        super().__init__()
        self.model = model
    def normalize_objectives(self, algorithm):
        if not self.model["Alg"] == "GA" and "-" in self.model["F"]:
            pass
        # Get required objective values
        objective_names = self.model["F"][0].split("-")[:-1]
        # for i, obj_name in enumerate(objective_names):
        #     if i==0:
        #         objective_names[i] = obj_name[:-1]
        #     elif i==len(objective_names)-1:
        #         objective_names[i] = obj_name[1:]
        #     else:
        #         objective_names[i] = obj_name[1:-1]
        #     print(objective_names[i])
        # objective_names = [x[1:-1] for i,x in enumerate(objective_names) if 0<i<len(objective_names)-1 elif i==0 x[:-1] else x[1:]]
        X = algorithm.pop.get("X")
        # print(algorithm.pop.get("F").shape)
        F = np.empty((X.shape[0], len(objective_names)), dtype=float)
        for i in range(X.shape[0]):
            sol = X[i][0]
            for j, obj_name in enumerate(objective_names):
                obj_calc = model_metric_info[obj_name]
                F[i, j] = obj_calc(sol)
        # Normalize
        F_norm = F.copy().T
        for i, objective_values in enumerate(F_norm):
            objective_name = objective_names[i]
            # Pass mean disconnected time and percentage connectivity objectives as they are already scaled properly
            if objective_name == "Mean Disconnected Time" or objective_name == "Percentage Connectivity":
                continue
            min = np.min(objective_values)
            max = np.max(objective_values)
            normalized_objective_values = (objective_values - min) / (max - min) if objective_name != "Percentage Connectivity" else -(objective_values - min) / (max - min)
            if objective_name == "Percentage Connectivity":
                print(normalized_objective_values.shape)
                print(f"Objective: {objective_name}, Min: {min}, Max: {max}")
                print(f"Original,Normalized): {list(zip(objective_values, normalized_objective_values))}")

            F_norm[i] = normalized_objective_values
        # Set normalized F
        F_norm = np.array([np.sum(F_norm.T, axis=1)]).T
        algorithm.pop.set("F", F_norm)

    def notify(self, algorithm):
        # Call normalize_objectives during each iteration
        self.normalize_objectives(algorithm)"""