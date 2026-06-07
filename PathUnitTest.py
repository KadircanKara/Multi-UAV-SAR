from typing import Any
from PathAlgorithm import *
from PathOutput import *
from pymoo.optimize import minimize
from Results import animate_extreme_point_paths
from Time import *
from PathOptimizationModel import *
from PathInput import *
from FilePaths import *
from PathFileManagement import save_as_pickle

max_n_tour = 10

class PathUnitTest(object):

    def __init__(self, scenario) -> None:

        self.model = model
        self.algorithm = self.model["Alg"] # From PathInput

        self.info = [PathInfo(scenario)] if not isinstance(scenario, list) else list(map(lambda x: PathInfo(x), scenario))

    def __call__(self, save_results=True, animation=False, copy_to_drive=True, *args: Any, **kwds: Any) -> Any:

        for info in self.info:
            scenario = str(info)
            print(f"Scenario: {str(info)}")
            res, F, F_abs, X, R = self.run_optimization(info)
            if X is not None:
                if save_results:
                    # Save PathSolutions
                    save_as_pickle(f"{solutions_filepath}{scenario}-SolutionObjects.pkl", X)
                    # Save Objective Values
                    F.to_pickle(f"{objective_values_filepath}{scenario}-ObjectiveValues.pkl")
                    F_abs.to_pickle(f"{objective_values_filepath}{scenario}-ObjectiveValuesAbs.pkl")
                    # Save Runtimes
                    save_as_pickle(f"{runtimes_filepath}{scenario}-Runtime.pkl", R)
                    # Save n_tour files if necessary and if nvisits=1
                    if info.n_visits==1:
                        for n_tour in np.arange(2,max_n_tour+1):
                            n_tour_scenario = scenario.replace("nvisits_1", f"ntours_{n_tour}")
                            X_ntour = X.copy()
                            for i in range(len(X_ntour)):
                                X_ntour[i] = produce_n_tour_sol(X_ntour[i], n_tour)
                            save_as_pickle(f"{solutions_filepath}{n_tour_scenario}-SolutionObjects.pkl", X_ntour)
                            F_columns = info.model["F"]
                            F_data = []
                            for obj in F_columns:
                                obj_values = []
                                for sol in X_ntour:
                                    if info.model["Type"] == "WS":
                                        obj_values.append(calculate_ws_score_from_ws_objective(sol))
                                    else:
                                        obj_values.append(getattr(sol, obj_name_sol_attr_dict[obj]))
                                F_data.append(obj_values)
                            F_data = np.array(F_data).T
                            F_ntour = pd.DataFrame(data=F_data, columns=F_columns)
                            save_as_pickle(f"{objective_values_filepath}{n_tour_scenario}-ObjectiveValues.pkl", F_ntour)
                            F_abs_ntour = abs(pd.read_pickle(f"{objective_values_filepath}{n_tour_scenario}-ObjectiveValues.pkl"))
                            save_as_pickle(f"{objective_values_filepath}{n_tour_scenario}-ObjectiveValuesAbs.pkl", F_abs_ntour)
                            # Runtimes
                            save_as_pickle(f"{runtimes_filepath}{n_tour_scenario}-Runtime.pkl", R) # Runtime does not change

                if animation:
                    animate_extreme_point_paths(info)

                print(f"Scenario: {str(info)} COMPLETED")
            else:
                print(f"Scenario: {str(info)} NO SOLUTION FOUND")

            
    def run_optimization(self, info):

        problem = PathProblem(info)
        algorithm = PathAlgorithm(self.algorithm)()
        termination = ("n_gen", n_gen)
        # termination = NoTermination()
        output = PathOutput(problem)

        res, F, F_abs, X, R = None, None, None, None, None

        t = time.time()
        t_start = time.time()

        res = minimize(problem=PathProblem(info),
                        algorithm=algorithm,
                        termination=termination,
                        save_history=True,
                        seed=1,
                        output=output,
                        verbose=True,
                        )
        
        t_end = time.time()
        t_elapsed_seconds = t_end - t_start

        if res.X is not None:
            X = res.X.flatten() # FLATTEN NEW !
            F = pd.DataFrame(res.F, columns=model['F'])
            F_abs= abs(F)
            R = t_elapsed_seconds
            # If certain attributes are missing from the solution objects, add them here
            sample_sol = X[0][0] if isinstance(X[0], np.ndarray) else X[0]
            # Add TBV and Disconnecivity attributes to the solution objects if they are not already calculated
            for row in X:
                if isinstance(row, np.ndarray):
                    sol = row[0]
                else:
                    sol = row
                if not sample_sol.calculate_tbv:
                    sol.get_visit_times()
                    sol.get_tbv()
                    sol.get_mean_tbv()
                if not sample_sol.calculate_disconnectivity:
                    sol.do_disconnectivity_calculations()

        return res, F, F_abs, X, R