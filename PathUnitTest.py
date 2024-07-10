from statistics import median, median_low, median_high
from typing import Any
import pickle
from PathAlgorithm import *
from PathOutput import *
from pymoo.operators.crossover.nox import NoCrossover
from pymoo.operators.mutation.nom import NoMutation
from pymoo.core.duplicate import NoDuplicateElimination
from pymoo.optimize import minimize
from PathResults import save_paths_and_anims_from_scenario
# from PathTermination import PathTermination
from PathAnimation import PathAnimation
from Time import *

from PathOptimizationModel import moo_model_with_disconn, distance_soo_model
from PathInput import model, algorithm, test_setup_scenario, comprehensive_setup_scenarios
from FilePaths import objective_values_filepath, solutions_filepath, runtimes_filepath
from FileManagement import save_as_pickle, load_pickle


class PathUnitTest(object):

    def __init__(self, scenario) -> None:

        self.model = model
        self.algorithm = self.model["Alg"] # From PathInput

        self.info = [PathInfo(scenario)] if not isinstance(scenario, list) else list(map(lambda x: PathInfo(x), scenario))
        # self.info = [PathInfo(scenario)]

        # test_min_visits = test_setup_scenario['min_visits']
        # self.test_path = np.random.permutation(64*test_min_visits)
        # self.test_start_points = [0,16,32,48]*test_min_visits
        # self.test_info = PathInfo(test_setup_scenario)

        # self.model = model if model else moo_model
        # self.algorithm = algorithm if algorithm else 'NSGA2'
        # self.scenario = scenario if scenario else test_setup_scenario
        # self.scenario_info = PathInfo(self.scenario)
        # self.scenario_text = model['Exp']
        # for key in self.scenario.keys():
        #     self.scenario_text += ("_" + abbreviations[key] + "_" + str(self.scenario[key]))
        # # self.scenario_text = self.scenario_text[:-1] # Delete the last underscore (_)
        # print("-->", self.scenario_text)


    def __call__(self, *args: Any, **kwds: Any) -> Any:

        return self.Optimize(show_anim=False)

        # if type=='algorithm':
        #     self.Optimize()
        # if type=='animation':
        #     self.Animate
        # else:
        #     self.Optimize()

    # def Animate(self):

    #     path = np.random.permutation(64)
    #     start_points = [0,16,32,48]
    #     Nd = 8
    #     rc = 2*sqrt(2)
    #     info = PathInfo(Nd=Nd, rc=rc)

    #     # Run PathAnimation

    #     sol = PathSolution(path, start_points, info)

    #     # anim = PathAnimation(sol)
    #     # anim()

    def Optimize(self, show_anim):

        t = time.time()

        # n_gen = 5000 if model == distance_soo_model else 200 # 1000 for SOO (Only Dist), 4000 for MOO (Dist, Conn, Disconn)
        n_gen = 4000

        # termination = PathTermination(roi_threshold=0.01, patience=10)

        # results = []

        # if not isinstance(self.algorithms, list):
        #     algorithms = [self.algorithm]

        # for i in range(len(self.algorithms)):
        # Add algorithm to scenario
        # original[:index] + insert + original[index:]
        #
        # self.scenario = self.scenario[:4] + (self.algorithm + "_") + self.scenario[4:]

        for info in self.info:

            # if info.number_of_drones < 8:
            #     continue

            print(f"Scenario: {str(info)}")

            t_start = time.time()
            res = minimize(problem=PathProblem(info),
                            algorithm=PathAlgorithm(self.algorithm)(),# algorithm_dict[alg],
                            termination=('n_gen',n_gen),
                            # termination=termination,
                            save_history=True,
                            seed=1,
                            output=PathOutput(PathProblem(info)),
                            verbose=True,
                            # termination=path_termination
                            )
            t_end = time.time()
            t_elapsed_seconds = t_end - t_start
            t_elapsed_minutes = t_elapsed_seconds / 60
            # print(f"Elapsed time: {round(t_elapsed_minutes)} minutes") if floor(t_elapsed_minutes) > 0 else print(f"Elapsed time: {round(t_elapsed_seconds)} seconds")

            # results.append(res)

            F= pd.DataFrame(abs(res.F), columns=model['F'])
            F.to_pickle(f"{objective_values_filepath}{str(info)}-ObjectiveValues.pkl")

            X = res.X
            # print("-->",solution_objects_filepath,self.scenario)
            save_as_pickle(f"{solutions_filepath}{str(info)}-SolutionObjects.pkl", X)
            # X.to_pickle(f"{solution_objects_filepath}{self.scenario}-SolutionObjects.pkl")
            # np.save(f"{solution_objects_filepath}{self.scenario}_SolutionObjects.npy", X)

            R = t_elapsed_seconds
            save_as_pickle(f"{runtimes_filepath}{str(info)}-Runtime.pkl", R)
            # with open(f"{runtimes_filepath}{self.scenario}_Runtime.pkl", 'wb') as pickle_file:
            #     pickle.dump(R, pickle_file)

            print(f"Scenario: {str(info)}\nRuntime: {R} seconds\nObjective Values:\n{F}") # Print the results

            save_paths_and_anims_from_scenario(str(info))

            if show_anim:

                if self.model == distance_soo_model:
                    test_anim = load_pickle(f"Results/Animations/{str(info)}-Best-Total_Distance-Animation.pkl")
                else:
                    test_anim = load_pickle(f"Results/Animations/{str(info)}-Mid-Percentage_Connectivity-Animation.pkl")

                test_anim()




'''        # try:
        X = res.X
        F = res.F
        # print('F:',abs(F))


        scenario = self.scenario_text
        model = self.model
        # Convert F (np array to dataframe)
        F = pd.DataFrame(F,columns=model['F'])
        print('F:',abs(F).to_string(index=False))

        # SAVE OBJS AND SOL OBJECTS AND RUNTIMES
        np.save(f"Results/X/{self.scenario_text}_SolutionObjects",X)
        np.save(f"Results/F/{self.scenario_text}_ObjectiveValues",F)
        np.save(f"Results/Time/{self.scenario_text}_Runtime",elapsed_time)

        # LOAD OBJS AND SOL OBJECTS
        sols = np.load(f"Results/X/{self.scenario_text}_SolutionObjects.npy",allow_pickle=True)
        objs = np.load(f"Results/F/{self.scenario_text}_ObjectiveValues.npy",allow_pickle=True)

        # EXPORT TO MATLAB
        for ind, obj_name in enumerate(self.model['F']):
            obj_values = objs[:,ind].tolist()
            export_paths(py_path, mat_path, model, scenario, obj_name, obj_values) # Saves paths to both MATLAB and Python


        return pd.DataFrame(F,columns=model['F']) , X
'''
            # # sol = sols[0][0]

            # if self.model == distance_soo_model:
            #     dist_values = objs[0]
            # else:
            #     dist_values, subtour_values, conn_values, meanDisconn_values, maxDisconn_values = objs.transpose().tolist()

            # export_to_matlab(matlab_filepath, self.model, self.scenario_text, sols, "Dist", dist_values, inv_rel=False)
            # if self.model==moo_model:
            #     export_to_matlab(matlab_filepath, self.model, self.scenario_text, sols, "Subtour", subtour_values, inv_rel=False)
            #     export_to_matlab(matlab_filepath, self.model, self.scenario_text, sols, "Conn", conn_values, inv_rel=True)
            #     export_to_matlab(matlab_filepath, self.model, self.scenario_text, sols, "MeanDisconn", meanDisconn_values, inv_rel=False)
            #     export_to_matlab(matlab_filepath, self.model, self.scenario_text, sols, "MaxDisconn", maxDisconn_values, inv_rel=False)

        # except:
            # print("NO SOLUTION FOUND !!!")


# test = PathUnitTest(scenario=test_setup_scenario, model=model, algorithm=algorithm)
test = PathUnitTest(scenario=test_setup_scenario)
test('algorithm')



'''matlab_filepath = '/Users/kadircan/Documents/MATLAB/Thesis/HoveringPathResults'

model_list = [moo_model]
# algorithm_list = ['NSGA2','NSGA3']
number_of_drones_list = [8] # 8
r_comm_list = [2*sqrt(2)]
min_visits_list = [1] # 4,5
hovering_states = [True]
realtime_connectivity_states = [False]

for model in model_list:
    for alg in model['Alg']:
        for hovering in hovering_states:
            # info.hovering = hovering
            for realtime_connectivity in realtime_connectivity_states:
                # info.realtime_connectivity = realtime_connectivity
                for min_visits in min_visits_list:
                    # info.Nd = number_of_drones
                    for r_comm in r_comm_list:
                        # info.rc = r_comm
                        for number_of_drones in number_of_drones_list:

                            # info.min_visits = min_visits
                            info = PathInfo(hovering=hovering, realtime_connectivity=realtime_connectivity, Nd=number_of_drones, rc=r_comm, min_visits=min_visits)

                            scenario = f"{model['Exp']}_Opt_alg_{alg}_hovering_{info.hovering}_realtimeConnectivityCalculation_{info.realtime_connectivity}_n_{info.Nc}_Ns_{info.Nd}_comm_{info.rc}_nvisits_{info.min_visits}"

                            print(f"Opt: {model['Exp']}, Algorithm: {alg}, Hovering: {info.hovering}, Realtime Connectivity: {info.realtime_connectivity}, Number of Drones: {info.Nd}, Number of Nodes: {info.Nn}, Communication Range: {info.rc}, Min Visits: {info.min_visits}")

                            t = time.time()

                            res = minimize(problem=PathProblem(info,model=model),
                                          algorithm=algorithm_dict[alg],
                                          termination=('n_gen',1000),
                                          seed=1,
                                          # output=PathOutput(PathProblem(info)),
                                          verbose=True,
                                          # termination=path_termination
                                          )

                            print(res)

                            elapsed_time = time.time() - t

                            X = res.X
                            F = res.F
                            print('F:',abs(F))

                            print(f"Elapsed time: {round(elapsed_time/60)} minutes")

                            # Save verbose
                            # df = pd.DataFrame(res)
                            # df.to_excel(f"Results/Verbose/{scenario}_verbose.txt", index=False)
                            # np.save(f"{scenario}_Pop",[individual._X[0]  for individual in res.pop])
                            # np.save(f"{scenario}_Opt",[individual._X[0]  for individual in res.opt])
                            # print(f"X: {[individual._X[0]  for individual in res.pop]}")
                            # print(f"opt: {[individual._X[0]  for individual in res.opt]}")
                            # save_to_file(res, f"Results/Verbose/alg_{algorithm}_hovering_{info.hovering}_realtimeConnectivityCalculation_{info.realtime_connectivity}_n_{info.Nc}_Ns_{info.Nd}_comm_{info.rc}_nvisits_{info.min_visits}_verbose.txt")

                            # Save Solution Objects and Objective Values
                            np.save(f"Results/X/{scenario}_SolutionObjects",X)
                            np.save(f"Results/F/{scenario}_ObjectiveValues",F)
                            np.save(f"Results/Time/{scenario}_Runtime",elapsed_time)

                            sols = np.load(f"Results/X/{scenario}_SolutionObjects.npy",allow_pickle=True)
                            objs = np.load(f"Results/F/{scenario}_ObjectiveValues.npy",allow_pickle=True)

                            # sol = sols[0][0]

                            if model == distance_soo_model:
                                dist_values = objs[0]
                            else:
                                dist_values, subtour_values, conn_values, meanDisconn_values, maxDisconn_values = objs.transpose().tolist()

                            export_to_matlab(matlab_filepath, model, scenario, sols, "Dist", dist_values, inv_rel=False)
                            if model==moo_model:
                                export_to_matlab(matlab_filepath, model, scenario, sols, "Subtour", subtour_values, inv_rel=False)
                                export_to_matlab(matlab_filepath, model, scenario, sols, "Conn", conn_values, inv_rel=True)
                                export_to_matlab(matlab_filepath, model, scenario, sols, "MeanDisconn", meanDisconn_values, inv_rel=False)
                                export_to_matlab(matlab_filepath, model, scenario, sols, "MaxDisconn", maxDisconn_values, inv_rel=False)

                            # anim = PathAnimationTest()
                            # anim(sol, speed=20, title='random', subtitle='random random randomness')'''
