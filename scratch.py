from FilePaths import *
from PathFileManagement import *
import os
import copy
import pandas as pd
import numpy as np
from PathSolution import produce_n_tour_sol, PathSolution
from PathOptimizationModel import *

def fix_disconnected_times(sol:PathSolution):
    drone_disconnected_times = np.zeros(sol.info.number_of_drones)
    for time in range(sol.time_slots):
        # real_time_elapsed_at_step = sol.time_elapsed_at_steps
        adj_mat = sol.connectivity_matrix[time,1:,1:] # nxn array (n = number of nodes), exclude BS
            # Find disconnected nodes
        disconnected_rows = np.all(adj_mat == 0, axis=1)
        # Get the indices of disconnected nodes
        disconnected_drones = np.where(disconnected_rows)[0]
        for drone in disconnected_drones:
            drone_disconnected_times[drone] += 1

    sol.mean_disconnected_time = np.sum(drone_disconnected_times) / sol.info.number_of_drones * sol.time_slots
    sol.max_disconnected_time = np.max(drone_disconnected_times)





# import joblib

# data = joblib.load("Results/Objectives/SOO_GA_MTSP_g_8_a_50_n_4_v_2.5_r_sqrt(8)_nvisits_2-ObjectiveValues.pkl")

# with open("Results/Objectives/SOO_GA_MTSP_g_8_a_50_n_4_v_2.5_r_sqrt(8)_nvisits_2-ObjectiveValues.pkl", "rb") as file:
#     data = pickle.load(file)
#     print(data)


# test = np.load("Results/Objectives/SOO_GA_MTSP_g_8_a_50_n_4_v_2.5_r_sqrt(8)_nvisits_2-ObjectiveValues.pkl", allow_pickle=False)
# test = pd.read_pickle("Results/Objectives/SOO_GA_MTSP_g_8_a_50_n_4_v_2.5_r_sqrt(8)_nvisits_2-ObjectiveValues.pkl")

"""for filename in os.listdir(solutions_filepath):
    if "WS" in filename:
        print("Scenario:", filename.split("-")[0])
        filepath = f"{solutions_filepath}{filename}"
        X = load_pickle(filepath)
        for sol in X:
            sol.info.model = TCDT_WS
        save_as_pickle(filepath, X)
        # print(load_pickle(filepath)[0].info.model["F"])
"""        

"""for filename in os.listdir(objective_values_filepath):
    if "WS" in filename:
        print("Scenario:", filename.split("-")[0])
        filepath = f"{objective_values_filepath}{filename}"
        F = pd.read_pickle(filepath)
        assert (len(list(F.columns))==1), "Length greater than 1 !"
        old_column = list(F.columns)[0]
        print("Old Column:", old_column)
        new_column = old_column.replace("-", " & ").replace(" & Weighted Sum", " Weighted Sum")
        print("New Column:", new_column)
        F.columns = [new_column]
        F.to_pickle(filepath)
        # print("-->", list(F.columns))
        print("Post-Update Columns:", list(pd.read_pickle(filepath).columns))
        print("-------------------------------------------------------------------------------------------------------------------------------------------------------------")
"""
obj_dict = {
    "Mission Time":{"attribute":"mission_time", "normalization_factor":1000},
    "Percentage Connectivity": {"attribute":"percentage_connectivity", "normalization_factor":1},
    "Max Mean TBV": {"attribute":"max_mean_tbv", "normalization_factor":1},
    "Max Disconnected Time": {"attribute":"max_disconnected_time", "normalization_factor":1},
    "Mean Disconnected Time": {"attribute":"mean_disconnected_time", "normalization_factor":1},
}

"""
n_tours_list = 2,3,4,5,6,7,8,9,10
# print(n_tours_list)

sols_dir = os.listdir(solutions_filepath)
# sols_dir.reverse()
# print(sols_dir)

for filename in sols_dir:
    scenario = filename.split("-")[0]
    # Debug: Print the filename being processed
    print(f"Processing filename: {filename}")
    
    # Ensure case-insensitive matching
    if "nvisits_1" in scenario and "MTSP" in scenario:
        print(f"Matched 'nvisits_1': {filename}")
        
        # Skip files containing "Weighted Sum" (case-insensitive)
        # if "weighted sum" in filename.lower():
        #     print(f"Skipping 'Weighted Sum' file: {filename}")
        #     continue
        
        # Proceed with processing
        F = copy.deepcopy(pd.read_pickle(f"{objective_values_filepath}{scenario}-ObjectiveValues.pkl"))
        # if "Weighted Sum" in objs.columns[0]:
        #     continue
        X = copy.deepcopy(load_pickle(f"{solutions_filepath}{filename}"))
        R = load_pickle(f"{runtimes_filepath}{scenario}-Runtime.pkl")
        
        for n_tour in n_tours_list:
            n_tour_scenario = scenario.replace("nvisits_1", f"ntours_{n_tour}")
            # Solution Objects
            # if f"{solutions_filepath}{n_tour_scenario}-SolutionObjects.pkl" not in os.listdir(solutions_filepath):
            X_ntour = X.copy()
            for i in range(len(X_ntour)):
                X_ntour[i] = produce_n_tour_sol(X_ntour[i], n_tour)
            save_as_pickle(f"{solutions_filepath}{n_tour_scenario}-SolutionObjects.pkl", X_ntour)
            # upload_file(f"{solutions_filepath}{n_tour_scenario}-SolutionObjects.pkl", PARENT_FOLDER_ID_DICT["Solutions"]) if copy_to_drive else None
            # Objective Values
            # if f"{objective_values_filepath}{n_tour_scenario}-ObjectiveValues.pkl" not in os.listdir(objective_values_filepath):
            # X_ntour = load_pickle(f"{solutions_filepath}{n_tour_scenario}-SolutionObjects.pkl")
            F_columns = list(F.columns)
            # if self.model["Type"]=="WS":
            #     F_data = [calculate_ws_score_from_ws_objective(x) for x in X_ntour]
            # else:
            F_data = []
            for obj in F_columns:
                obj_values = []
                for sol in X_ntour:
                    if "Weighted Sum" in obj:
                        obj_values.append(calculate_ws_score_from_ws_objective(sol))
                    else:
                        obj_values.append(getattr(sol, obj_name_sol_attr_dict[obj]))
                F_data.append(obj_values)
            F_data = np.array(F_data).T
            # print(f"Data: {F_data}")
            F_ntour = pd.DataFrame(data=F_data, columns=F_columns)
            save_as_pickle(f"{objective_values_filepath}{n_tour_scenario}-ObjectiveValues.pkl", F_ntour)
            # upload_file(f"{objective_values_filepath}{n_tour_scenario}-ObjectiveValues.pkl", PARENT_FOLDER_ID_DICT["Objectives"]) if copy_to_drive else None
            # if f"{objective_values_filepath}{n_tour_scenario}-ObjectiveValuesAbs.pkl" not in os.listdir(objective_values_filepath):
            F_abs_ntour = abs(pd.read_pickle(f"{objective_values_filepath}{n_tour_scenario}-ObjectiveValues.pkl"))
            save_as_pickle(f"{objective_values_filepath}{n_tour_scenario}-ObjectiveValuesAbs.pkl", F_abs_ntour)
            # upload_file(f"{objective_values_filepath}{n_tour_scenario}-ObjectiveValuesAbs.pkl", PARENT_FOLDER_ID_DICT["Objectives"]) if copy_to_drive else None
            # Runtimes
            # if f"{runtimes_filepath}{n_tour_scenario}-Runtime.pkl" not in os.listdir(runtimes_filepath):
            save_as_pickle(f"{runtimes_filepath}{n_tour_scenario}-Runtime.pkl", R) # Runtime does not change
"""




            # n_tours_sols = sols.copy()
            # n_tours_F = objs.copy()
            # for row, sol in enumerate(n_tours_sols):
            #     sol = produce_n_tour_sol(sol, n_tours)
            #     if len(n_tours_F.columns) > 1:
            #         for col in n_tours_F.columns:
            #             obj = obj_dict[col]["attribute"]
            #             n_tours_F[col].iloc[row] = getattr(sol, obj)
            #     else:
            #         score = 0
            #         objectives = sol.info.model["F"]
            #         for objective in objectives:
            #             score += (
            #                 getattr(sol, obj_dict[objective]["attribute"]) *
            #                 obj_dict[objective]["normalization_factor"]
            #             )
            #         n_tours_F.iloc[row] = score

            # save_as_pickle(f"{solutions_filepath}{filename.replace('nvisits_1', f'ntours_{n_tours}')}", n_tours_sols)
            # save_as_pickle(f"{objective_values_filepath}{scenario.replace('nvisits_1', f'ntours_{n_tours}')}-ObjectiveValues.pkl", n_tours_F)
            # save_as_pickle(f"{objective_values_filepath}{scenario.replace('nvisits_1', f'ntours_{n_tours}')}-ObjectiveValuesAbs.pkl", abs(n_tours_F))
            # save_as_pickle(f"{runtimes_filepath}{scenario.replace('nvisits_1', f'ntours_{n_tours}')}-Runtime.pkl", runtime)

"""for filename in sols_dir:
    scenario = filename.split("-")[0]
    if "nvisits_1" in filename:
        print(filename)
        if "Weighted Sum" in filename:
            continue
        sols = copy.deepcopy(load_pickle(f"{solutions_filepath}{filename}"))
        objs = copy.deepcopy(pd.read_pickle(f"{objective_values_filepath}{scenario}-ObjectiveValues.pkl"))
        runtime = load_pickle(f"{runtimes_filepath}{scenario}-Runtime.pkl")
        for n_tours in n_tours_list:
            # if f"{solutions_filepath}{filename.replace("nvisits_1",f"ntours_{n_tours}")}" in os.listdir(solutions_filepath):
            #     continue
            n_tours_sols = sols.copy()
            n_tours_F = objs.copy()
            for row,sol in enumerate(n_tours_sols):
                sol = produce_n_tour_sol(sol, n_tours)
                if len(n_tours_F.columns) > 1:
                    for col in n_tours_F.columns:
                        obj = obj_dict[col]["attribute"]
                        n_tours_F[col].iloc[row] = getattr(sol, obj)
                else:
                    score = 0
                    objectives = sol.info.model["F"]
                    for objective in objectives:

                        score += ( getattr(sol, obj_dict[objective]["attribute"]) * obj_dict[objective]["normalization_factor"] )
                    n_tours_F.iloc[row] = score

            save_as_pickle( f"{solutions_filepath}{filename.replace('nvisits_1',f'ntours_{n_tours}')}", n_tours_sols)
            save_as_pickle( f"{objective_values_filepath}{scenario.replace('nvisits_1',f'ntours_{n_tours}')}-ObjectiveValues.pkl",  n_tours_F)
            save_as_pickle( f'{objective_values_filepath}{scenario.replace("nvisits_1",f"ntours_{n_tours}")}-ObjectiveValuesAbs.pkl',  abs(n_tours_F))
            save_as_pickle( f'{runtimes_filepath}{scenario.replace("nvisits_1",f"ntours_{n_tours}")}-Runtime.pkl',  runtime) # Add runtimes too for consistency
"""



        



"""dirs = [solutions_filepath, objective_values_filepath, runtimes_filepath]

for dir in dirs:
    filenames = os.listdir(dir)
    for filename in filenames:
        split_filename = filename.split("_")
        type_ = split_filename[0]
        exp = filename.split("_")[2]
        if type_ == "SOO" and exp == "TCDT":
            new_filename = filename.replace("SOO","WS")
            # print(f"Original Scenario: {filename.split('-')[0]}\nNew Scenario: {new_filename.split('-')[0]}")
            os.rename(f"{dir}{filename}", f"{dir}{new_filename}")
        # split_filename = filename.split("_")
        # # print(split_filename)
        # split_filename = split_filename[:2] + ["MTSP"] + split_filename[2:]
        # new_filename = ""
        # for i in range(len(split_filename)):
        #     new_filename += split_filename[i] + "_"
        # new_filename = new_filename[:-1]
        # # print(new_filename)
        # # new_filename = filename.replace("minv", "nvisits")
        # os.rename(f"{dir}{filename}", f"{dir}{new_filename}")
        # print(f"Renamed {filename} to {new_filename}")"""