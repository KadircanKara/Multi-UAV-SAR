from posixpath import split
import matplotlib
from matplotlib import pyplot as plt
import os
from FilePaths import *
from PathFileManagement import *
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

from PathAnimation import PathAnimation
from PathSolution import PathSolution
from Time import get_real_paths
from Connectivity import calculate_disconnected_timesteps
from PathInput import *
from PathInfo import *
from PathFuncDict import model_metric_info
if os.path.isfile("GoogleDriveUpload.py"):
    from GoogleDriveUpload import upload_file, authenticate, PARENT_FOLDER_ID_DICT

# from Report import get_attribute

# def get_attribute()


def list_files(directory):
    files = []
    # Walk through the directory
    for dirpath, dirnames, filenames in os.walk(directory):
        for filename in filenames:
            files.append(os.path.join(dirpath, filename))
    return files

def save_best_solutions(scenario:str, copy_to_drive=False):
    X = load_pickle(f"{solutions_filepath}{scenario}-SolutionObjects.pkl").copy().flatten()
    F = pd.read_pickle(f"{objective_values_filepath}{scenario}-ObjectiveValues.pkl").copy()
    all_objectives = model_metric_info["Objectives"].keys()
    # TODO: Calculate a new F that includes all objective value information (even the ones not in the original optimization problem).
    for objective_name in all_objectives:
        # Skip if the objective value information is already in F
        if objective_name in F.columns:
            continue
        # Calculate the attribute(s) not in the original F
        obj_calc_func = model_metric_info["Objectives"][objective_name][0]
        obj_pol = model_metric_info["Objectives"][objective_name][1]
        F[objective_name] = [obj_calc_func(sol)*obj_pol for sol in X]
        # print(objective_name)
        # print(F[objective_name])
    # print(F)
    # TODO: Get the best solution for each objective using normalization and comparison of other objectives
    for objective_name in all_objectives:
        # print(objective_name)
        # Get best indices
        obj_values = F[objective_name]
        min_obj_value = obj_values.min()
        best_obj_sol_ind = obj_values[obj_values == min_obj_value].index.tolist() # F[objective_name].idxmin()
        # print(best_obj_sol_ind)
        if len(best_obj_sol_ind) > 0:
            # Normalize other objectives and pick the solution corresponding to the best overall objective value
            f = F.copy()
            f = f.drop(columns=[objective_name]) # Drop the objective column
            f_norm = f.copy()
            for col in f_norm.columns:
                f_norm[col] = (f_norm[col] - f_norm[col].min()) / (f_norm[col].max() - f_norm[col].min()) if f_norm[col].max() != f_norm[col].min() else 0
            # f_norm = (f - f.min() / (f.max() - f.min())).iloc[best_obj_sol_ind]
            # print(f_norm)
            f_norm["Sum"] = f_norm.sum(axis=1)
            # print(f_norm)
            best_obj_sol_ind = f_norm["Sum"].idxmin()
        # print(best_obj_sol_ind)
        best_obj_sol = X[best_obj_sol_ind]
        # Save the best solution
        save_as_pickle(f"{solutions_filepath}{scenario}-Best-{objective_name.replace(' ','_')}-Solution.pkl", best_obj_sol)
        upload_file(f"{solutions_filepath}{scenario}-Best-{objective_name.replace(' ','_')}-Solution.pkl", PARENT_FOLDER_ID_DICT["Solutions"]) if copy_to_drive else None


                





def animate_extreme_point_paths(info:PathInfo):

    scenario = str(info)
    directions = ["Best", "Mid", "Worst"]
    objectives = [x.replace(" ", "_") for x in info.model["F"]]

    # Iterate through each objective
    for obj in objectives:
        fig, axes = plt.subplots(1, 3, figsize=(15, 5))  # Create subplots with 1 row and 3 columns
        fig.suptitle(f"Obj: {model['Exp']}, Alg: {model['Alg']}, Gen: {info.n_gen}, Pop: {info.pop_size}, n={info.number_of_drones}, r={info.comm_cell_range}, v:{info.n_visits}")
        # title = f"{direction} {obj.replace('_',' ')} Paths"
        animations = []  # Store all animations to prevent garbage collection

        for ax, direction in zip(axes, directions):
            ax.set_title(f"{direction} {obj.replace('_',' ')} Paths")
            sol = load_pickle(f"Results/Solutions/{scenario}-{direction}-{obj}-Solution.pkl")
            anim_object = PathAnimation(sol, fig, ax)
            anim = FuncAnimation(
                fig, anim_object.update, frames=anim_object.paths[0].shape[1],
                init_func=anim_object.initialize_figure, blit=False, interval=0
            )
            animations.append(anim)  # Store the animation object

        plt.tight_layout()
        plt.show()



def plot_total_distance_vs_pecentage_connectivity(direction:str, obj:str):

    obj_with_underscore = obj.replace(" ","_")

    # List all objective value pkl files
    X_files = list_files(solutions_filepath)

    r_2_X_files = [file for file in X_files if ("r_2" in file and "minv_1" in file and "SOO" not in file and f"{direction}-{obj_with_underscore}" in file)]
    r_4_X_files = [file for file in X_files if ("r_4" in file and "minv_1" in file and "SOO" not in file and f"{direction}-{obj_with_underscore}" in file)]

    number_of_drones_list = [2,4,8,12,16,20]

    plt.figure(figsize=(10,20))
    plt.suptitle(f"{direction} {obj} Results")

    # Plot for r=2
    total_distance_values = list(map(lambda x: load_pickle([file for file in r_2_X_files if load_pickle(file).info.number_of_drones==x][0]).total_distance, number_of_drones_list))
    percentage_connectivity_values = list(map(lambda x: load_pickle([file for file in r_2_X_files if load_pickle(file).info.number_of_drones==x][0]).percentage_connectivity, number_of_drones_list))
    disconnectivity_values = list(map(lambda x: calculate_disconnected_timesteps(load_pickle([file for file in r_2_X_files if load_pickle(file).info.number_of_drones==x][0])), number_of_drones_list))
    mean_disconnectivity_values = [np.mean(x) for x in disconnectivity_values]
    max_disconnectivity_values = [np.max(x) for x in disconnectivity_values]

    # print(percentage_connectivity_values)
    plt.subplot(2, 4, 1)
    # fig = plt.figure()
    # ax = fig.axes
    plt.title("r=2")
    plt.xlabel("Number of Drones")
    plt.ylabel("Total Distance")
    plt.xticks(number_of_drones_list)
    plt.grid()
    # plt.ylim((0,1))
    plt.scatter(x=number_of_drones_list, y=total_distance_values, color="blue")
    plt.plot(number_of_drones_list, total_distance_values, color='blue', linestyle='-', label='Line')

    plt.subplot(2, 4, 2)
    # ax = fig.axes
    plt.title("r=2")
    plt.xlabel("Number of Drones")
    plt.ylabel("Percentage Connectivity")
    plt.xticks(number_of_drones_list)
    plt.ylim((0,1.1))
    plt.grid()
    # plt.ylim((0,1))
    plt.scatter(x=number_of_drones_list, y=percentage_connectivity_values, color='blue')
    plt.plot(number_of_drones_list, percentage_connectivity_values, color='blue', linestyle='-', label='Line')

    plt.subplot(2, 4, 3)
    # ax = fig.axes
    plt.title("r=2")
    plt.xlabel("Number of Drones")
    plt.ylabel("Mean Disconnectivity")
    plt.xticks(number_of_drones_list)
    # plt.ylim((0,1.1))
    plt.grid()
    # plt.ylim((0,1))
    plt.scatter(x=number_of_drones_list, y=mean_disconnectivity_values, color='blue')
    plt.plot(number_of_drones_list, mean_disconnectivity_values, color='blue', linestyle='-', label='Line')

    plt.subplot(2, 4, 4)
    # ax = fig.axes
    plt.title("r=2")
    plt.xlabel("Number of Drones")
    plt.ylabel("Max Disconnectivity")
    plt.xticks(number_of_drones_list)
    # plt.ylim((0,1.1))
    plt.grid()
    # plt.ylim((0,1))
    plt.scatter(x=number_of_drones_list, y=max_disconnectivity_values, color='blue')
    plt.plot(number_of_drones_list, max_disconnectivity_values, color='blue', linestyle='-', label='Line')


    # Plot for r=4
    total_distance_values = list(map(lambda x: load_pickle([file for file in r_4_X_files if load_pickle(file).info.number_of_drones==x][0]).total_distance, number_of_drones_list))
    percentage_connectivity_values = list(map(lambda x: load_pickle([file for file in r_4_X_files if load_pickle(file).info.number_of_drones==x][0]).percentage_connectivity, number_of_drones_list))
    disconnectivity_values = list(map(lambda x: calculate_disconnected_timesteps(load_pickle([file for file in r_4_X_files if load_pickle(file).info.number_of_drones==x][0])), number_of_drones_list))
    mean_disconnectivity_values = [np.mean(x) for x in disconnectivity_values]
    max_disconnectivity_values = [np.max(x) for x in disconnectivity_values]

    plt.subplot(2, 4, 5)
    # fig = plt.figure()
    # ax = fig.axes
    plt.title("r=4")
    plt.xlabel("Number of Drones")
    plt.ylabel("Total Distance")
    plt.xticks(number_of_drones_list)
    plt.grid()
    # plt.ylim((0,1))
    plt.scatter(x=number_of_drones_list, y=total_distance_values, color="blue")
    plt.plot(number_of_drones_list, total_distance_values, color='blue', linestyle='-', label='Line')

    plt.subplot(2, 4, 6)
    # ax = fig.axes
    plt.title("r=4")
    plt.xlabel("Number of Drones")
    plt.ylabel("Percentage Connectivity")
    plt.xticks(number_of_drones_list)
    plt.ylim((0,1.1))
    plt.grid()
    # plt.ylim((0,1))
    plt.scatter(x=number_of_drones_list, y=percentage_connectivity_values, color='blue')
    plt.plot(number_of_drones_list, percentage_connectivity_values, color='blue', linestyle='-', label='Line')

    plt.subplot(2, 4, 7)
    # ax = fig.axes
    plt.title("r=4")
    plt.xlabel("Number of Drones")
    plt.ylabel("Mean Disconnectivity")
    plt.xticks(number_of_drones_list)
    # plt.ylim((0,1.1))
    plt.grid()
    # plt.ylim((0,1))
    plt.scatter(x=number_of_drones_list, y=mean_disconnectivity_values, color='blue')
    plt.plot(number_of_drones_list, mean_disconnectivity_values, color='blue', linestyle='-', label='Line')

    plt.subplot(2, 4, 8)
    # ax = fig.axes
    plt.title("r=4")
    plt.xlabel("Number of Drones")
    plt.ylabel("Max Disconnectivity")
    plt.xticks(number_of_drones_list)
    # plt.ylim((0,1.1))
    plt.grid()
    # plt.ylim((0,1))
    plt.scatter(x=number_of_drones_list, y=max_disconnectivity_values, color='blue')
    plt.plot(number_of_drones_list, max_disconnectivity_values, color='blue', linestyle='-', label='Line')

    plt.subplots_adjust(left=0.1, right=0.9, top=0.9, bottom=0.1, wspace=0.4, hspace=0.4)

    plt.show()




    # print(total_distance_values)
    # print(percentage_connectivity_values)
    # for number_of_drones in number_of_drones_list:
    #     number_of_drones_X = load_pickle([file for file in r_2_X_files if f"n_{number_of_drones}" in file][0])
    #     plt.scatter(x=number_of_drones_X.total_distance, y=number_of_drones_X.percentage_connectivity)

    # plt.show()


    # # Create a new list with parsed strings
    # F_files_parsed = list(map(lambda x: x.split("_")[1:-1], F_files))
    # # Find r (comm_cell_range) and minv (n_visits) indices
    # r_index = F_files_parsed[0].index('r') + 1
    # minv_index = F_files_parsed[0].index('minv') + 1
    # # Get r=2, minv=1 files
    # # print(F_files_parsed[5][r_index])
    # r_2_F_files = [file for file in F_files if F_files_parsed[F_files.index(file)][r_index] == 2]
    # # Get r=4, minv=1 files
    # r_4_F_files = [F_files[i] for i in range(len(F_files)) if F_files_parsed[i][r_index] == 4 and F_files_parsed[i][minv_index] == 1]


    # F = load_pickle(f"{objective_values_filepath}{scenario}-ObjectiveValues.pkl")
    # distance_and_connectivity_values = F[["Total Distance","Percentage Connectivity"]]
    #


def save_paths_and_anims_from_scenario(scenario:str):

    # f"{objective_values_filepath}{self.scenario}_ObjectiveValues.pkl"
    # f"{solution_objects_filepath}{self.scenario}_SolutionObjects.pkl"

    F:pd.DataFrame = load_pickle(f"{objective_values_filepath}{scenario}-ObjectiveValues.pkl")
    X = load_pickle(f"{solutions_filepath}{scenario}-SolutionObjects.pkl")

    for objective_name in F.columns:
        objective_name_with_underscore = objective_name.replace(" ", "_")
        objective_values = F[objective_name]
        fig, axes = plt.subplots()
        if len(model["F"]) == 1: # If model is SOO
            sol = X[0]
            sol_xpath, sol_ypath = get_real_paths(sol)
            np.savez(f"{paths_filepath}{scenario}-Best-{objective_name_with_underscore}-Paths.npz", arr1=sol_xpath, arr2=sol_ypath)
            sol_anim = PathAnimation(sol, fig, axes)
            save_as_pickle(f"{animations_filepath}{scenario}-Best-{objective_name_with_underscore}-Animation.pkl", sol_anim)
        else: # If model is MOO
            # Get best, worst indices
            best_idx = objective_values.idxmin()
            worst_idx = objective_values.idxmax()
            if objective_name == "Percentage Connectivity" : best_idx, worst_idx = worst_idx, best_idx
            # Get median index
            sorted_objective_values = objective_values.sort_values().reset_index()
            print(f"Sorted {objective_name} Values:\n{sorted_objective_values}")
            n = len(sorted_objective_values)
            if n % 2 == 1:
                # If odd, take the middle element
                median_pos = n // 2
            else:
                # If even, take the lower middle element
                median_pos = n // 2 - 1
            median_row = sorted_objective_values.iloc[median_pos]
            mid_idx = int(median_row['index'])
            print(f"mid idx: {mid_idx}")
            # Get and save best, worst and mid solution objects
            best_sol:PathSolution = X[best_idx][0]
            worst_sol:PathSolution = X[worst_idx][0]
            mid_sol:PathSolution = X[mid_idx][0]
            save_as_pickle(f"{solutions_filepath}{scenario}-Best-{objective_name_with_underscore}-Solution.pkl", best_sol)
            # save_as_pickle(f"{solutions_filepath}{scenario}-Worst-{objective_name_with_underscore}-Solution.pkl", worst_sol)
            # save_as_pickle(f"{solutions_filepath}{scenario}-Mid-{objective_name_with_underscore}-Solution.pkl", mid_sol)
            # Save best, worst and mid paths
            best_sol_xpath, best_sol_ypath = get_real_paths(best_sol)
            worst_sol_xpath, worst_sol_ypath = get_real_paths(worst_sol)
            mid_sol_xpath, mid_sol_ypath = get_real_paths(mid_sol)
            np.savez(f"{paths_filepath}{scenario}-Best-{objective_name_with_underscore}-Paths.npz", arr1=best_sol_xpath, arr2=best_sol_ypath)
            np.savez(f"{paths_filepath}{scenario}-Worst-{objective_name_with_underscore}-Paths.npz", arr1=worst_sol_xpath, arr2=worst_sol_ypath)
            np.savez(f"{paths_filepath}{scenario}-Mid-{objective_name_with_underscore}-Paths.npz", arr1=mid_sol_xpath, arr2=mid_sol_ypath)
            # Save best, worst and mid animations
            best_sol_anim = PathAnimation(best_sol, fig, axes)
            worst_sol_anim = PathAnimation(worst_sol, fig, axes)
            mid_sol_anim = PathAnimation(mid_sol, fig, axes)
            save_as_pickle(f"{animations_filepath}{scenario}-Best-{objective_name_with_underscore}-Animation.pkl", best_sol_anim)
            save_as_pickle(f"{animations_filepath}{scenario}-Worst-{objective_name_with_underscore}-Animation.pkl", worst_sol_anim)
            save_as_pickle(f"{animations_filepath}{scenario}-Mid-{objective_name_with_underscore}-Animation.pkl", mid_sol_anim)
