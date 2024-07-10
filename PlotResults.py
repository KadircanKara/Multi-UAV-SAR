from copy import deepcopy
import os
import matplotlib
from matplotlib import pyplot as plt
from FileManagement import *
from FilePaths import *

from Connectivity import *
from PathSolution import PathSolution
from Time import get_real_paths, calculate_time_between_visits, calculate_visit_times, calculate_nvisits_and_visit_times_and_tbv

def list_files(directory):
    files = []
    # Walk through the directory
    for dirpath, dirnames, filenames in os.walk(directory):
        for filename in filenames:
            files.append(os.path.join(dirpath, filename))
    return files



def plot_number_of_drones_vs_parameter():

    # List all PathSolution files that contain all solutions found (not just best distance for example)
    all_sol_filenames = list_files(solutions_filepath)
    sol_filenames = [x for x in all_sol_filenames if "SolutionObjects" in x]
    # sols = []
    # scenarios = []
    number_of_drones_list = [2,4,8,12,16,20]
    # r_2_best_mission_times = [None]*len(number_of_drones_list) # For 2 drones, 4 drones, ..., 20 drones
    # r_4_best_mission_times = [None]*len(number_of_drones_list) # For 2 drones, 4 drones, ..., 20 drones
    moo_results = {"best_mission_time" : [None]*len(number_of_drones_list),
                   "avg_nvisits" : [None]*len(number_of_drones_list),
                   "max_nvisits" : [None]*len(number_of_drones_list),
                   "avg_max_time_between_visits" : [None]*len(number_of_drones_list),
                    }
    soo_results = deepcopy(moo_results)
    # moo_best_mission_times = [None]*len(number_of_drones_list) # For 2 drones, 4 drones, ..., 20 drones
    # soo_best_mission_times = [None]*len(number_of_drones_list) # For 2 drones, 4 drones, ..., 20 drones
    for x in sol_filenames:
        scenario = x.split("/")[2].split("-")[0]

        split_scenario = scenario.split("_")
        opt_type = split_scenario[0]
        alg = split_scenario[1]
        number_of_drones = split_scenario[split_scenario.index("n")+1]
        comm_cell_range = split_scenario[split_scenario.index("r")+1]
        minv = split_scenario[split_scenario.index("minv")+1]

        sols = load_pickle(x).flatten().tolist()



        mission_times = list(map(lambda x: get_real_paths(x)[0].shape[1], sols))
        best_mission_time = min(mission_times)

        best_mission_time_sol_ind = mission_times.index(best_mission_time)
        best_mission_time_sol = sols[best_mission_time_sol_ind]
        xs, ys = get_real_paths(best_mission_time_sol)

        nvisits, visit_times, time_between_visits = calculate_nvisits_and_visit_times_and_tbv(xs, ys, best_mission_time_sol.info)
        avg_nvisits = np.mean(nvisits)
        max_nvisits = max(nvisits)
        max_time_between_visits = list(map(lambda x: max(x), time_between_visits))
        avg_max_time_between_visits = np.mean(max_time_between_visits)

        # print(best_mission_time, avg_nvisits, max_nvisits, avg_max_time_between_visits)

        # tbvs = list(map(lambda x: calculate_time_between_visits(x), sols))
        # print("best mission time:", best_mission_time)

        # print(minv, comm_cell_range, opt_type, number_of_drones)

        for i,y in enumerate(number_of_drones_list):
            # if minv=="1" and comm_cell_range=="2" and opt_type=="MOO" and number_of_drones==str(y):
            #     r_2_best_mission_times[i] = best_mission_time
            # elif minv=="1" and comm_cell_range=="4" and opt_type=="MOO" and number_of_drones==str(y):
            #     r_4_best_mission_times[i] = best_mission_time
            if minv=="1" and opt_type=="SOO" and number_of_drones==str(y):
                soo_results["best_mission_time"][i] = best_mission_time
                soo_results["avg_max_time_between_visits"][i] = avg_max_time_between_visits
                soo_results["avg_nvisits"][i] = avg_nvisits
                soo_results["max_nvisits"][i] = max_nvisits
                # soo_best_mission_times[i] = best_mission_time
            elif minv=="1" and opt_type=="MOO" and number_of_drones==str(y):
                moo_results["best_mission_time"][i] = best_mission_time
                moo_results["avg_max_time_between_visits"][i] = avg_max_time_between_visits
                moo_results["avg_nvisits"][i] = avg_nvisits
                moo_results["max_nvisits"][i] = max_nvisits
                # moo_best_mission_times[i] = best_mission_time

    fig, axs = plt.subplots(2, 2, figsize=(10, 8))

    # plt.subplot(2,2,1)
    axs[0, 0].scatter(number_of_drones_list, soo_results["best_mission_time"])
    axs[0, 0].plot(number_of_drones_list, soo_results["best_mission_time"], linestyle="-", label="SOO")
    axs[0, 0].scatter(x=number_of_drones_list, y=moo_results["best_mission_time"])
    axs[0, 0].plot(number_of_drones_list, moo_results["best_mission_time"], linestyle="-", label="MOO")
    axs[0, 0].set_title("Number of drones vs. Best Mission Time")
    axs[0, 0].set_xticks(number_of_drones_list)
    axs[0, 0].legend()
    axs[0, 0].grid()

    # plt.subplot(2,2,2)
    axs[0, 1].scatter(number_of_drones_list, soo_results["avg_max_time_between_visits"])
    axs[0, 1].plot(number_of_drones_list, soo_results["avg_max_time_between_visits"], linestyle="-", label="SOO")
    axs[0, 1].scatter(x=number_of_drones_list, y=moo_results["avg_max_time_between_visits"])
    axs[0, 1].plot(number_of_drones_list, moo_results["avg_max_time_between_visits"], linestyle="-", label="MOO")
    axs[0, 1].set_title("Number of drones vs. Avg Max Time Between Visits")
    axs[0, 1].set_xticks(number_of_drones_list)
    axs[0, 1].legend()
    axs[0, 1].grid()

    # plt.subplot(2,2,3)
    axs[1, 0].scatter(number_of_drones_list, soo_results["avg_nvisits"])
    axs[1, 0].plot(number_of_drones_list, soo_results["avg_nvisits"], linestyle="-", label="SOO")
    axs[1, 0].scatter(x=number_of_drones_list, y=moo_results["avg_nvisits"])
    axs[1, 0].plot(number_of_drones_list, moo_results["avg_nvisits"], linestyle="-", label="MOO")
    axs[1, 0].set_title("Number of drones vs. Avg Number of Visits")
    axs[1, 0].set_xticks(number_of_drones_list)
    axs[1, 0].legend()
    axs[1, 0].grid()

    # plt.subplot(2,2,4)
    axs[1, 1].scatter(number_of_drones_list, soo_results["max_nvisits"])
    axs[1, 1].plot(number_of_drones_list, soo_results["max_nvisits"], linestyle="-", label="SOO")
    axs[1, 1].scatter(x=number_of_drones_list, y=moo_results["max_nvisits"])
    axs[1, 1].plot(number_of_drones_list, moo_results["max_nvisits"], linestyle="-", label="MOO")
    axs[1, 1].set_title("Number of drones vs. Max Number of Visits")
    axs[1, 1].set_xticks(number_of_drones_list)
    axs[1, 1].legend()
    axs[1, 1].grid()

    plt.show()

plot_number_of_drones_vs_parameter()



    # plt.xticks(number_of_drones_list)
    # plt.grid()
    # plt.legend()




    # plt.scatter(x=number_of_drones_list, y=r_2_best_mission_times)
    # plt.plot(number_of_drones_list, r_2_best_mission_times, linestyle="-", label="MOO R=2")
    # plt.scatter(x=number_of_drones_list, y=r_4_best_mission_times)
    # plt.plot(number_of_drones_list, r_4_best_mission_times, linestyle="-", label="MOO R=4")

    # PLOT MISSIN TIME
    
    # PLOT AVG, MAX VISITS PER CELL

    # print(f"r_2_best_mission_times: {r_2_best_mission_times}\nr_4_best_mission_times: {r_4_best_mission_times}\nsoo_best_mission_times: {soo_best_mission_times}")



        


        # scenarios.append(scenario)
        

        # sols.append(sol.flatten().tolist())
        # if not isinstance(sol, PathSolution):
        #     # print("->", sol.flatten().tolist())
        #     sols.append(sol.flatten().tolist())
        # else:
        #     sols.append(sol)

    # sol_filenames = list_files(solutions_filepath)

    # print(f"sol_filenames: {sol_filenames}")

    # List all desired solution objects
    # sols = []
    # scenarios = []
    # moo_r_2_sols = []
    # moo_r_4_sols = []
    # soo_sols = []
    # for x in sol_filenames:
    #     scenario = x.split("/")[2].split("-")[0]
    #     sol = load_pickle(x)
    #     if not isinstance(sol, PathSolution):
    #         # print("->", sol.flatten().tolist())
    #         sols.append(sol.flatten().tolist())
    #     else:
    #         sols.append(sol)
    #     scenarios.append(scenario)
    #     split_scenario = scenario.split("_")
    #     opt_type = split_scenario[0]
    #     alg = split_scenario[1]
    #     number_of_drones = split_scenario[split_scenario.index("n")+1]
    #     comm_cell_range = split_scenario[split_scenario.index("r")+1]
    #     minv = split_scenario[split_scenario.index("minv")+1]
    #     if opt_type=="MOO" and minv=="1" and comm_cell_range=="2":
    #         moo_r_2_sols.append(sols[-1])
    #     if opt_type=="MOO" and minv=="1" and comm_cell_range=="4":
    #         moo_r_4_sols.append(sols[-1])
    #     if opt_type=="SOO":
    #         soo_sols.append(sols[-1])

    # # print("->", len(sols), len(scenarios))
    # print(f"moo_r_2_sols: {moo_r_2_sols}\nmoo_r_4_sols: {moo_r_4_sols}\nsoo_sols: {soo_sols}")


    # print("->", sols)

    # sols = [load_pickle(x).tolist() for x in sol_filenames if not isinstance(load_pickle(x), PathSolution) else load_pickle(x)]

    # print("-->", sols)

    # List all PathInfo values and scenarios of the solutions
    # infos = []
    # scenarios = []
    # for i,x in enumerate(sols):
    #     if isinstance(x, list):
    #         infos.append(x[0].info)
    #         scenarios.append(str(infos[-1]))
    #         print(str(infos[-1])[:3])
    #     else:
    #         infos.append(x.info)
    #         scenarios.append(str(infos[-1]))

    # print(f"infos: {infos}\nscenarios: {scenarios}")


    # print("->", list(map(lambda x: str(x)[:3], infos)))

    # Filter solutions (min_visits=1)
    # desired_indices = []
    # for i,y in enumerate(sols):
    #     # print("->", str(y))
    #     if y.min_visits==1:
    #         desired_indices.append(i)

    # # desired_indices = np.where(infos_array.min_visits==1 and str(infos_array)[:3]=="MOO")
    # print("->", desired_indices)


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
