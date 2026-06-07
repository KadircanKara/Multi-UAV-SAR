from Sensing import sensing_and_discrete_info_sharing, sensing_and_realtime_info_sharing
from SensingReplay import SensingConfig
from PathSolution import *
from PathFileManagement import load_pickle
from FilePaths import *
import pandas as pd
import numpy as np
from matplotlib import pyplot as plt
from matplotlib import patches
from math import log

import re

from PathAnimation import *

objective_name_legend_dict = {
    "Mission Time": "\\tau",
    "Percentage Connectivity": "\\Psi",
    "Max Disconnected Time": "\\Phi_{max}",
    "Mean Disconnected Time": "\\Phi_{mean}",
    "Max Mean TBV": "\\Lambda"
}


def latex_escape(s):
    """Escape special LaTeX characters for matplotlib."""
    return re.sub(r'([#$%&_\{\}])', r'\\\1', s)

def get_model_label(model):
    exp = latex_escape(model["Exp"])
    typ = latex_escape(model["Type"])
    return f"${exp}_{{{typ}}}$"


def initialize_figure(size=(10,8), dpi=300, _title=[None, 14, 0], _suptitle=[None, 14], xlabel=[None, 14], ylabel=[None, 14], xticks=[None, 14], yticks=[None, 14], grid_on=True):
    fig,ax = plt.subplots(figsize=size, dpi=dpi)
    if _title[0] is not None: ax.set_title(_title[0], fontsize=_title[1], pad=_title[-1])
    if _suptitle[0] is not None: fig.suptitle(_suptitle[0], fontsize=_suptitle[1])
    if xlabel[0] is not None: ax.set_xlabel(xlabel[0], fontsize=xlabel[1])
    if ylabel[0] is not None: ax.set_ylabel(ylabel[0], fontsize=ylabel[1])
    if xticks[0] is not None: ax.set_xticks(xticks[0])
    ax.tick_params(axis='x', labelsize=xticks[1])
    if yticks[0] is not None: ax.set_yticks(yticks[0])
    ax.tick_params(axis='y', labelsize=yticks[1])
    if grid_on: ax.grid()

    return fig, ax

def get_path_snapshot_at_step(B, p, p0, model, direction, objective, n_targets, number_of_drones, comm_cell_range, merging_strategy, step):

    q=1-p
    # Get the scenario string
    info = PathInfo()
    info.model = model
    info.number_of_drones = number_of_drones
    info.comm_cell_range = comm_cell_range
    info.n_visits = ceil(log(p0*(1-B)/(B*(1-p0))) / ( (1-p) * log((1-q)/(1-p)) + (p*log(q/p)) ))
    scenario = str(info)
    # Load the solution object
    X = load_pickle(f"{solutions_filepath}{scenario}-SolutionObjects.pkl")
    F = load_pickle(f"{objective_values_filepath}{scenario}-ObjectiveValues.pkl")
    if direction=="Best":
        sol = X[F[objective].idxmin()]
    elif direction=="Median":
        sol = X[get_median_index_of_scenario(scenario)]
    else:
        sol = X[F[objective].idxmax()]

    # Randomly select target locations
    target_locations = np.random.choice(range(1, 64), n_targets, replace=False)

    # TODO: merging_strategy param is currently ignored; wire merge_topology=merging_strategy when these plots need topology comparisons
    cfg = SensingConfig.from_info(sol.info, merge_topology="onboard", time_model="discrete",
                                  target_locations=target_locations,
                                  detection_prob=p, false_alarm_prob=q, belief_threshold=B)
    time_metrics, new_sol = sensing_and_discrete_info_sharing(sol=sol, config=cfg)

    # Draw paths and grid (copy from PathAnimation)

    fig, ax = plt.subplots(dpi=300)

    search_paths = get_real_paths(sol)  # np.array([x_matrix, y_matrix])
    search_real_time_x_matrix, search_real_time_y_matrix = search_paths
    search_real_time_connectivity_matrix = get_real_connectivity_matrix(search_real_time_x_matrix, search_real_time_y_matrix, sol)


    paths = get_real_paths(new_sol)  # np.array([x_matrix, y_matrix])
    real_time_x_matrix, real_time_y_matrix = paths
    real_time_connectivity_matrix = get_real_connectivity_matrix(real_time_x_matrix, real_time_y_matrix, new_sol)
    # anim = PathAnimation(new_sol, fig, ax)
    # anim()
    # return
    # Draw paths
    n_drones = real_time_x_matrix.shape[0]
    for i in range(n_drones):
        # draw drone positions at step
        ax.plot(real_time_x_matrix[i][step], real_time_y_matrix[i][step], marker="o")
        ax.plot(search_real_time_x_matrix[i], search_real_time_y_matrix[i], linewidth=1)
    
    for i in range(n_drones):
        for j in range(i+1, n_drones):
            if real_time_connectivity_matrix[step, i, j]:
                x_coords = [real_time_x_matrix[i, step], real_time_x_matrix[j, step]]
                y_coords = [real_time_y_matrix[i, step], real_time_y_matrix[j, step]]
                ax.plot(x_coords, y_coords, color='black', linewidth=3)

    search_map = time_metrics["search map"]
    for target in target_locations:
        max_recent_prob = round( max( [search_map[:,target][i][-1]["prob"] for i in range(search_map.shape[0])] ), 2)
        target_x, target_y = PathSolution.get_coords(new_sol, target)
        plt.annotate(str(max_recent_prob), xy=(target_x, target_y), xytext=(0.0, 0.0), textcoords='offset points')
        bottom_left_x, bottom_left_y = target_x - new_sol.info.cell_side_length/2, target_y - new_sol.info.cell_side_length/2
        rect = patches.Rectangle((bottom_left_x, bottom_left_y),
            width=new_sol.info.cell_side_length,
            height=new_sol.info.cell_side_length,
            linewidth=1,
            edgecolor='none',         # no border
            facecolor='orange',       # fill color
            alpha=0.3)                # transparency      
        ax.add_patch(rect)

    # Write Highest probability inside cell
    # ax.legend()
    # plt.title("Drone Paths")
    plt.xlabel("X Coordinate")
    plt.ylabel("Y Coordinate")
    

    x_ticks_values = [i for i in range(-new_sol.info.cell_side_length, 
                                        (new_sol.info.grid_size + 1) * new_sol.info.cell_side_length, 
                                        new_sol.info.cell_side_length)]
    y_tick_values = x_ticks_values.copy()
    x_ticks_labels = [i for i in range(-1, new_sol.info.grid_size + 1)]
    y_tick_labels = x_ticks_labels.copy()

    ax.set_xticks(x_ticks_values)
    ax.set_xticklabels(x_ticks_labels)
    ax.set_yticks(y_tick_values)
    ax.set_yticklabels(y_tick_labels)
    ax.grid(linestyle='--')

    plt.show()
    fig.savefig(f"Figures/Sensing/{scenario}_step{step}_snapshot.png", dpi=300, bbox_inches='tight')    


def animate_mission(B, p, p0, model, direction, objective, n_targets, number_of_drones, comm_cell_range, merging_strategy):

    q=1-p
    # Get the scenario string
    info = PathInfo()
    info.model = model
    info.number_of_drones = number_of_drones
    info.comm_cell_range = comm_cell_range
    info.n_visits = ceil(log(p0*(1-B)/(B*(1-p0))) / ( (1-p) * log((1-q)/(1-p)) + (p*log(q/p)) ))
    scenario = str(info)
    # Load the solution object
    X = load_pickle(f"{solutions_filepath}{scenario}-SolutionObjects.pkl")
    F = load_pickle(f"{objective_values_filepath}{scenario}-ObjectiveValues.pkl")
    if direction=="Best":
        sol = X[F[objective].idxmin()]
    elif direction=="Median":
        sol = X[get_median_index_of_scenario(scenario)]
    else:
        sol = X[F[objective].idxmax()]

    # Randomly select target locations
    target_locations = np.random.choice(range(1, 64), n_targets, replace=False)

    # TODO: merging_strategy param is currently ignored; wire merge_topology=merging_strategy when these plots need topology comparisons
    cfg = SensingConfig.from_info(sol.info, merge_topology="onboard", time_model="discrete",
                                  target_locations=target_locations,
                                  detection_prob=p, false_alarm_prob=q, belief_threshold=B)
    time_metrics, new_sol = sensing_and_discrete_info_sharing(sol=sol, config=cfg)

    # print(time_metrics["detection time"])

    search_map = time_metrics["search map"]

    cell_occupancy_probabilities = time_metrics["cell occupancy probabilities"]

    # Draw paths and grid (copy from PathAnimation)

    fig, ax = plt.subplots(figsize=(10, 8))

    search_paths = get_real_paths(sol)  # np.array([x_matrix, y_matrix])
    search_real_time_x_matrix, search_real_time_y_matrix = search_paths
    search_real_time_connectivity_matrix = get_real_connectivity_matrix(search_real_time_x_matrix, search_real_time_y_matrix, sol)


    paths = get_real_paths(new_sol)  # np.array([x_matrix, y_matrix])
    real_time_x_matrix, real_time_y_matrix = paths
    real_time_connectivity_matrix = get_real_connectivity_matrix(real_time_x_matrix, real_time_y_matrix, new_sol)
    anim = PathAnimation(new_sol, fig, ax, target_locations=target_locations, cell_occupancy_probabilities=cell_occupancy_probabilities, p0=p0, B=B)
    anim()
    return


def plot_time_metrics_specific_models(n_runs, p0, B_list, p_list, comm_range_list, number_of_drones_list, n_targets_list,
                                      models, visits_or_tours, merging_capabilities, merging_strategies, objectives, directions,
                                      title_on, folder_name, show, save):

    rl_2 = load_pickle("Figures/Sensing RL Comparison/plotting_results_rc_2_p_0.8.pkl")
    rl_sqrt8 = load_pickle("Figures/Sensing RL Comparison/plotting_results_rc_sqrt(8)_p_0.8.pkl")

    objective_name_legend_dict = {
        "Mission Time": "$\\tau$",
        "Percentage Connectivity": "$\\Psi$",
        "Max Disconnected Time": "$\\Phi_{max}$",
        "Mean Disconnected Time": "$\\Phi_{mean}$",
        "Max Mean TBV": "$\\Lambda$"
        # "Mission Time": "$\Tau$",
        # "Percentage Connectivity": "$\Psi$",
        # "Max Disconnected Time": "$\Phi_{max}$",
        # "Mean Disconnected Time": "$\Phi_{mean}$",
        # "Max Mean TBV": "\Lambda"
    }

    color_dict = {
        "MTSP_SOO": "black",
        "TC_MOO": "blue",
        "TCT_MOO": "red",
        "TCDT_MOO": "red",
        "TCD_MOO": "green"
    }
    linestyles_dict = {
        "Mission Time":"-",
        "Percentage Connectivity":"--",
        "Max Mean TBV":"-.",
        "Mean Disconnected Time":":",
        "Max Disconnected Time": "-."
    }

    markers_dict = {
        "visit":"^",
        "tour":"o",
    }

    # markers_dict = {
    #     "realtime":"^",
    #     "discrete":"o",
    # }

    markerfacecolors_dict = {
        "onboard": "None",
        "gcs": "auto",
    }

    colors = ["black", "blue", "red", "magenta"]
    linestyles = ["-", ":", "--", "-."]
    markers = ["^", "o", "x", "D"]
    markerfacecolors = ["None","auto"]

    figsize = (8,6)
    linewidth = 1.0
    title_pad = 0
    small_fontsize = 10
    medium_fontsize = 12
    big_fontsize = 16

    info = PathInfo()

    title = [parameters, big_fontsize, title_pad] if title_on else [None, 0, 0]
    dt_suptitle = ["Detection Time", big_fontsize] if title_on else [None, 0]
    it_suptitle = ["Inform Time", big_fontsize] if title_on else [None, 0]
    # alo_suptitle = ["Time at Least One Drone Knows All Targets", big_fontsize] if title_on else [None, 0]
    mt_suptitle = ["Mission Time", big_fontsize] if title_on else [None, 0]
    # sr_suptitle = ["Mission Success Rate", big_fontsize] if title_on else [None, 0]

    for B in B_list:
        for p in p_list:
            q = 1-p
            # m = ceil( log10((p0*(1-B))/(B*(1-p0))) / log10(q/p) )
            m = ceil(log(p0*(1-B)/(B*(1-p0))) / ( (1-p) * log((1-q)/(1-p)) + (p*log(q/p)) ))
            info.n_visits = m
            # print("m:",m)
            for comm_range in comm_range_list:
                info.comm_cell_range = comm_range
                for n_targets in n_targets_list:
                    # parameters = f"runs: {n_runs}, B: {round(B,2)}, p: {round(p,2)}, q: {round(q,2)}, T: {n_targets}, $r_c$: {comm_range}, " + "$n_{visits}$: " + str(m)
                    # Initialize the figures and y_data
                    detection_time_fig, detection_time_ax = initialize_figure(_title=title, _suptitle=dt_suptitle,
                                                                              size=figsize, dpi=300, xlabel=["Number of Drones", big_fontsize], ylabel=["Detection Time (s)", big_fontsize], xticks=[number_of_drones_list, big_fontsize], yticks=[None, big_fontsize], grid_on=True
                                                                              )
                    inform_time_fig, inform_time_ax = initialize_figure(_title=title, _suptitle=it_suptitle,
                                                                        size=figsize, dpi=300, xlabel=["Number of Drones", big_fontsize], ylabel=["Inform Time (s)", big_fontsize], xticks=[number_of_drones_list, big_fontsize], yticks=[None, big_fontsize], grid_on=True
                                                                        )
                    # time_at_least_one_drone_knows_all_targets_fig, time_at_least_one_drone_knows_all_targets_ax = initialize_figure(_title=title, _suptitle=alo_suptitle,
                    #                                                                                                                 size=figsize, dpi=300, xlabel=["Number of Drones", big_fontsize], ylabel=["ALO Time (s)", big_fontsize], xticks=number_of_drones_list, yticks=None, grid_on=True
                    #                                                                                                                 )
                    mission_time_fig, mission_time_ax = initialize_figure(_title=title, _suptitle=mt_suptitle,
                                                                          size=figsize, dpi=300, xticks=[number_of_drones_list, big_fontsize], yticks=[None, big_fontsize], grid_on=True, xlabel=["Number of Drones", big_fontsize], ylabel=["Mission Time (s)", big_fontsize],
                                                                          )
                    # success_rate_fig, success_rate_ax = initialize_figure(_title=title, _suptitle=sr_suptitle,
                    #                                                       size=figsize, dpi=300, xlabel=["Number of Drones", big_fontsize], ylabel=["Mission Success Rate (%)", big_fontsize], xticks=number_of_drones_list, yticks=None, grid_on=True
                    #                                                       )
                    
                    figs = [detection_time_fig, inform_time_fig, mission_time_fig]
                    axes = [detection_time_ax, inform_time_ax, mission_time_ax]
                    # success_rate_ax.set_ylim(0,1.1)

                    # figs = [detection_time_fig, inform_time_fig, time_at_least_one_drone_knows_all_targets_fig, mission_time_fig, success_rate_fig]
                    # axes = [detection_time_ax, inform_time_ax, time_at_least_one_drone_knows_all_targets_ax, mission_time_ax, success_rate_ax]
                    # success_rate_ax.set_ylim(0,1.1)

                    # = [np.zeros(len(number_of_drones_list)) for _ in range(10)]
                    # y_detection_time, y_inform_time, y_alo_time, y_mission_time, y_successful_runs = [np.zeros(len(number_of_drones_list)) for _ in range(10)]
                    # print(y_detection_time)
                    
                    y_detection_time = [np.zeros(len(number_of_drones_list)) for _ in range(10)]
                    y_inform_time = [np.zeros(len(number_of_drones_list)) for _ in range(10)]
                    # y_alo_time = [np.zeros(len(number_of_drones_list)) for _ in range(10)]
                    y_mission_time = [np.zeros(len(number_of_drones_list)) for _ in range(10)]
                    # y_successful_runs = [np.zeros(len(number_of_drones_list)) for _ in range(10)]

                    # models =                [MTSP, MTSP] # [MTSP, TCDT_MOO_NSGA2, TCT_MOO_NSGA2, TC_MOO_NSGA2]
                    # objectives =            [["Mission Time"], ["Mission Time"]] # [["Mission Time"], ["Mission Time", "Mean Disconnected Time", "Max Mean TBV"], ["Mission Time", "Max Mean TBV"], ["Mission Time"]]
                    # directions =            ["Best", "Best"] # ["Best", "Best", "Best", "Best"]
                    # visits_or_tours =       ["visit", "visit"] # ["visit", "visit", "visit", "visit"]
                    # merging_strategies =    ["onboard", "gcs"] # ["onboard", "onboard", "onboard", "onboard"]



                    plot_labels = []
                    plot_colors = []
                    plot_linestyles = []
                    plot_markers = []
                    plot_markerfacecolors = []
                    for model_no, model in enumerate(models):
                        if model==MTSP:
                            label = "mTSP "
                        # else:
                        #     label = f"{model['Exp']} "
                            # label = "Multi-tour mTSP " if visits_or_tours[model_no]=="tour" else "Multi-visit mTSP "
                        else:
                            label = "Multi-tour " if visits_or_tours[model_no]=="tour" else ""
                            print(model)
                            label += f"{model['Exp']} "
                        label += "with merging " if merging_strategies[model_no] == "onboard" and model["Exp"]=="MTSP" else ""
                            
                        for objective_no, objective in enumerate(objectives[model_no]):
                            # label = "$" + model["Exp"] + "_" + "{" + model["Type"] + "," + visits_or_tours[model_no] + "," + merging_strategies[model_no] + "}" + "$" + " " + directions[model_no] + " " + objective_name_legend_dict[objective]
                            label_with_objective = label + f"{directions[model_no]} {objective_name_legend_dict[objective]} Path" if model["Type"]=="MOO" else label
                            plot_labels.append(label_with_objective)
                            plot_colors.append(color_dict[model["Exp"] + "_" + model["Type"]])
                            plot_linestyles.append(linestyles_dict[objective])
                            # plot_markers.append(markers_dict[visits_or_tours[model_no]])
                            plot_markers.append(markers_dict[visits_or_tours[model_no]])
                            plot_markerfacecolors.append(markerfacecolors_dict[merging_capabilities[model_no]])
                    
                            # plot_colors.append()
                            # plot_linestyles.append(linestyles[objective_no])
                            # plot_markers.append(markers[merging_strategies[model_no]])
                            # plot_markerfacecolors.append(markerfacecolors[visit_or_tour[model_no]])
                            
                    for run in range(n_runs):
                        y_ind = -1
                        target_locations = np.random.choice(range(1, 64), n_targets, replace=False)
                        # target_locations = np.append(np.random.choice(range(1, 63), n_targets-1, replace=False), 63)
                        for model_no, model in enumerate(models):
                            parameters = f"runs: {n_runs}, B: {round(B,2)}, p: {round(p,2)}, q: {round(q,2)}, T: {n_targets}, $r_c$: {comm_range}, " + "$n_{visits}$: " + str(m)
                            info.model = model
                            model_objectives = objectives[model_no]
                            direction = directions[model_no]
                            merging_capability = merging_capabilities[model_no]
                            merging_strategy = merging_strategies[model_no]
                            # parameters = f"dt: {merging_strategy}, merging: {merging_capability}, " + parameters
                            visit_or_tour = visits_or_tours[model_no]
                            for objective_no, objective in enumerate(model_objectives):
                                y_ind += 1
                                for number_of_drones_no, number_of_drones in enumerate(number_of_drones_list):
                                    info.number_of_drones = number_of_drones
                                    scenario = str(info) if visit_or_tour == "visit" else str(info).replace("visit", "tour")
                                    F = pd.read_pickle(f"{objective_values_filepath}{scenario}-ObjectiveValues.pkl")
                                    X = pd.read_pickle(f"{solutions_filepath}{scenario}-SolutionObjects.pkl")
                                    sol = X[F[objective].idxmin()] if direction=="Best" else X[get_median_index_of_scenario(scenario)]
                                    cfg = SensingConfig.from_info(sol.info, merge_topology="onboard",
                                                                  time_model=merging_strategy,
                                                                  target_locations=target_locations,
                                                                  detection_prob=p, false_alarm_prob=q, belief_threshold=B)
                                    if cfg.time_model == "discrete":
                                        time_metrics, updated_sol = sensing_and_discrete_info_sharing(sol=sol, config=cfg)
                                    else:
                                        time_metrics, updated_sol = sensing_and_realtime_info_sharing(sol=sol, config=cfg)
                                    # print(f"Run {run+1} Target Locations: {target_locations} Model: {model['Exp']} {objective_name_legend_dict[objective]} Path")
                                    y_detection_time[y_ind][number_of_drones_no] += time_metrics["detection time"] if time_metrics["detection time"] != np.inf else 0
                                    y_inform_time[y_ind][number_of_drones_no] += time_metrics["inform time"] if time_metrics["inform time"] != np.inf else 0
                                    # y_alo_time[y_ind][number_of_drones_no] += time_metrics["time at least one drone knows all targets"] if time_metrics["time at least one drone knows all targets"] != np.inf else 0
                                    y_mission_time[y_ind][number_of_drones_no] += time_metrics["mission time"] if time_metrics["mission time"] != np.inf else 0
                                    # y_successful_runs[y_ind][number_of_drones_no] += 1 if time_metrics["detection time"] != np.inf else 0
                                    mission_status = "Successful" if time_metrics["detection time"] != np.inf else "Failed"
                                    print( f"Run {run+1} Target Locations: {target_locations} Mission {mission_status}")
                                    # if mission_status=="Failed":
                                    #     print(f"nvisits: {m}")
                                    #     # missing_targets = []
                                    #     occ_status = time_metrics["occupancy status"][:,target_locations]
                                    #     missing_targets = target_locations[np.where(np.sum(occ_status, axis=0)==0)[0]]
                                    #     # Find missing targets
                                    #     all_target_occ_status = time_metrics
                                    #     for missing_target in missing_targets:
                                    #         missing_target_visits = np.where(sol.real_time_path_matrix==missing_target)[0]
                                    #         missing_target_occurance_frequency = len(missing_target_visits)
                                    #         print(f"Missing Target Frequency: {missing_target_occurance_frequency}, Occurances: {missing_target_visits}")
                                    #         print(f"Target Cell {missing_target} Search Map:\n{time_metrics['search map'][:,missing_target]}")

                            

                    # ys = [y_detection_time, y_inform_time, y_alo_time, y_mission_time, y_successful_runs]
                    ys = [y_detection_time, y_inform_time, y_mission_time]
                    for y in ys:
                        for dataset in y:
                            dataset/=n_runs
                    # Plot the results
                    for i in range(len(plot_colors)):
                        # print(">",i)
                        # print(">",plot_colors)
                        # print(labels[i])
                        detection_time_ax.plot(number_of_drones_list, y_detection_time[i],
                                               color=plot_colors[i],
                                               linewidth=linewidth, label=plot_labels[i], linestyle=plot_linestyles[i], marker=plot_markers[i], markerfacecolor=plot_markerfacecolors[i]
                                               )
                        
                        inform_time_ax.plot(number_of_drones_list, y_inform_time[i],
                                            color=plot_colors[i],
                                            linewidth=linewidth, label=plot_labels[i], linestyle=plot_linestyles[i], marker=plot_markers[i], markerfacecolor=plot_markerfacecolors[i]
                                            )
                        # time_at_least_one_drone_knows_all_targets_ax.plot(number_of_drones_list, y_alo_time[i],
                        #                                                   color=plot_colors[i],
                        #                                                   linewidth=linewidth, label=plot_labels[i], linestyle=plot_linestyles[i], marker=plot_markers[i], markerfacecolor=plot_markerfacecolors[i]
                        #                                                   )
                        mission_time_ax.plot(number_of_drones_list, y_mission_time[i],
                                             color=plot_colors[i],
                                             linewidth=linewidth, label=plot_labels[i], linestyle=plot_linestyles[i], marker=plot_markers[i], markerfacecolor=plot_markerfacecolors[i]
                                             )
                        # success_rate_ax.plot(number_of_drones_list, y_successful_runs[i],
                        #                      color=plot_colors[i],
                        #                      linewidth=linewidth, label=plot_labels[i], linestyle=plot_linestyles[i], marker=plot_markers[i], markerfacecolor=plot_markerfacecolors[i]
                        #                      )
                    
                    ### ADD RL RESULTS ###
                    rl_detection_time_values = []
                    rl_inform_time_values = []
                    rl_mission_time_values = []
                    # rl_success_rate_values = []
                    if comm_range == 2:
                        rl_results = rl_2.copy()
                    else:
                        rl_results = rl_sqrt8.copy()
                    n_targets_results = rl_results[f"RL {n_targets} Targets"]
                    for number_of_drones in number_of_drones_list:
                        n_targets_n_drones_results = n_targets_results[number_of_drones]
                        rl_detection_time_values.append(n_targets_n_drones_results["detection_time"])
                        rl_inform_time_values.append(n_targets_n_drones_results["informed_time"])
                        rl_mission_time_values.append(n_targets_n_drones_results["final_time_in_seconds"])
                        # rl_success_rate_values.append(n_targets_n_drones_results["success_rate"])

                    detection_time_ax.plot(number_of_drones_list, rl_detection_time_values, linewidth=linewidth, color="magenta", marker="D", markerfacecolor="none", label="RL")
                    inform_time_ax.plot(number_of_drones_list, rl_inform_time_values, linewidth=linewidth, color="magenta", marker="D", markerfacecolor="none", label="RL")
                    mission_time_ax.plot(number_of_drones_list, rl_mission_time_values, linewidth=linewidth, color="magenta", marker="D", markerfacecolor="none", label="RL")
                    # success_rate_ax.plot(number_of_drones_list, rl_success_rate_values, linewidth=linewidth, color="magenta", marker="D", markerfacecolor="none", label="RL")


                    # Add legends
                    for i in range(len(axes)):
                        axes[i].legend(loc='upper center', bbox_to_anchor=(0.5, -0.15), fontsize=big_fontsize, ncol=2, frameon=False)#(loc='upper center', bbox_to_anchor=(0.5, -0.15), fontsize=big_fontsize, ncol=2, frameon=False) # 0.5, -0.15
                        figs[i].subplots_adjust(top=0.95, bottom=0.335)  # play with these values top=0.90, bottom=0.335
                    # detection_time_ax.legend(ncol=2, loc='upper center', fontsize=10)
                    # inform_time_ax.legend(ncol=2, loc='upper center', fontsize=10)
                    # time_at_least_one_drone_knows_all_targets_ax.legend(ncol=2, loc='upper center', fontsize=10)
                    # mission_time_ax.legend(ncol=2, loc='upper center', fontsize=10)
                    # success_rate_ax.legend(ncol=2, loc='upper center', fontsize=10)

                    if show:
                        plt.show()

                    # Save figures if needed
                    if save:
                        for i, fig in enumerate(figs):
                            filename = parameters.replace(" ","").replace(":","_").replace(".","").replace(",","_").replace("$r_c$","rc").replace("$n_{visits}$","nvisits") + "_" + axes[i].get_ylabel().replace(" (s)", "").replace(" (%)", "").replace(" ", "_").lower()# .get_text().replace(" ","_").lower()
                            fig.savefig(f'Figures/{folder_name}/{filename}.png', dpi=300)
                            # Re-open with Pillow and save compressed
                            # img = Image.open(f'Figures/{folder_name}/{filename}.tif')
                            # img.save(f'Figures/{folder_name}/{filename}.tif', compression="tiff_lzw")
                            # fig.savefig(f"results/plots/{fig.get_title()}.png", dpi=300, bbox_inches='tight')

                        # fig_suffix = f"B{B}_p{p}_T{n_targets}_rc{comm_range}"
                        # detection_time_fig.savefig(f"results/plots/detection_time_{fig_suffix}.png", dpi=300, bbox_inches='tight')
                        # inform_time_fig.savefig(f"results/plots/inform_time_{fig_suffix}.png", dpi=300, bbox_inches='tight')
                        # time_at_least_one_drone_knows_all_targets_fig.savefig(f"results/plots/time_one_drone_knows_all_targets_{fig_suffix}.png", dpi=300, bbox_inches='tight')
                        # mission_time_fig.savefig(f"results/plots/mission_time_{fig_suffix}.png", dpi=300, bbox_inches='tight')
                        # success_rate_fig.savefig(f"results/plots/success_rate_{fig_suffix}.png", dpi=300, bbox_inches='tight')


plot_time_metrics_specific_models(
                                n_runs=5,
                                p0=0.5,
                                B_list=[0.9],
                                p_list=[0.8],
                                comm_range_list=[2],
                                number_of_drones_list=[4,8,12,16],
                                n_targets_list=[3,5],

                                models=[MTSP, TC_MOO_NSGA2, TCT_MOO_NSGA2],
                                visits_or_tours=["visit", "visit", "visit"],
                                merging_capabilities=["onboard", "onboard", "onboard"],
                                merging_strategies=["discrete", "discrete", "discrete"],
                                # objectives=[["Mission Time", "Max Mean TBV", "Percentage Connectivity", "Mean Disconnected Time", "Max Disconnected Time"]],
                                objectives = [["Mission Time"], ["Mission Time"], ["Mission Time", "Max Mean TBV"]],
                                directions=["Best", "Best", "Best"],

                                folder_name="Sensing RL Comparison",
                                title_on=False, show=True, save=False
                                )