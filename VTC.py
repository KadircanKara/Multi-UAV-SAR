import numpy as np
from collections import defaultdict
from math import inf, isnan
import os
from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.colors import TwoSlopeNorm
import seaborn as sns
from FilePaths import *
from PathFileManagement import load_pickle
from PathSolution import *
from PathInfo import *
from PathFuncDict import model_metric_info
from PathOptimizationModel import obj_unit_dict, obj_abbr_dict
import math
from Time import get_real_connectivity_matrix, get_real_paths

import matplotlib.patches as patches

def get_path_snapshot_at_step(model, objective, direction, number_of_drones, comm_cell_range, n_visits, add_transmission_range_and_highlight_target_cells, target_locations, n_targets, step, show, save):

    # Get the scenario string
    info = PathInfo()
    info.model = model
    info.number_of_drones = number_of_drones
    info.comm_cell_range = comm_cell_range
    info.n_visits = n_visits
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
    if target_locations is None:
        target_locations = np.random.choice(range(1, 64), n_targets, replace=False)

    # Draw paths and grid (copy from PathAnimation)

    fig, ax = plt.subplots()

    paths = get_real_paths(sol)  # np.array([x_matrix, y_matrix])
    real_time_x_matrix, real_time_y_matrix = paths
    real_time_connectivity_matrix = get_real_connectivity_matrix(real_time_x_matrix, real_time_y_matrix, sol)

    # Draw paths
    n_drones = real_time_x_matrix.shape[0]
    for i in range(n_drones):
        x, y = real_time_x_matrix[i][step], real_time_y_matrix[i][step]
        ax.plot(real_time_x_matrix[i], real_time_y_matrix[i], linewidth=1)
        # draw drone positions at step
        ax.plot(x, y, marker="o")
        # Add transmission range with circles
        # if i > 0:
        #     circle = patches.Circle((x, y), radius=info.comm_cell_range * info.cell_side_length, edgecolor='blue',
        #                 linestyle='dotted', fill=False)
        #     ax.add_patch(circle)
        # # Draw UAV paths
        
    
    for i in range(n_drones):
        for j in range(i+1, n_drones):
            if real_time_connectivity_matrix[step, i, j]:
                x_coords = [real_time_x_matrix[i, step], real_time_x_matrix[j, step]]
                y_coords = [real_time_y_matrix[i, step], real_time_y_matrix[j, step]]
                ax.plot(x_coords, y_coords, color='black', linewidth=3)

    if add_transmission_range_and_highlight_target_cells:

        for i in range(1, n_drones):
                circle = patches.Circle((x, y), radius=info.comm_cell_range * info.cell_side_length, edgecolor='blue',
                            linestyle='dotted', fill=False)
                ax.add_patch(circle)


        for cell in range(info.number_of_cells):
            cell_x, cell_y = PathSolution.get_coords(sol, cell)
            plt.annotate(cell, xy=(cell_x, cell_y), xytext=(0.0, 0.0), textcoords='offset points')
            if cell in target_locations:
                # plt.annotate(str(max_recent_prob), xy=(target_x, target_y), xytext=(0.0, 0.0), textcoords='offset points')
                # bottom_left_x, bottom_left_y = target_x - sol.info.cell_side_length/2, target_y - sol.info.cell_side_length/2
                bottom_left_x, bottom_left_y = cell_x - sol.info.cell_side_length/2, cell_y - sol.info.cell_side_length/2
                rect = patches.Rectangle((bottom_left_x, bottom_left_y),
                    width=sol.info.cell_side_length,
                    height=sol.info.cell_side_length,
                    linewidth=1,
                    edgecolor='none',         # no border
                    facecolor='red',       # fill color
                    alpha=0.3)                # transparency      
                ax.add_patch(rect)

    # Write Highest probability inside cell
    # ax.legend()
    # plt.title("Drone Paths")
    # plt.xlabel("X Coordinate")
    # plt.ylabel("Y Coordinate")
    

    x_ticks_values = [i for i in range(-sol.info.cell_side_length, 
                                        (sol.info.grid_size + 1) * sol.info.cell_side_length, 
                                        sol.info.cell_side_length)]
    y_tick_values = x_ticks_values.copy()
    x_ticks_labels = [i for i in range(-1, sol.info.grid_size + 1)]
    y_tick_labels = x_ticks_labels.copy()

    ax.set_xticks(x_ticks_values)
    ax.set_xticklabels(x_ticks_labels, fontsize=20)
    ax.set_yticks(y_tick_values)
    ax.set_yticklabels(y_tick_labels, fontsize=20)
    ax.grid(linestyle='--')

    if show:
        plt.show()
    # fig.savefig(f"Figures/Sensing/{scenario}_step{step}_snapshot.png", dpi=300, bbox_inches='tight')

    # Draw paths
    



def show_map_with_cell_ids():
    fig, ax = plt.subplots(figsize=(8, 8))
    ax.set_xlim(-1, 8)
    ax.set_ylim(-1, 8)
    ax.set_xticks([])
    ax.set_yticks([])

    def draw_cell(x, y, label):
        if (x >= 0 and y >= 0):
            ax.plot([x, x + 1], [y, y], color='gray')
            ax.plot([x + 1, x + 1], [y, y + 1], color='gray')
            ax.plot([x, x + 1], [y + 1, y + 1], color='gray')
            ax.plot([x, x], [y, y + 1], color='gray')
            ax.text(x + 0.5, y + 0.5, label, ha='center', va='center', fontsize=12)

    # Draw grid cells
    for y in range(8):
        for x in range(8):
            cell_id = x + y * 8
            draw_cell(x, y, str(cell_id))

    # Draw BS cell at (-1, -1)
    bs_x, bs_y = -1, -1
    ax.plot([bs_x, bs_x + 1], [bs_y, bs_y], color='black')
    ax.plot([bs_x + 1, bs_x + 1], [bs_y, bs_y + 1], color='black')
    ax.plot([bs_x, bs_x + 1], [bs_y + 1, bs_y + 1], color='black')
    ax.plot([bs_x, bs_x], [bs_y, bs_y + 1], color='black')
    ax.text(bs_x + 0.5, bs_y + 0.5, "BS", ha='center', va='center',
            fontsize=12, bbox=dict(boxstyle="round,pad=0.3", facecolor="lightgray", edgecolor="black"))

    # Main border lines from top-right of BS
    ax.plot([0, 0], [0, 8], color='black')
    ax.plot([0, 8], [0, 0], color='black')

    # Overwrite unwanted borders with white
    ax.plot([0, -1], [7, -1], color='white', linewidth=3)
    ax.plot([-1, 0], [-1, 7], color='white', linewidth=3)

    # Curly bracket and A next to cell 0 (left center)
    ax.text(-0.2, 0.5, '{', fontsize=25, rotation=0, ha='center', va='center')
    ax.text(-0.35, 0.5, 'A', fontsize=14, ha='center', va='center')

    # Bottom curly bracket and A under cell 0
    ax.text(0.5, -0.25, '}', fontsize=25, rotation=270, ha='center', va='top')
    ax.text(0.5, -0.55, "A", fontsize=14, ha='center', va='top')

    ax.set_aspect('equal')
    
    plt.show()

def get_objective_ranges(models, objectives, M_list, r_list, nvisit_list):
    info = PathInfo()
    for obj in objectives:
        # min_obj_vals, max_obj_vals = [], []
        for model in models:
            cum_min_obj, cum_max_obj = np.inf, -1
            info.model = model
            # print(f"Model: {model['Exp']}", end=" ")
            for m in M_list:
                info.number_of_drones = m
                for r in r_list:
                    info.comm_cell_range = r
                    for nvisit in nvisit_list:
                        info.n_visits = nvisit
                        scenario = str(info)
                        # print(f"Scenario: {scenario}", end=" ")
                        # print(info.comm_cell_range, scenario)
                        F = pd.read_pickle(f"{objective_values_filepath}{scenario}-ObjectiveValuesAbs.pkl")
                        X = load_pickle(f"{solutions_filepath}{scenario}-SolutionObjects.pkl")
                        if obj not in list(F.columns):
                            # Calculate objective values
                            F[obj] = [getattr(sol, obj_name_sol_attr_dict[obj]) for sol in X]
                        model_min_obj, model_max_obj = F[obj].min(), F[obj].max()
                        # print(f"Min: {model_min_obj}, Max: {model_max_obj}", end="\n")
                        # print(f"Pre-Range: ({cum_min_obj},{cum_max_obj})", end=" ")
                        # if model_min_obj < cum_min_obj:
                        #     cum_min_obj = model_min_obj
                        # if model_max_obj > cum_max_obj:
                        #     cum_max_obj = model_max_obj
                        cum_min_obj = model_min_obj if model_min_obj < cum_min_obj else cum_min_obj
                        cum_max_obj = model_max_obj if model_max_obj > cum_max_obj else cum_max_obj
                        # print(f"Post-Range: ({cum_min_obj},{cum_max_obj})", end="\n")
                        # print("-"*100)
                    
            print(f"{info.model['Exp']} {obj} Range: {(cum_min_obj, cum_max_obj)}")
            # print("x"*150)


def plot_pareto_fronts(models=[TCDT_MOO_NSGA2, TCD_MOO_NSGA2], M_list=[4,8,12,16], r_list=[2,"sqrt(8)"], nvisit_list=[1,2,3], fontsize=14, folder_name=None, show=True, save=False):

    big_fontsize=16
    medium_fontsize=14
    small_fontsize=12

    info = PathInfo() # Initialize PathInfo object

    for model in models:
        info.model = model
        for m in M_list:
            info.number_of_drones = m
            for r in r_list:
                info.comm_cell_range = r
                for nvisit in nvisit_list:
                    info.n_visits = nvisit
                    scenario = str(info)
                    # print(info.comm_cell_range, scenario)
                    F = pd.read_pickle(f"{objective_values_filepath}{scenario}-ObjectiveValuesAbs.pkl")

                    # Copy everything after F, info

                    obj_names = list(F.columns)
                    num_objs = len(obj_names)
                    if info.comm_cell_range != sqrt(8):
                        scenario_info = f"$M$: {info.number_of_drones}, $r_c$: {info.comm_cell_range} " + "$n_{" + "visits}$: " + f"{str(info.n_visits)} "
                    else:
                        scenario_info = f"$M$: {info.number_of_drones}, $r_c$: " + r"$2\sqrt{2}$ " + "$n_{" + "visits}$: " + f"{str(info.n_visits)} "
                    exp = info.model["Exp"]
                    title = scenario_info + exp

                    # Value Check
                    # print(f"Scenario: {scenario_info}")
                    # for obj in obj_names:
                    #     print(f"Objective: {obj} Range: {(F[obj].min(), F[obj].max())}")
                    # print("-" * 150)

                    if num_objs==1:
                        continue
                    # F = round(F,2)
                    # Update Perc Conn Values from decial to percentage values
                    if "Percentage Connectivity" in obj_names:
                        F["Percentage Connectivity"] = F["Percentage Connectivity"] * 100
                    for num_objs_to_plot in [2, 3]:  # Iterate over 2D and 3D cases
                        if len(obj_names) < num_objs_to_plot:
                            print(f"Skipping {num_objs_to_plot}D plot: Only {len(obj_names)} objectives available.")
                            continue  # Skip if not enough objectives
                    # Generate combinations of objectives
                        for obj_combination in itertools.combinations(obj_names, num_objs_to_plot):
                            if info.comm_cell_range != sqrt(8):
                                save_as = f"{exp}_M_{info.number_of_drones}_r_{info.comm_cell_range}_nvisits_{info.n_visits}_"
                            else:
                                save_as = f"{exp}_M_{info.number_of_drones}_r_sqrt(8)_nvisits_{info.n_visits}_"
                            if num_objs_to_plot == 2:
                                # 2D Pareto Front
                                obj1, obj2 = obj_combination
                                save_as += f"{obj1.replace(' ', '_')}_{obj2.replace(' ', '_')}_pf"
                                if info.n_visits==1 and "Max Mean TBV" in obj_combination:
                                    continue
                                fig, ax = plt.subplots(figsize=(8,6))
                                ax.grid()
                                ax.scatter(F[obj1], F[obj2])
                                # ax.set_title(f'{title} PF', fontsize=fontsize)
                                ax.set_xlabel(f"{obj_abbr_dict[obj1]} ({obj_unit_dict[obj1]})", fontsize=fontsize)
                                ax.set_ylabel(f"{obj_abbr_dict[obj2]} ({obj_unit_dict[obj2]})", fontsize=fontsize)
                                ax.set_xticklabels([round(x, 2) if x < 10 else round(x) for x in ax.get_xticks()], fontsize=fontsize)
                                ax.set_yticklabels([round(x, 2) if x < 10 else round(x) for x in ax.get_yticks()], fontsize=fontsize)

                            elif num_objs_to_plot == 3:
                                # 3D Pareto Front
                                obj1, obj2, obj3 = obj_combination
                                if info.n_visits==1 and "Max Mean TBV" in obj_combination:
                                    continue
                                save_as += f"{obj1.replace(' ', '_')}_{obj2.replace(' ', '_')}_{obj3.replace(' ', '_')}_pf"
                                fig = plt.figure()
                                ax = fig.add_subplot(111, projection='3d')
                                fig.set_size_inches(8, 6)
                                ax.grid()
                                ax.scatter(F[obj1], F[obj2], F[obj3])
                                # ax.set_title(f'{title} PF', fontsize=fontsize)
                                ax.set_xlabel(f"{obj_abbr_dict[obj1]} ({obj_unit_dict[obj1]})", fontsize=fontsize)
                                ax.set_ylabel(f"{obj_abbr_dict[obj2]} ({obj_unit_dict[obj2]})", fontsize=fontsize)
                                ax.set_zlabel(f"{obj_abbr_dict[obj3]} ({obj_unit_dict[obj3]})", fontsize=fontsize)
                                ax.set_xticklabels([round(x, 2) if x < 10 else round(x) for x in ax.get_xticks()], fontsize=fontsize)
                                ax.set_yticklabels([round(x, 2) if x < 10 else round(x) for x in ax.get_yticks()], fontsize=fontsize)
                                ax.set_zticklabels([round(x, 2) if x < 10 else round(x) for x in ax.get_zticks()], fontsize=fontsize)
                                # plt.show()

                            # print(save_as)

                            if show:
                                plt.show()
                            if save:
                                fig.savefig(f"Figures/Pareto Fronts/{save_as}.png")

                            # Reset fig, ax
                            fig, ax = None, None


def plot_best_objs_for_nvisits(models, objectives, r, n, v, folder_name=None, show=False, save=True):

    big_fontsize = 14

    if save:
        model_info = ""
        for model in models:
            model_info += f"{model['Exp']}_{model['Alg']}_"
        model_info = model_info[:-1]

    model_exps = [model["Exp"] for model in models]

    # assert len(models) <= 2, "Only two models can be compared"
    # if len(models) == 2:
    #     assert models[0]['Alg'] == models[1]['Alg'], "Algorithms must be the same"
    # if  not isinstance(models, list):
    #     models = [models]
    # if isinstance(r, float) or isinstance(r, int):
    #     r = [r]
    # if isinstance(n, float) or isinstance(n, int):
    #     n = [n]
    # if isinstance(v, float) or isinstance(v, int):
    #     n = [v]

    # Define linestyles for different models
    linewidth = 0.8
    linestyles = ['solid','--', ':','-.']
    markers = ['o', '>', 'x','D']
    linecolors = ['black', 'blue', 'red', 'green']
    markerfacecolors = ['none', 'none', 'none', 'none']  # Hollow and filled markers
    markeredgecolors = ['black', 'blue', 'red', 'green']  # Edge colors for markers

    unit_dict = {"Mission Time": "sec", "Percentage Connectivity": "%", "Max Mean TBV": "sec", "Max Disconnected Time":"timestep", "Mean Disconnected Time":r"$\frac{Isolated Timesteps}{Total Timesteps}$"}

    # nvisit 1 icin o, 2 icin >, 3 icin x marker

    # Get common objectives between models
    # if len(models) == 2:
    #     objective_names = [x for x in models[0]["F"] if x in models[1]["F"]]
    # else:
    #     objective_names = models[0]["F"]
    # if "Max Mean TBV" not in objective_names:
    #     objective_names.append("Max Mean TBV")
    
    # all_objective_filenames = os.listdir(objective_values_filepath)
    # all_solution_filenames = [x for x in os.listdir(solutions_filepath) if "SolutionObjects" in x]
    # best_time_solution_filenames = [x for x in os.listdir(solutions_filepath) if "Best-Mission_Time" in x]
    # best_tbv_solution_filenames = [x for x in os.listdir(solutions_filepath) if "Best-Max_Mean_TBV" in x]
    # print(np.array(best_tbv_solution_filenames))

    objective_names = []
    for model in models:
        for objective in model["F"]:
            if objective not in objective_names and "Weighted Sum" not in objective:
                objective_names.append(objective)

    # print(objective_names)

    info = PathInfo()

    for objective in objectives:
        print("Objective:", objective)
        for r_value in r:
            info.comm_cell_range = r_value
            # Create a figure and axis
            fig, ax = plt.subplots(figsize=(8, 6)) # 8, 6
            # plt.subplots_adjust(left=0.145)  # Increase the left margin (default is ~0.125)
            ax.set_xticks(n)
            ax.set_xticklabels(n, fontsize=big_fontsize)
            ax.grid()
            ax.set_xlabel('Number of Drones', fontsize=big_fontsize)
            if "Weighted Sum" in objective:
                ax.set_ylabel(f"{objective} (-)", fontsize=big_fontsize)
            else:
                ax.set_ylabel(f"{objective} ({obj_unit_dict[objective]})", fontsize=big_fontsize)
            
            # Modify r_value
            if r_value == sqrt(8):
                r_value = "sqrt(8)"
            for i, model in enumerate(models):
                info.model = model
                linestyle = linestyles[i]
                color = linecolors[i]
                # marker = markers[models.index(model)]
                markerfacecolor = markerfacecolors[i]
                markeredgecolor = markeredgecolors[i]
                # Skip 1 visit for TBV Plots
                for j, v_value in enumerate(v):
                    info.n_visits = v_value
                    if ( objective == "Max Mean TBV" or (model["Exp"]=="TCDT" and "TCD" in model_exps) ) and v_value == 1:
                        continue
                    marker = markers[j]
                    y = []
                    y_time_at_best_tbv = [] # if objective == "Mission Time" # and model==TCDT_MOO_NSGA2 else None
                    y_conn_at_best_tbv = [] # if objective == "Percentage Connectivity" # and model==TCDT_MOO_NSGA2 else None
                    y_tbv_at_best_time = [] # if objective == "Max Mean TBV" # and model==TCDT_MOO_NSGA2 else None
                    for n_value in n:
                        info.number_of_drones = n_value
                        # Get the solution filename
                        scenario = str(info)
                        objective_values = load_pickle(f"{objective_values_filepath}{scenario}-ObjectiveValues.pkl")
                        solution_objects = load_pickle(f"{solutions_filepath}{scenario}-SolutionObjects.pkl").flatten()
                        # print(objective_values)

                        for objective_name in objectives:
                            if objective_name not in list(objective_values.columns):
                                if "Weighted Sum" in objective:
                                    new_objective_values = [calculate_ws_score_from_ws_objective(sol) for sol in solution_objects]
                                else:
                                    new_objective_values = [getattr(sol, obj_name_sol_attr_dict[objective]) for sol in solution_objects]
                                objective_values[objective_name] = new_objective_values


                        # best_tbv_solution = solution_objects[objective_values["Max Mean TBV"].idxmin()] # if (objective == "Mission Time" or objective == "Percentage Connectivity") and model==TCDT_MOO_NSGA2 else None
                        # best_time_solution = solution_objects[objective_values["Mission Time"].idxmin()] # if (objective == "Mission Time" or objective == "Percentage Connectivity") and model==TCDT_MOO_NSGA2 else None
                        # print(objective_values)
                        best_objective_value = abs(objective_values[objective].min())
                        print(f"{scenario} best {objective} value: {best_objective_value}")
                        # if "Weighted Sum" not in objective:
                        #     best_objective_value *= model_metric_info["Objectives"][objective][1]
                        # best_objective_value = objective_values[objective].min()*model_metric_info["Objectives"][objective][1]

                        # time_at_best_tbv = getattr(best_tbv_solution, obj_name_sol_attr_dict["Mission Time"]) # if objective == "Mission Time" else None
                        # conn_at_best_tbv = getattr(best_tbv_solution, obj_name_sol_attr_dict["Percentage Connectivity"])*100 # if objective == "Percentage Connectivity" else None
                        # tbv_at_best_time = getattr(best_time_solution, obj_name_sol_attr_dict["Max Mean TBV"]) # if objective == "Max Mean TBV" else None

                        # objective_values = pd.read_pickle(f"{objective_values_filepath}{objective_filename}")[objective]
                        # best_objective_value = min(objective_values) if objective != "Percentage Connectivity" else max(objective_values)*100
                        # time_at_best_tbv = get_attribute(load_pickle(f"{solutions_filepath}{best_tbv_solution_filename}"), "Mission Time") if objective == "Mission Time" and model==TCDT_MOO_NSGA2 else None
                        # conn_at_best_tbv = get_attribute(load_pickle(f"{solutions_filepath}{best_tbv_solution_filename}"), "Percentage Connectivity")*100 if objective == "Percentage Connectivity" and model==TCDT_MOO_NSGA2 else None
                        # tbv_at_best_time = get_attribute(load_pickle(f"{solutions_filepath}{best_time_solution_filename}"), "Max Mean TBV") if objective == "Max Mean TBV" and model==TCDT_MOO_NSGA2 else None
                        y.append(best_objective_value)
                        # y_time_at_best_tbv.append(time_at_best_tbv) # if objective == "Mission Time" and model==TCDT_MOO_NSGA2 else None
                        # y_conn_at_best_tbv.append(conn_at_best_tbv) # if objective == "Percentage Connectivity" and model==TCDT_MOO_NSGA2 else None
                        # y_tbv_at_best_time.append(tbv_at_best_time) # if objective == "Max Mean TBV" and model==TCDT_MOO_NSGA2 else None
                    # if objective=="Mission Time" and n_value==16 and v_value==2 and r_value==4:
                    #     print(objective_filename, objective_values, best_objective_value)
                    # print(f"Debug Counter: {debug_counter}")
                    # model_underscript_alg_superscript_minv = "$" + model["Exp"] + "_" + "{" + model["Alg"] + "}^{" + str(v_value) + "}" + "$"
                    model_underscript_minv = rf"${model['Exp']}_{str(v_value)}$"
                    # model_underscript_minv = "$" + f'{model["Exp"]}_{str(v_value)}' + "}" + "$"
                    # print(n, y)
                    ax.plot(n, y, linestyle=linestyle, color=color, linewidth=linewidth, marker=marker, markerfacecolor=markerfacecolor, markeredgecolor=markeredgecolor, label=rf'{model_underscript_minv}')
                    # ax.plot(n, y_time_at_best_tbv, linestyle='dashdot', color='red', linewidth=2, marker=marker, markerfacecolor=markerfacecolor, markeredgecolor='red', label=rf'{model_underscript_minv} - Time at Best TBV') if y_time_at_best_tbv is not None else None
                    # ax.plot(n, y_conn_at_best_tbv, linestyle='dashdot', color='red', linewidth=2, marker=marker, markerfacecolor=markerfacecolor, markeredgecolor='red', label=rf'{model_underscript_minv} - Conn at Best TBV') if y_conn_at_best_tbv is not None else None
                    # ax.plot(n, y_tbv_at_best_time, linestyle='dashdot', color='red', linewidth=2, marker=marker, markerfacecolor=markerfacecolor, markeredgecolor='red', label=rf'{model_underscript_minv} - TBV at Best Time') if y_tbv_at_best_time is not None else None
                    # if model["Exp"] == "TCD":
                    #     ax.plot(n, y, linestyle=linestyle, color=color, linewidth=2, marker=marker, markerfacecolor=markerfacecolor, markeredgecolor=markeredgecolor, label=f'TCD - {v_value} Visit(s)')
                    # else:
                    #     ax.plot(n, y, linestyle=linestyle, color=color, linewidth=2, marker=marker, markerfacecolor=markerfacecolor, markeredgecolor=markeredgecolor, label=f'TCDT - {v_value} Visit(s)')

            # Set the title
            # if objective == "Max Mean TBV as Objective":
            #     ax.set_title(f'Best Max Mean TBV Values for {r_value} Cell(s) Communication Range', pad=60, fontsize=16)
            # else:
            #     ax.set_title(f'Best {objective} Values for {r_value} Cell(s) Communication Range', pad=60, fontsize=16)
            # ax.set_title(f'Best {objective} Values for {r_value} Cell(s) Communication Range', pad=70, fontsize=14)
            # ax.set_yticklabels([round(x) for x in ax.get_yticks()], fontsize=16)
            ax.set_yticklabels(ax.get_yticklabels(), fontsize=big_fontsize)
            # Adjust the plot area to make space for the legend
            fig.subplots_adjust(top=0.95, bottom=0.335) # 0.8
            # Add a legend to the plot
            # ax.legend(ncol=3, loc="upper center", bbox_to_anchor=(0.5, 1.24),  fontsize=16) # 1.185
            ax.legend(ncol=4, loc="upper center", bbox_to_anchor=(0.5, -0.15), fontsize=big_fontsize, frameon=False) # 0.470, 1.275
            # ax.legend(ncol=3, loc="upper center")
            # Annotate
            # for j in range(len(n)):
            #     ax.annotate(f'{round(y[j], 2)}', (n[j], n[j]), textcoords="offset points", xytext=(0,5), ha='center')
            # Save plot
            if save:
                model_names = ""
                for model in models:
                    model_names += model["Exp"] + "_"
                model_names = model_names[:-1]
                # print(model['Alg'])
                if folder_name is not None:
                    fig.savefig(f"Figures/Objective Values/{folder_name}/{model_names}_r_{r_value}_{objective.replace(' ', '_')}_best_values.png", bbox_inches='tight', dpi=300)
                else:
                    fig.savefig(f"Figures/Objective Values/{model_names}_r_{r_value}_{objective.replace(' ', '_')}_best_values.png", bbox_inches='tight', dpi=300)
            # Show plot
            if show:
                plt.show()