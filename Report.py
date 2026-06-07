import numpy as np
from math import inf
import os
from matplotlib import pyplot as plt
from matplotlib.colors import TwoSlopeNorm
import seaborn as sns
from FilePaths import *
from PathFileManagement import load_pickle
from PathSolution import *
from PathInfo import *
from PathFuncDict import model_metric_info
import sympy as sp

def compare_objs(models=[MTSP], objectives=["Percentage Connectivity"], number_of_drones_values = [4,8,12,16], comm_cell_range__values=[2,"sqrt(8)"], nvisit_values=[1,2,3], add_ntour=True, show=True, save=False):
    """Compare obj values of models"""
    fontsize=16
    colors = ["black", "blue", "red"]
    linestyles = ['solid', 'dashed','dotted', "dashdot"]
    markers = ['o', '>', 'x', 'D']
    visit_or_tour = ["visit","tour"]
    info = PathInfo() # Initialize PathInfo object
    # Create figure for each objective & comm range combo
    for objective_name in objectives:
        for comm_cell_range in comm_cell_range__values:
            info.comm_cell_range = comm_cell_range # Update PathInfo object
            for direction in ["Median", "Best", "Mean"]:
                fig, ax = plt.subplots(figsize=(8, 6))
                ax.grid()
                ax.set_title(f"{direction} {objective_name} Comparison for {comm_cell_range} Range", fontsize=fontsize)
                ax.set_xlabel("Number of Drones", fontsize=fontsize)
                ax.set_ylabel(f"{objective_name} ({model_metric_info['Objectives'][objective_name][-1]})", fontsize=fontsize)
                ax.set_xticks(number_of_drones_values)
                for i, model in enumerate(models):
                    info.model = model
                    color = colors[i]
                    for j, nvisit in enumerate(nvisit_values):
                        info.n_visits = nvisit
                        marker = markers[j]
                        y = [np.zeros(len(number_of_drones_values))]
                        if add_ntour and nvisit>1:
                            y.append(np.zeros(len(number_of_drones_values)))
                        for k, number_of_drones in enumerate(number_of_drones_values):
                            info.number_of_drones = number_of_drones
                            nvisit_scenario = str(info)
                            ntour_scenario = nvisit_scenario.replace("nvisits", "ntours") if add_ntour and nvisit>1 else None
                            # Load Objective Values
                            F_nvisit = pd.read_pickle(f"{objective_values_filepath}{nvisit_scenario}-ObjectiveValues.pkl")
                            model_objectives = list(F_nvisit.columns)
                            if objective_name not in model_objectives:
                                # Calculate objective values from solution objects
                                X_nvisit = load_pickle(f"{solutions_filepath}{nvisit_scenario}-SolutionObjects.pkl")
                                X_ntour = load_pickle(f"{solutions_filepath}{ntour_scenario}-SolutionObjects.pkl") if add_ntour and nvisit>1 else None
                                F_nvisit = np.array([getattr(x, obj_name_sol_attr_dict[objective_name]) for x in X_nvisit])
                                F_ntour = np.array([getattr(x, obj_name_sol_attr_dict[objective_name]) for x in X_ntour]) if add_ntour and nvisit>1 else None
                            else:
                                F_nvisit = F_nvisit[objective_name]
                                F_ntour = F_ntour[objective_name] if add_ntour and nvisit>1 else None
                            # F_ntour = pd.read_pickle(f"{objective_values_filepath}{ntour_scenario}") if add_ntour else None
                            if direction == "Median":
                                if model["Type"]=="MOO":
                                    nvisit_obj_value = F_nvisit[get_median_index_of_scenario(nvisit_scenario)] * model_metric_info["Objectives"][objective_name][1]
                                    ntour_obj_value = F_ntour[get_median_index_of_scenario(ntour_scenario)] * model_metric_info["Objectives"][objective_name][1] if add_ntour and nvisit>1 else None
                                else:
                                    nvisit_obj_value = F_nvisit[0] * model_metric_info["Objectives"][objective_name][1]
                                    ntour_obj_value = F_ntour[0] * model_metric_info["Objectives"][objective_name][1] if add_ntour and nvisit>1 else None
                            elif direction == "Best":
                                nvisit_obj_value = F_nvisit.min() * model_metric_info["Objectives"][objective_name][1]
                                ntour_obj_value = F_ntour.min() * model_metric_info["Objectives"][objective_name][1] if add_ntour and nvisit>1 else None
                            elif direction == "Mean":
                                nvisit_obj_value = F_nvisit.mean() * model_metric_info["Objectives"][objective_name][1]
                                ntour_obj_value = F_ntour.mean() * model_metric_info["Objectives"][objective_name][1] if add_ntour and nvisit>1 else None

                            value_sets = [nvisit_obj_value]
                            if add_ntour and nvisit>1:
                                value_sets.append(ntour_obj_value)

                            for l, value_set in enumerate(value_sets):
                                y[l][k] = value_set

                        # Plot data
                        for t, value_set in enumerate(y):
                            linestyle = linestyles[t]
                            ax.plot(number_of_drones_values, value_set, label=f"{model['Exp']}_{nvisit}_{visit_or_tour[t]}", marker=marker, color=color, linewidth=1, linestyle=linestyle)
                ax.legend()
                if show:
                    plt.show()                    

def format_percentage(value):
    return f"%{value:.2f}"  # Add '%' before each value

def get_visit_times(sol):
    info = sol.info
    drone_path_matrix = sol.real_time_path_matrix[1:,:]
    visit_times = [[] for _ in range(info.number_of_cells)]
    for cell in range(info.number_of_cells):
        visit_times[cell] = np.sort(np.where(drone_path_matrix==cell)[1])[:info.n_visits] # Last bit is to exclude hovering steps

    sol.visit_times = visit_times

    return visit_times

def calculate_tbv(sol):
    get_visit_times(sol)
    tbv = [np.diff(x) for x in sol.visit_times]
    sol.tbv = tbv

    return tbv

def calculate_mean_tbv(sol):
    calculate_tbv(sol)
    mean_tbv = list(map(lambda x: np.mean(x), sol.tbv))
    sol.mean_tbv = mean_tbv
    sol.max_mean_tbv = max(sol.mean_tbv)
    return sol.mean_tbv

def calculate_max_mean_tbv(sol):
    calculate_mean_tbv(sol)
    return sol.max_mean_tbv

def calculate_runtimes(model):
    """Calculate runtime for a specific model. Calculate for each number of drones, number of visits combination, 
    take the average runtime of all com_range values"""
    info = PathInfo()
    info.model = model
    for number_of_drones in [4,8,12,16]:
        info.number_of_drones = number_of_drones
        runtimes = []
        for n_visits in [1,2,3]:
            info.n_visits = n_visits
            for comm_cell_range in [2,2*sqrt(2),4]:
                info.comm_cell_range = comm_cell_range
                scenario_text = str(info)
                scenario = f"{scenario_text}"
                runtime = pd.read_pickle(f"{runtimes_filepath}{scenario}-Runtime.pkl")
                runtimes.append(runtime)
        avg_runtime = np.mean(runtimes)/60 # In minutes
        print(f"Average runtime for {number_of_drones} drones: {avg_runtime}")

def get_attribute(obj, attribute_name):
    # Map input names to actual attribute names
    attribute_mapping = {
        "Mission Time": "mission_time",
        "Percentage Connectivity": "percentage_connectivity",
        "Max Mean TBV": "max_mean_tbv",
        "Mean Disconnected Time": "mean_disconnected_time",
        "Max Disconnected Time": "max_disconnected_time",
        # Add more mappings as needed
    }
    # Check if the input attribute exists in the mapping
    if attribute_name in attribute_mapping:
        # Retrieve the actual attribute value using getattr
        return getattr(obj, attribute_mapping[attribute_name])
    else:
        raise ValueError(f"Attribute '{attribute_name}' not found.")
    
def get_attr(obj:str):
    if obj == "Mission Time":
        return PathSolution.misson_time
    elif obj == "Percentage Connectivity":
        return lambda x: x.percentage_connectivity
    elif obj == "Max Mean TBV":
        return lambda x: x.max_mean_tbv
    elif obj == "Mean Disconnected Time":
        return lambda x: x.mean_disconnected_time
    elif obj == "Max Disconnected Time":
        return lambda x: x.max_disconnected_time
    else:
        raise ValueError(f"Objective {obj} is not valid")

def get_attribute(sol:PathSolution, obj:str):
    if obj == "Mission Time":
        return sol.mission_time
    elif obj == "Percentage Connectivity":
        return sol.percentage_connectivity
    elif obj == "Max Mean TBV":
        return sol.max_mean_tbv
    elif obj == "Mean Disconnected Time":
        return sol.mean_disconnected_time
    elif obj == "Max Disconnected Time":
        return sol.max_disconnected_time
    else:
        raise ValueError(f"Objective {obj} is not valid")
    
# ------------------------------------------------------------------ PLOT FUNCTIONS ------------------------------------------------------------------

def plot_time_tbv_gain(models=[TCDT_MOO_NSGA2, TCD_MOO_NSGA2], numbers_of_drones = [4,8,12,16], comm_ranges = [2,2*sqrt(2),4], numbers_of_visits=[1,2,3], show=True, save=False):
    info = PathInfo()
    fontsize = 16
    linestyles = ['--','solid']
    markers = ['o', '>', 'x']
    colors = ["black", "blue"]
    objectives = ["Mission Time", "Percentage Connectivity", "Max Mean TBV"]
    for objective in objectives:
        for comm_range in comm_ranges:
            info.comm_cell_range = comm_range
            str_comm_range = "sqrt(8)" if comm_range == 2*sqrt(2) else comm_range
            fig, ax = plt.subplots(figsize=(8, 6))
            for number_of_visits in numbers_of_visits:
                info.n_visits = number_of_visits
                marker = markers[numbers_of_visits.index(number_of_visits)]
                model_objectives = []
                objective_gain_between_models = []
                time_gain = [] if objective=="Mission Time" else None
                conn_gain = [] if objective=="Percentage Connectivity" else None
                tbv_gain = [] if objective=="Max Mean TBV" else None
                for model in models:
                    info.model = model
                    scenario = str(info)
                    try:
                        best_objective_value = get_attribute(load_pickle(f"{solutions_filepath}{scenario}-Best-{objective.replace(' ','_')}-Solution.pkl"), objective)
                    except:
                        all_solutions = load_pickle(f"{solutions_filepath}{scenario}-SolutionObjects.pkl")
                        best_objective_value = min(list(map(lambda x: get_attribute(x, objective), all_solutions)))
                    # For objective_gain_between_models
                    model_objectives.append(best_objective_value) # For the gain plot
                    # For Objective Comparison Between Best Other Objective Solutions
                    if model == TCDT_MOO_NSGA2:
                        # TODO: Get Best Time and TBV Solutions
                        best_time_sol = load_pickle(f"{solutions_filepath}{scenario}-Best-Mission_Time-Solution.pkl")
                        best_tbv_sol = load_pickle(f"{solutions_filepath}{scenario}-Best-Max_Mean_TBV-Solution.pkl")
                        # For Time Gain
                        if objective=="Mission Time":
                            a=1

            
def plot_mission_time_percentage_connectivity_for_best_tbv(model, numbers_of_drones, comm_ranges, numbers_of_visits, show=True, save=False):
    fontsize = 16
    linestyles = ['--','solid']
    markers = ['o', '>', 'x']
    colors = ["black", "blue"]
    info = PathInfo()
    info.model = model
    for comm_range in comm_ranges:
        comm_range_str = "sqrt(8)" if comm_range == 2*sqrt(2) else comm_range
        # Initialize figure
        fig, ax = plt.subplots(figsize=(8, 6))
        info.comm_cell_range = comm_range
        for number_of_visits in numbers_of_visits:
            marker = markers[numbers_of_visits.index(number_of_visits)]
            time_y = []
            conn_y = []
            info.n_visits = number_of_visits
            for number_of_drones in numbers_of_drones:
                info.number_of_drones = number_of_drones
                scenario = str(info)
                sol = load_pickle(f"{solutions_filepath}{scenario}-Best-Max_Mean_TBV-Solution.pkl")
                time_y.append(sol.mission_time)
                print(sol.percentage_connectivity)
                conn_y.append(sol.percentage_connectivity*100) # Convert to % format
            time_label = "$" + model["Exp"] + "_" + "{time}" + "^" + "{" + str(number_of_visits) + "}" + "$"
            conn_label = "$" + model["Exp"] + "_" + "{conn}" + "^" + "{" + str(number_of_visits) + "}" + "$"
            ax.plot(numbers_of_drones, conn_y, label=conn_label, marker=marker, color="black", linewidth=2, linestyle="--")
            ax.plot(numbers_of_drones, time_y, label=time_label, marker=marker, color="blue", linewidth=2, linestyle="solid")
        # Add title, labels, legend, and grid
        ax.grid()
        ax.set_title(f"Time and Conn Values for {comm_range_str} Range Best TBV Solution", fontsize=fontsize)
        ax.set_xticks(numbers_of_drones)
        ax.set_xticklabels(numbers_of_drones, fontsize=fontsize)
        default_yticks = ax.get_yticks()
        int_yticks = [round(x) for x in default_yticks]
        min_ytick, max_ytick = min(int_yticks), max(int_yticks)
        # ax.set_yticks(np.arange(0, max_ytick, 50))
        ax.set_yticklabels(int_yticks, fontsize=fontsize)
        fig.subplots_adjust(top=0.8) # 0.8
        ax.legend(ncol=3, loc="upper center", bbox_to_anchor=(0.5, 1.325),  fontsize=fontsize)
        # ax.legend(fontsize=16)
        ax.set_xlabel("Number of Drones", fontsize=fontsize)
        ax.set_ylabel("Time (sec) & Connectivity (%) Values", fontsize=fontsize)
        if show:
            plt.show()
        if save:
            fig.savefig(f"Figures/Objective Values/Time Conn at Best TBV/{model['Alg']}_{model['Exp']}_r_{comm_range_str}_time_conn_at_best_tbv.png")


def plot_conn_tbv_for_best_mission_time(model, numbers_of_drones, comm_ranges, numbers_of_visits, show=True, save=False):
    # TODO: Get tbv best mission time solutions
    # print('MOO_NSGA2_time_conn_disconn_tbv_g_8_a_50_n_4_v_2.5_r_4_minv_2_maxv_5_Nt_1_tarPos_12_ptdet_0.99_pfdet_0.01_detTh_0.9_maxIso_0-Best-Mission_Time-Solution.pkl' == "MOO_NSGA2_time_conn_disconn_tbv_g_8_a_50_n_4_v_2.5_r_4_minv_2_maxv_5_Nt_1_tarPos_12_ptdet_0.99_pfdet_0.01_detTh_0.9_maxIso_0-Best-Mission_Time-Solution.pkl")
    fontsize = 16
    linestyles = ['--','solid']
    markers = ['o', '>', 'x']
    colors = ["black", "blue"]
    info = PathInfo()
    info.model = model
    for comm_range in comm_ranges:
        comm_range_str = "sqrt(8)" if comm_range == 2*sqrt(2) else comm_range
        # Initialize figure
        fig, ax = plt.subplots(figsize=(8, 6))
        info.comm_cell_range = comm_range
        for number_of_visits in numbers_of_visits:
            marker = markers[numbers_of_visits.index(number_of_visits)]
            conn_y = []
            tbv_y = []
            info.n_visits = number_of_visits
            for number_of_drones in numbers_of_drones:
                info.number_of_drones = number_of_drones
                scenario = str(info)
                sol = load_pickle(f"{solutions_filepath}{scenario}-Best-Mission_Time-Solution.pkl")
                conn_y.append(sol.percentage_connectivity*100) # Convert to % format
                tbv_y.append(sol.max_mean_tbv)
            conn_label = "$" + model["Exp"] + "_" + "{CONN}" + "^" + "{" + str(number_of_visits) + "}" + "$"
            tbv_label = "$" + model["Exp"] + "_" + "{TBV}" + "^" + "{" + str(number_of_visits) + "}" + "$"
            ax.plot(numbers_of_drones, conn_y, label=conn_label, marker=marker, color="black", linewidth=2, linestyle="--")
            ax.plot(numbers_of_drones, tbv_y, label=tbv_label, marker=marker, color="blue", linewidth=2, linestyle="solid")
        # Add title, labels, legend, and grid
        ax.grid()
        ax.set_title(f"Conn and TBV Values for {comm_range_str} Range Best Mission Time Solution", fontsize=fontsize)
        ax.set_xticks(numbers_of_drones)
        ax.set_xticklabels(numbers_of_drones, fontsize=fontsize)
        default_yticks = ax.get_yticks()
        int_yticks = [round(x) for x in default_yticks]
        ax.set_yticklabels(int_yticks, fontsize=fontsize)
        fig.subplots_adjust(top=0.8) # 0.8
        ax.legend(ncol=3, loc="upper center", bbox_to_anchor=(0.5, 1.325),  fontsize=fontsize)
        # ax.legend(fontsize=16)
        ax.set_xlabel("Number of Drones", fontsize=fontsize)
        ax.set_ylabel("Percentage Connectivity (%) & Max Mean TBV (sec)", fontsize=fontsize)
        if show:
            plt.show()
        if save:
            fig.savefig(f"Figures/Objective Values/Conn TBV at Best Time/{model['Alg']}_{model['Exp']}_r_{comm_range_str}_conn_tbv_at_best_mission_time.png")


def plot_objective_values(models, objectives=["Mission Time","Percentage Connectivity","Max Mean TBV","Mean Disconnected Time"], number_of_drones_values=[4,8,12,16], comm_cell_range_values=[2,2*sqrt(2),4], minv_values=[1,2,3], put_model_data_on_same_plot=True, show=True, save=False):

    

    # ASSERTIONS

    if not isinstance(models, list):
        models = [models]

    assert len(models) <= 2, "Up to two models can be compared"
    if len(models) == 2:
        assert(models[0]['Alg'] == models[1]['Alg']), "Algorithms must be the same"

    # OBJECTIVE VALUE - UNIT MAPPING
    objective_units = {
        "Mission Time": "sec",
        "Percentage Connectivity": "%",
        "Max Mean TBV": "sec",
        "Mean Disconnected Time": "timestep",
        "Max Disconnected Time": "timestep"
    }

    # PLOT CUSTOMIZATION
    linestyles = ['--','solid'] # For different models
    colors = ["black" , "blue"] # For dfferent models
    markers = ["o" , "x" , "^"] # For different minv values

    # Intialize model_plots if you want to put all model data on the same plot with the same values for objective name and comm__range
    model_plots = []

    combine_counter = -1
    combine_flag = len(objectives) * len(comm_cell_range_values) - 1

    test_counter = -1

    axes = []
    
    for i , model in enumerate(models):
        model_plots.append([])
        color = colors[i]
        linestyle = linestyles[i]
        for j , objective_name in enumerate(objectives):
            for k , comm_cell_range in enumerate(comm_cell_range_values):
                comm_cell_range = "sqrt(8)" if comm_cell_range == 2*sqrt(2) else comm_cell_range
                # Create figure and axis
                fig, ax = plt.subplots()
                ax.grid()
                title = f"Best {objective_name} Values for Transmission Range: " + "$\sqrt{8}$ Cells" if comm_cell_range == "sqrt(8)" else f"Best {objective_name} Values for Transmission Range: {comm_cell_range} Cells"
                ax.set_title(title, fontsize=12)
                ax.set_xlabel("Number of Drones")
                ax.set_ylabel(f"{objective_name} ({objective_units[objective_name]})")
                ax.set_xticks(number_of_drones_values)
                for l , minv in enumerate(minv_values):
                    marker = markers[l]
                    # Initialize y at every minv iteration
                    y = []
                    for m , number_of_drones in enumerate(number_of_drones_values):
                        # Get scenario
                        scenario = f"{model['Type']}_{model['Alg']}_{model['Exp']}_g_8_a_50_n_{number_of_drones}_v_2.5_r_{comm_cell_range}_minv_{minv}_maxv_5_Nt_1_tarPos_12_ptdet_0.99_pfdet_0.01_detTh_0.9_maxIso_0"
                        # Get the objective values
                        objective_values = pd.read_pickle(f"{objective_values_filepath}{scenario}-ObjectiveValues.pkl")
                        if objective_name in objective_values.columns:
                            best_objective_solution = pd.read_pickle(f"{solutions_filepath}{scenario}-Best-{objective_name.replace(' ','_')}-Solution.pkl")
                            best_objective_value = get_attribute(best_objective_solution, objective_name)*100 if objective_name == "Percentage Connectivity" else get_attribute(best_objective_solution, objective_name)
                        else:
                            solution_objects = pd.read_pickle(f"{solutions_filepath}{scenario}-SolutionObjects.pkl")
                            # First flatten solution_objects 2D array to get a list of PathSolution objects
                            solution_objects = solution_objects.flatten()
                            # Use lambda function to get the required objective value for each PathSolution object
                            objective_values = np.array(list(map(lambda x: get_attribute(x, objective_name), solution_objects)))
                            best_objective_value = objective_values.max()*100 if objective_name == "Percentage Connectivity" else objective_values.min()
                        # Best objective value for the scenario has been found, now append it to y values
                        y.append(best_objective_value)
                    # Plot y vs number_of_drones lineplot after all best objective values for each number_of_drones value have been appended to y
                    model_underscript_alg_comma_minv = "$" + model["Exp"] + "_" + "{" + model["Alg"] + "," + str(minv) + "}" + "$"
                    model_underscript_alg_superscript_minv = "$" + model["Exp"] + "_" + "{" + model["Alg"] + "}^{" + str(minv) + "}" + "$"
                    label = "$" + model["Exp"] + "_" + "{" + model["Alg"] + "," + str(minv) + "}" + "$"
                    # Add objective value vs. number of drones lineplot to the figure for current (objective_name, comm_cell_range, minv) combination
                    ax.plot(number_of_drones_values, y, linestyle=linestyle, linewidth=2, color=color, marker=marker,  label=rf"{model_underscript_alg_superscript_minv}")
                    model_plots[-1].append((fig, ax))
                    axes.append(ax)
                    plt.close()


    if put_model_data_on_same_plot:
        num_plots = len(objectives) * len(comm_cell_range_values)
        for i in range(num_plots):
            fig_combined, ax_combined = plt.subplots()
            axes_to_combine = []
            for j in range(len(models)):
                axes_to_combine.append(model_plots[j][i][1])
            print(axes_to_combine, len(axes_to_combine))
            for ax in axes_to_combine:
                lines = ax.get_lines()
                for line in lines:
                    ax_combined.plot(line.get_xdata(), line.get_ydata(), linestyle=line.get_linestyle(), linewidth=line.get_linewidth(), color=line.get_color(), marker=line.get_marker(), label=line.get_label())
                # Add title and legend
                ax_combined.set_title(ax.get_title())
                ax_combined.grid()
                ax_combined.set_xlabel(ax.get_xlabel())
                ax_combined.set_ylabel(ax.get_ylabel())
                ax_combined.legend()

    if show:
        plt.show()


def plot_pareto_fronts(show=True, save=False):
    fontsize=12
    """Plot the pareto fronts of the models"""
    all_objective_filenames = np.array([x for x in os.listdir(objective_values_filepath) if "ObjectiveValuesAbs" in x])
    figures = []
    for filename in all_objective_filenames:
        # Get plot title
        title = filename.split("-")[0]
        objective_values = pd.read_pickle(f"{objective_values_filepath}{filename}")

        if "minv_1" not in title:
            continue
            # Draw 3d pareto front
            fig = plt.figure()
            ax = fig.add_subplot(111, projection='3d')
            ax.scatter(objective_values["Mission Time"], objective_values["Percentage Connectivity"]*100, objective_values["Max Mean TBV"])
            # ax.set_title(f'{title} Pareto Front', fontsize="small")
            ax.set_xlabel("Mission Time (sec)", fontsize=fontsize)
            ax.set_ylabel("Percentage Connectivity (%)", fontsize=fontsize)
            ax.set_zlabel("Max Mean TBV (sec)", fontsize=fontsize)
            ax.set_xticklabels([round(x) for x in ax.get_xticks()], fontsize=fontsize)
            ax.set_yticklabels([round(x) for x in ax.get_yticks()], fontsize=fontsize)
            ax.set_zticklabels([round(x) for x in ax.get_zticks()], fontsize=fontsize)
        else:
            fig, ax = plt.subplots()
            ax.grid()
            ax.scatter(objective_values["Mission Time"], objective_values["Percentage Connectivity"]*100)
            ax.set_title(f'{title} Pareto Front', fontsize="small")
            ax.set_xlabel("Mission Time (sec)", fontsize=fontsize)
            ax.set_ylabel("Percentage Connectivity (%)", fontsize=fontsize)
            ax.set_xticklabels([round(x) for x in ax.get_xticks()], fontsize=fontsize)
            ax.set_yticklabels([round(x) for x in ax.get_yticks()], fontsize=fontsize)        

        if show:
            plt.show()
        if save:
            fig.savefig(f"Figures/Pareto Fronts/{title}_pf.png")

        # Reset fig, ax
        fig, ax = None, None


def compare_tbvs_heatmap(models=[TCDT_MOO_NSGA2, TCDT_MOO_NSGA3], r=2, numbers_of_drones=[4,8,12,16], numbers_of_visits=[2,3], show=True, save=False):
    """Compare TBV performances of models with the same algorithm"""
    assert len(models) == 2, "Only two models can be compared"
    assert(models[0]['Alg'] == models[1]['Alg']), "Algorithms must be the same"
    alg = models[0]['Alg']
    all_solution_filenames = [x for x in os.listdir(solutions_filepath) if "SolutionObjects" in x]
    # Replace 2*sqrt(2) with sqrt(8)
    if r == sqrt(8):
        r = "sqrt(8)"
    
    # Initialize a matrix to store the differences
    diff_matrix = np.zeros((len(numbers_of_visits), len(numbers_of_drones)))
    
    for i, v in enumerate(numbers_of_visits):
        for j, n in enumerate(numbers_of_drones):
            model_min_mean_tbvs = []
            for model in models:
                # Get the solution filename
                try:
                    solutions_filename = [x for x in all_solution_filenames if f"r_{r}" in x and f"minv_{v}" in x and f"n_{n}" in x and model["Type"] in x and model["Alg"] in x and model["Exp"] in x][0]
                except:
                    print(f"Model: {model['Exp']}, Alg: {model['Alg']}, n={n}, v={v}")
                    print(f"r_{r}, minv_{v}, n_{n}")
                    raise
                solution_objects = load_pickle(f"{solutions_filepath}{solutions_filename}")
                min_max_mean_tbv = inf
                for x in solution_objects:
                    sol = x[0]
                    max_mean_tbv = calculate_max_mean_tbv(sol)
                        # print(f"Model: {model['Exp']}, Alg: {model['Alg']}, n={n}, v={v}")
                        # print(f"Visit Times: {sol.visit_times}")
                        # print(f"Path Matrix:\n{sol.real_time_path_matrix}")
                        # print(f"Model: {model['Exp']}, Alg: {model['Alg']}, n={n}, v={v}, Cumulative Mean TBV: {cum_mean_tbv}")
                    if max_mean_tbv < min_max_mean_tbv:
                        min_max_mean_tbv = max_mean_tbv
                # cum_mean_tbvs = [calculate_cumulative_mean_tbv(x[0]) for x in solution_objects]
                # min_cum_mean_tbv = min(cum_mean_tbvs)
                model_min_mean_tbvs.append(min_max_mean_tbv)
            # Calculate the difference between the two models
            # print(model_min_mean_tbvs)
            diff_matrix[i, j] = (model_min_mean_tbvs[0] - model_min_mean_tbvs[1])/model_min_mean_tbvs[0] * 100
    
    # Create a heatmap
    fig, ax = plt.subplots()
    sns.heatmap(diff_matrix, annot=True, fmt=".2f", xticklabels=numbers_of_drones, yticklabels=numbers_of_visits, ax=ax)
    ax.set_xlabel("Number of Drones")
    ax.set_ylabel("Number of Visits")
    title = f"Max Mean TBV Performance Comparison for Alg: {alg}, r={r} ({models[0]['Exp']} - {models[1]['Exp']})"
    ax.set_title(title, fontsize="small")
    
    # Save plot
    if save:
        fig.savefig(f"Figures/Time Between Visits/Max Mean TBV Performance Comparison between Models/alg_{alg}_r_{r}_max_mean_tbv_performance_comparison.png")
    
    # Show plot
    if show:
        plt.show()
    else:
        plt.close(fig)


def compare_objs_for_models_heatmap(models=[TCDT_MOO_NSGA2, TCDT_MOO_NSGA3], r=2, numbers_of_drones=[4,8,12,16], numbers_of_visits=[2,3], show=True, save=False):
    """Show the gain in performance for the models (one without tbv and one with tbv same algorithm)"""
    assert len(models) == 2, "Only two models can be compared"
    assert(models[0]['Alg'] == models[1]['Alg']), "Algorithms must be the same"
    alg = models[0]['Alg']
    all_objective_filenames = os.listdir(objective_values_filepath)
    all_solution_filenames = os.listdir(solutions_filepath)
    # Replace 2*sqrt(2) with sqrt(8)
    if r == sqrt(8):
        r = "sqrt(8)"
    # Get common objectives between models
    objective_names = models[0]["F"]
    for objective in objective_names:
        if objective not in models[1]["F"]:
            objective_names.remove(objective)
    
    for objective_name in objective_names:
        # print(f"Objective: {objective_name}")
        # Initialize a matrix to store the differences
        # diff_matrix = np.empty((len(numbers_of_visits), len(numbers_of_drones)), dtype=str)
        # print(f"Diff Matrix: {diff_matrix}")
        # diff_matrix = np.empty((len(numbers_of_visits), len(numbers_of_drones)), dtype=str)
        diff_matrix = np.zeros((len(numbers_of_visits), len(numbers_of_drones)))
        
        for i, v in enumerate(numbers_of_visits):
            for j, n in enumerate(numbers_of_drones):
                # print(f"Number of Drones: {n}, Number of Visits: {v}")
                model_best_objective_values = []
                for model in models:
                    # print(f"Model: {model['Exp']}")
                    objective_filename = [x for x in all_objective_filenames if f"r_{r}" in x and f"minv_{v}" in x and f"n_{n}" in x and model["Type"] in x and model["Alg"] in x and model["Exp"] in x][0]
                    # print(f"Objective Filename: {objective_filename}")
                    objective_values = pd.read_pickle(f"{objective_values_filepath}{objective_filename}")
                    # print(f"Objective Values: {np.array(sorted(objective_values[objective_name]))}")
                    best_objective_value = min(objective_values[objective_name]) if objective_name != "Percentage Connectivity" else max(objective_values[objective_name])
                    # print(f"Best Objective Value for {model['Exp']}: {best_objective_value}")
                    model_best_objective_values.append(best_objective_value)
                # Calculate the difference between the two models (percentage change)
                perc_diff = (model_best_objective_values[0] - model_best_objective_values[1])/model_best_objective_values[0] * 100
                diff_matrix[i, j] = perc_diff
                # diff_matrix[i, j] = (model_best_objective_values[0] - model_best_objective_values[1])/model_best_objective_values[0] * 100
                print(diff_matrix[i, j], perc_diff)
                # print(f"Value 1: {model_best_objective_values[0]}, Value 2: {model_best_objective_values[1]}, Difference: {diff_matrix[i, j]}")
        
        # Create a heatmap
        fig, ax = plt.subplots()
        if objective_name != "Percentage Connectivity":
            cmap = sns.diverging_palette(150, 10, as_cmap=True)  # Green to Red with white in the middle
        else:
            cmap = sns.diverging_palette(10, 150, as_cmap=True)  # Red to Green with white in the middle
        norm = TwoSlopeNorm(vmin=diff_matrix.min(), vcenter=0, vmax=diff_matrix.max())
        sns.heatmap(diff_matrix, annot=np.vectorize(format_percentage)(diff_matrix), cmap=cmap, fmt="", annot_kws={"fontsize": 10}, cbar_kws={'label': 'Color Bar'},  xticklabels=numbers_of_drones, yticklabels=numbers_of_visits, ax=ax)
        ax.set_xlabel("Number of Drones")
        ax.set_ylabel("Number of Visits")
        ax.set_title(f"Difference in {objective_name} Performance for Communication Range: {r} ({alg})", fontsize="small")
        
        # Save plot
        if save:
            fig.savefig(f"Figures/Objective Values/{alg}_{objective_name.replace(' ', '_')}_performance_comparison_between_models.png")
        
        # Show plot
        if show:
            plt.show()
        else:
            plt.close(fig)


def compare_objs_for_models_lineplot(models=[TCDT_MOO_NSGA2, TCDT_MOO_NSGA3], r=2, numbers_of_drones=[4,8,12,16], numbers_of_visits=[2,3], show=True, save=False):
    """Show the gain in performance for the models (one without tbv and one with tbv same algorithm)"""
    assert len(models) == 2, "Only two models can be compared"
    assert(models[0]['Alg'] == models[1]['Alg']), "Algorithms must be the same"
    alg = models[0]['Alg']
    all_objective_filenames = os.listdir(objective_values_filepath)
    all_solution_filenames = os.listdir(solutions_filepath)
    # Replace 2*sqrt(2) with sqrt(8)
    if r == sqrt(8):
        r = "sqrt(8)"
    # Get common objectives between models
    objective_names = models[0]["F"]
    for objective in objective_names:
        if objective not in models[1]["F"]:
            objective_names.remove(objective)
    for objective_name in objective_names:
        fig, ax = plt.subplots()
        ax.grid()
        ax.set_xticks(numbers_of_drones)
        ax.set_xlabel("Number of Drones")
        ax.set_ylabel(objective_name)
        ax.set_title(f"{objective_name} Performance Comparison for Communication Range: {r} ({alg})", fontsize="small")
        for model in models:
            for v in numbers_of_visits:
                y = []
                for n in numbers_of_drones:
                    objective_filename = [x for x in all_objective_filenames if f"r_{r}" in x and f"v_{v}" in x and f"n_{n}" in x and model["Type"] in x and model["Alg"] in x and model["Exp"] in x][0]
                    # solution_filename = [x for x in all_solution_filenames if f"r_{r}" in x and f"v_{v}" in x and f"n_{n}" in x and model["Type"] in x and model["Alg"] in x and model["Exp"] and "" in x]
                    objective_values = load_pickle(f"{objective_values_filepath}{objective_filename}")
                    best_objective_value = min(objective_values[objective_name]) if objective_name != "Percentage Connectivity" else max(objective_values[objective_name])
                    y.append(best_objective_value)
                ax.plot(numbers_of_drones, y, linestyle="dashdot", marker="o", label=f"{model['Exp']} - {v} Visit(s)")
        ax.legend()
    if show:
        plt.show()
    if save:
        pass
        # fig.savefig(f"Figures/Performance Comparison/{alg}_performance_comparison.png")

def lineplot_for_runtimes(alg:str, model_exp:str, comm_ranges:list, numbers_of_drones:list, numbers_of_visits:list,  show=True, save=False):
    """Plot average runtimes for different communication ranges, numbers of drones and visits"""
    assert isinstance(comm_ranges, list) and all((isinstance(x, int) or isinstance(x, float)) and x > 0 for x in comm_ranges), "comm_ranges must be a list of numbers greater than 0"
    assert isinstance(numbers_of_drones, list) and all(isinstance(x, int) and x > 0 for x in numbers_of_drones), "numbers_of_drones must be a list of integers greater than 0"
    assert isinstance(numbers_of_visits, list) and all(isinstance(x, int) and x > 0 for x in numbers_of_visits), "numbers_of_visits must be a list of integers greater than 0"
    runtimes_filenames = os.listdir(runtimes_filepath)
    filtered_runtime_filenames = [x for x in runtimes_filenames if model_exp in x]
    # Create x for plot
    x = numbers_of_drones
    fig, ax = plt.subplots()
    ax.grid()
    ax.set_xticks(numbers_of_drones)
    ax.set_xlabel("Number of Drones")
    ax.set_ylabel("Average Runtime")
    ax.set_title(f"{alg} Average Runtime for Different Numbers of Drones and Communication Ranges", fontsize="small")
    for v in numbers_of_visits:
        print(f"Number of Visits: {v}", sep=" ")
        # Initialize y for plot
        y = []
        # Create a figure and axis
        for n in numbers_of_drones:
            print(f"Number of Drones: {n}", sep=" ")
            runtimes = []
            for r in comm_ranges:
                if r == 2*sqrt(2):
                    r = "sqrt(8)"
                print(f"Comm Range: {r}", sep="\n")
                filtered_runtime_filenames = [x for x in runtimes_filenames if f"v_{v}" in x and f"n_{n}" in x and f"r_{r}" in x and alg in x]
                print("Filenames:\n",np.array(filtered_runtime_filenames))
                runtime = load_pickle(f"{runtimes_filepath}{filtered_runtime_filenames[0]}") if len(filtered_runtime_filenames)==1 else sum([load_pickle(f"{runtimes_filepath}{x}") for x in filtered_runtime_filenames])
                print(f"Runtime: {runtime}")
                runtimes.append(runtime)
            average_runtime = np.mean(runtimes)
            # Update y for plot
            y.append(average_runtime)
        # Add lineplot for v visit(s) to the figure
        ax.plot(x, y, marker='o', label=f"{v} Visit(s)")
    
    # Add a legend to the plot
    ax.legend()
    # Show plot
    if show:
        plt.show()
    # Save plot
    if save:
        fig.savefig(f"Figures/Average Runtimes/{model_exp}_runtimes.png")


def compare_average_runtimes_for_different_algorithms(algorithms: list, model_exp:str):

    runtime_filenames = os.listdir(runtimes_filepath)
    average_runtimes = dict()
    for alg in algorithms:
        average_runtimes[alg] = []
        alg_runtime_filenames = [x for x in runtime_filenames if model_exp in x and alg in x]
        for filename in alg_runtime_filenames:
            runtime = load_pickle(f"{runtimes_filepath}{filename}")
            average_runtimes[alg].append(runtime)
        average_runtimes[alg] = np.round(np.mean(average_runtimes[alg])/60)  # Convert to minutes
    print(average_runtimes)
    return average_runtimes


def model_comparison_heatmap_for_best_objs(models: list, r, numbers_of_drones: list, numbers_of_visits: list, show=True, save=False):
    """Plot a heatmap for comparison of the best objective values of the models for different numbers of drones and visits"""
    assert len(models) == 2, "Only two models can be compared"
    assert isinstance(r, (float, int)), "Communication range should be a number"
    assert isinstance(numbers_of_drones, list) and all(isinstance(x, int) and x > 0 for x in numbers_of_drones), "numbers_of_drones must be a list of integers greater than 0"
    assert isinstance(numbers_of_visits, list) and all(isinstance(x, int) and x > 0 for x in numbers_of_visits), "numbers_of_visits must be a list of integers greater than 0"
    
    objectives_folder_filenames = os.listdir(objective_values_filepath)
    # First filter by communication range
    filtered_objective_filenames = [x for x in objectives_folder_filenames if f"r_{r}" in x]
    # Get common objectives between models
    objective_names = models[0]["F"]
    for objective_name in objective_names:
        for model in models[1:]:
            if objective_name not in model["F"]:
                objective_names.remove(objective_name)
    
    for objective_name in objective_names:
        # Create a figure and axis
        fig, ax = plt.subplots()
        # ax.grid()
        ax.set_xticks(numbers_of_drones)
        ax.set_xlabel("Number of Drones")
        ax.set_ylabel(f"Objective: {objective_name}")
        ax.set_title(f"Best {objective_name} Comparison by Algorithm ({models[0]['Alg']} - {models[1]['Alg']})", fontsize=10)
        
        best_objective_values_for_models = {model["Alg"]: [] for model in models}
        
        for v in numbers_of_visits:
            row = []
            for n in numbers_of_drones:
                model_best_objective_values = []
                for model in models:
                    objective_filename = [x for x in filtered_objective_filenames if model["Type"] in x and model["Alg"] in x and model["Exp"] in x and f"v_{v}" in x and f"n_{n}" in x][0]
                    objective_values = pd.read_pickle(f"{objective_values_filepath}{objective_filename}")[objective_name]
                    best_objective_value = min(objective_values) if objective_name != "Percentage Connectivity" else max(objective_values)
                    model_best_objective_values.append(best_objective_value)
                model_best_objective_values_diff = model_best_objective_values[0] - model_best_objective_values[1]
                # print(model_best_objective_values_diff)
                row.append(model_best_objective_values_diff)
            best_objective_values_for_models[models[0]["Alg"]].append(row)
        
        # Convert to numpy array for heatmap
        data = np.array(best_objective_values_for_models[models[0]["Alg"]])
        
        # Plot heatmap
        sns.heatmap(data, annot=True, fmt=".2f", xticklabels=numbers_of_drones, yticklabels=numbers_of_visits, ax=ax)
        
        # Save plot
        if save:
            fig.savefig(f"Figures/Heatmaps/r_{r}_comparison_{objective_name.replace(' ', '_')}.png")
        
        # Show plot
        if show:
            plt.show()
        else:
            plt.close(fig)


def plot_objs_for_best_mission_time(models, r, n, v, show=False, save=True):
    assert len(models) <= 2, "Only two models can be compared"
    if len(models) == 2:
        assert models[0]['Alg'] == models[1]['Alg'], "Algorithms must be the same"
    if  not isinstance(models, list):
        models = [models]
    if isinstance(r, float) or isinstance(r, int):
        r = [r]
    if isinstance(n, float) or isinstance(n, int):
        n = [n]
    if isinstance(v, float) or isinstance(v, int):
        n = [v]

    # Define linestyles for different models
    fontsize = 14
    linestyles = ['solid','--',":"]
    markers = ['o', '>', 'x']
    linecolors = ['black', 'blue', 'red']
    markerfacecolors = ['none', 'none', 'none']  # Hollow and filled markers
    markeredgecolors = ['black', 'blue', 'red']  # Edge colors for markers

    unit_dict = {"Percentage Connectivity": "%", "Max Mean TBV": "sec", "Max Disconnected Time":"timestep", "Mean Disconnected Time":r"$\frac{Isolated Timesteps}{Total Timesteps}$"}

    # Get common objectives between models
    if len(models) == 2:
        objective_names = [x for x in models[0]["F"] if x in models[1]["F"]]
    else:
        objective_names = models[0]["F"]
    objective_names.remove("Mission Time")
    if "Max Mean TBV" not in objective_names:
        objective_names.append("Max Mean TBV")
    
    all_objective_filenames = os.listdir(objective_values_filepath)
    all_solution_filenames = [x for x in os.listdir(solutions_filepath) if "SolutionObjects" in x]
    best_mission_time_solution_filenames = [x for x in os.listdir(solutions_filepath) if "Best-Mission_Time" in x]

    for objective in objective_names:
        if objective == "Max Mean TBV as Objective":
            v_new = [x for x in v if x!=1]
        else:
            v_new = v
        print(objective, v_new)
        for r_value in r:
            debug_counter = 0
            # Create a figure and axis
            fig, ax = plt.subplots(figsize=(8, 6))
            ax.set_xticks(n)
            ax.set_xticklabels(n, fontsize=16)
            ax.grid()
            ax.set_xlabel('Number of Drones', fontsize=16)
            ax.set_ylabel(f"{objective} ({unit_dict[objective]})", fontsize=16)
            
            # Modify r_value
            if r_value == sqrt(8):
                r_value = "sqrt(8)"
            for i, model in enumerate(models):
                linestyle = linestyles[models.index(model)]
                color = linecolors[models.index(model)]
                # marker = markers[models.index(model)]
                markerfacecolor = markerfacecolors[models.index(model)]
                markeredgecolor = markeredgecolors[models.index(model)]
                # Skip 1 visit for TBV Plots
                for j, v_value in enumerate(v_new):
                    if "TBV" in objective and v_value == 1:
                        continue
                    marker = markers[j]
                    y = []
                    for n_value in n:
                        best_mission_time_solution_filename = [x for x in best_mission_time_solution_filenames if f"r_{r_value}" in x and f"minv_{v_value}" in x and f"n_{n_value}" in x and f'{model["Type"]}_{model["Alg"]}_{model["Exp"]}_g' in x][0]
                        best_mission_time_solution = load_pickle(f"{solutions_filepath}{best_mission_time_solution_filename}")
                        objective_value = get_attribute(best_mission_time_solution, objective)*100 if objective == "Percentage Connectivity" else get_attribute(best_mission_time_solution, objective)
                        y.append(objective_value)
                    model_underscript__minv = "$" + model["Exp"] + "_" + "{" + str(v_value) + "}" + "$"
                    ax.plot(n, y, linestyle=linestyle, color=color, linewidth=2, marker=marker, markerfacecolor=markerfacecolor, markeredgecolor=markeredgecolor, label=rf'{model_underscript__minv}')

            # Set the title
            ax.set_title(f'Best Mission Time Solution {objective} Values for {r_value} Cell(s) Communication Range', pad=60, fontsize=16)
            ax.set_yticklabels([round(x) for x in ax.get_yticks()], fontsize=16)
            # Adjust the plot area to make space for the legend
            fig.subplots_adjust(top=0.8) # 0.8
            # Add a legend to the plot
            ax.legend(ncol=3, loc="upper center", bbox_to_anchor=(0.5, 1.185),  fontsize=16)
            # Annotate
            # for j in range(len(n)):
            #     ax.annotate(f'{round(y[j], 2)}', (n[j], n[j]), textcoords="offset points", xytext=(0,5), ha='center')
            # Save plot
            if save:
                fig.savefig(f"Figures/Objective Values/{model['Alg']}_r_{r_value}_{objective.replace(' ', '_')}_best_values.png")
            # Show plot
            if show:
                plt.show()


def plot_best_objs_for_nvisits(models, r, n, v, show=False, save=True):

    if save:
        model_info = ""
        for model in models:
            model_info += f"{model['Exp']}_{model['Alg']}_"
        model_info = model_info[:-1]

    model_exps = [model["Exp"] for model in models]

    # Define linestyles for different models
    fontsize = 14
    linestyles = ['solid','--', ':','-.']
    markers = ['o', '>', 'x','D']
    linecolors = ['black', 'blue', 'red', 'green']
    markerfacecolors = ['none', 'none', 'none', 'none']  # Hollow and filled markers
    markeredgecolors = ['black', 'blue', 'red', 'green']  # Edge colors for markers

    unit_dict = {"Mission Time": "sec", "Percentage Connectivity": "%", "Max Mean TBV": "sec", "Max Disconnected Time":"timestep", "Mean Disconnected Time":r"$\frac{Isolated Timesteps}{Total Timesteps}$"}

    objective_names = []
    for model in models:
        for objective in model["F"]:
            if objective not in objective_names and "Weighted Sum" not in objective:
                objective_names.append(objective)

    info = PathInfo()

    for objective in objective_names:
        print("Objective:", objective)
        for r_value in r:
            info.comm_cell_range = r_value
            # Create a figure and axis
            fig, ax = plt.subplots(figsize=(8, 6))
            ax.set_xticks(n)
            ax.set_xticklabels(n, fontsize=16)
            ax.grid()
            ax.set_xlabel('Number of Drones', fontsize=16)
            if "Weighted Sum" in objective:
                ax.set_ylabel(f"{objective} (-)", fontsize=16)
            else:
                ax.set_ylabel(f"{objective} ({unit_dict[objective]})", fontsize=16)
            
            # Modify r_value
            if r_value == sqrt(8):
                r_value = "sqrt(8)"
            for i, model in enumerate(models):
                info.model = model
                linestyle = linestyles[i]
                color = linecolors[i]
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

                        for objective_name in objective_names:
                            if objective_name not in list(objective_values.columns):
                                if "Weighted Sum" in objective:
                                    new_objective_values = [calculate_ws_score_from_ws_objective(sol) for sol in solution_objects]
                                else:
                                    new_objective_values = [getattr(sol, obj_name_sol_attr_dict[objective]) for sol in solution_objects]
                                objective_values[objective_name] = new_objective_values


                        best_tbv_solution = solution_objects[objective_values["Max Mean TBV"].idxmin()] # if (objective == "Mission Time" or objective == "Percentage Connectivity") and model==TCDT_MOO_NSGA2 else None
                        best_time_solution = solution_objects[objective_values["Mission Time"].idxmin()] # if (objective == "Mission Time" or objective == "Percentage Connectivity") and model==TCDT_MOO_NSGA2 else None
                        best_objective_value = abs(objective_values[objective].min())
                        print(f"{scenario} best {objective} value: {best_objective_value}")

                        time_at_best_tbv = get_attribute(best_tbv_solution, "Mission Time") # if objective == "Mission Time" else None
                        conn_at_best_tbv = get_attribute(best_tbv_solution, "Percentage Connectivity")*100 # if objective == "Percentage Connectivity" else None
                        tbv_at_best_time = get_attribute(best_time_solution, "Max Mean TBV") # if objective == "Max Mean TBV" else None

                        y.append(best_objective_value)
                        y_time_at_best_tbv.append(time_at_best_tbv) # if objective == "Mission Time" and model==TCDT_MOO_NSGA2 else None
                        y_conn_at_best_tbv.append(conn_at_best_tbv) # if objective == "Percentage Connectivity" and model==TCDT_MOO_NSGA2 else None
                        y_tbv_at_best_time.append(tbv_at_best_time) # if objective == "Max Mean TBV" and model==TCDT_MOO_NSGA2 else None
                    model_underscript__minv = "$" + model["Exp"] + "_" + "{" + f"{model['Alg']},{str(v_value)}" + "}" + "$"
                    ax.plot(n, y, linestyle=linestyle, color=color, linewidth=2, marker=marker, markerfacecolor=markerfacecolor, markeredgecolor=markeredgecolor, label=rf'{model_underscript__minv}')

            ax.set_title(f'Best {objective} Values for {r_value} Cell(s) Communication Range', pad=70, fontsize=14)
            ax.set_yticklabels(ax.get_yticklabels(), fontsize=16)
            # Adjust the plot area to make space for the legend
            fig.subplots_adjust(top=0.8) # 0.8
            # Add a legend to the plot
            ax.legend(ncol=3, loc="upper center", bbox_to_anchor=(0.5, 1.24), fontsize=14) # 1.185
            # Save plot
            if save:
                fig.savefig(f"Figures/Objective Values/{model_info}_r_{r_value}_{objective.replace(' ', '_')}_best_values.png")
            # Show plot
            if show:
                plt.show()
                        

def plot_time_between_visits_vs_number_of_drones(model:dict, r_values:list, numbers_of_drones:list, numbers_of_visits:list, show=True, save=False):
    """Take the best objective solutions for the required scenarios and plot tbv vs number of drones"""

    filepath = "Figures/Time Between Visits/Mean Mean TBV vs Number of Drones/"

    assert (1 not in numbers_of_visits), "1 Should not be in numbers of visits"

    x = numbers_of_drones

    model_parameters_str = f"{model['Type']}_{model['Alg']}_{model['Exp']}"
    model_solutions_filenames = [x for x in os.listdir("Results/Solutions") if model_parameters_str in x]
    best_solutions_filenames = [x for x in model_solutions_filenames if "Best" in x]
    objective_names = model["F"] # list(set([x.split("-")[2].replace("_", " ") for x in best_solutions_filenames]))
    best_solutions = [load_pickle(f"{solutions_filepath}{x}") for x in best_solutions_filenames]
    infos_of_best_solutions = [x.info for x in best_solutions]

    for r_comm in r_values:
        for obj in objective_names:
            objective_with_underscore = obj.replace(" ", "_")
            fig, ax = plt.subplots(figsize=(7,5))
            ax.set_xticks(numbers_of_drones)
            ax.grid()
            for n_visits in numbers_of_visits:
                y = []
                max_values = []
                for n_drones in numbers_of_drones:
                    print(obj, n_drones, r_comm, n_visits)
                    n_drones_r_comm_n_visits_best_obj_solution = [
                                                                    best_solutions[i] for i in range(len(best_solutions)) if infos_of_best_solutions[i].model["F"]==model["F"] # Filter model parameters
                                                                    and infos_of_best_solutions[i].model["Type"]==model["Type"] 
                                                                    and infos_of_best_solutions[i].model["Exp"]==model["Exp"] 
                                                                    and infos_of_best_solutions[i].model["Alg"]==model["Alg"] # Filter model parameters
                                                                    and infos_of_best_solutions[i].n_visits==n_visits 
                                                                    and infos_of_best_solutions[i].number_of_drones==n_drones 
                                                                    and infos_of_best_solutions[i].comm_cell_range==r_comm # Filter scenario parameters
                                                                    and obj.replace(" ","_") in best_solutions_filenames[i] # Filter objective
                                                                ][0]
                    n_drones_r_comm_n_visits_best_obj_info = n_drones_r_comm_n_visits_best_obj_solution.info
                    scenario = str(n_drones_r_comm_n_visits_best_obj_info)
                    mean_tbv = n_drones_r_comm_n_visits_best_obj_solution.mean_tbv
                    cum_mean_tbv = np.mean(mean_tbv)
                    # Append cumulative mean tbv to y
                    y.append(cum_mean_tbv)
                # Scatter Plot
                ax.scatter(x, y)
                ax.plot(x, y, label=f"{n_visits} visit(s)")
                variance = np.var(y)
                std_dev = np.sqrt(variance)
                # Add shaded variance area
                ax.fill_between(x, y - variance, y + variance, alpha=0.2, label=f"{n_visits} visit(s) variance")
                max_values.append(max(y))

            
            max_value = max(max_values)
            ax.set_title(f"Best {obj} Solution Cumulative Mean TBV vs Number of Drones with Variance", fontsize="small")
            ax.legend(loc='upper right', ncol=2, fontsize="small")

            if show:
                plt.show()

            if save:
                fig.savefig(f"{filepath}{scenario}-Best-{objective_with_underscore}-mean_tbv_vs_number_of_drones_with_variance.png")


def plot_time_between_visits_vs_dist_to_bs(info:PathInfo, show=False, save=True):

    mean_tbv_vs_dist_to_bs_dist_folder = "Figures/Time Between Visits/Distance to BS vs Mean TBV/Distribution/"
    mean_tbv_vs_dist_to_bs_hist_folder = "Figures/Time Between Visits/Distance to BS vs Mean TBV/Histogram/"
    cum_mean_tbv_vs_number_of_drones_folder = "Figures/Time Between Visits/Cumulative Mean TBV vs Number of Drones/"

    scenario = str(info)
    brief_scenario = f"n_{info.number_of_drones}_r_{info.comm_cell_range}_v_{info.n_visits}" if info.comm_cell_range != 2*sqrt(2) else f"n_{info.number_of_drones}_r_{sp.sqrt(round(info.comm_cell_range**2))}_v_{info.n_visits}"
    save_as = f"{info.model['Type']}_{info.model['Alg']}_{info.model['Exp']}_n_{info.number_of_drones}_r_{info.comm_cell_range}_v_{info.n_visits}" if info.comm_cell_range != 2*sqrt(2) else f"{info.model['Type']}_{info.model['Alg']}_{info.model['Exp']}_n_{info.number_of_drones}_r_{sp.sqrt(round(info.comm_cell_range**2))}_v_{info.n_visits}"

    scenario_all_objective_values = pd.read_pickle(f"{objective_values_filepath}{scenario}-ObjectiveValues.pkl")
    objective_names = scenario_all_objective_values.columns

    dist_to_bs = info.D[-1][:-1]

    directions = ["Best"]

    for objective in objective_names:

        # Create a figure and axis
        mean_tbv_vs_dist_to_bs_dist_fig, mean_tbv_vs_dist_to_bs_dist_ax = plt.subplots()
        mean_tbv_vs_dist_to_bs_hist_fig, mean_tbv_vs_dist_to_bs_hist_ax = plt.subplots()
        # ax.set_xticks(dist_to_bs)
        mean_tbv_vs_dist_to_bs_dist_ax.grid(axis='y')
        # Add axis labels and a title
        mean_tbv_vs_dist_to_bs_dist_ax.set_xlabel('Distance to BS')
        mean_tbv_vs_dist_to_bs_dist_ax.set_ylabel("Mean Time Between Visits")

        mean_tbv_vs_dist_to_bs_hist_ax.set_xlabel('Mean Time Between Visits')
        mean_tbv_vs_dist_to_bs_hist_ax.set_ylabel("Frequency")

        objective_with_underscore = objective.replace(" ","_")

        for dir in directions:
            sol = load_pickle(f"{solutions_filepath}{scenario}-{dir}-{objective_with_underscore}-Solution.pkl")
            vt = get_visit_times(sol)
            tbv = calculate_tbv(vt)
            mean_tbv = [np.mean(x) for x in tbv]
            cum_mean_tbv = np.mean(mean_tbv)
            mean_tbv_vs_dist_to_bs_dist_ax.set_title(f'{save_as} {dir} {objective} Solution Mean TBV', fontsize="small")
            mean_tbv_vs_dist_to_bs_dist_ax.scatter(dist_to_bs, mean_tbv, label="Mean TBV")
            mean_tbv_vs_dist_to_bs_dist_ax.plot(np.arange(ceil(max(dist_to_bs))+1), [cum_mean_tbv]*(ceil(max(dist_to_bs))+1), label = "Cumulative Mean TBV", color="blue")
            mean_tbv_vs_dist_to_bs_dist_ax.set_xlim([0,round(max(dist_to_bs))+1])
            mean_tbv_vs_dist_to_bs_dist_ax.set_xticks([min(dist_to_bs), np.mean(dist_to_bs), max(dist_to_bs)])
            mean_tbv_vs_dist_to_bs_dist_ax.legend()
            bin_width = 1.0
            bins = np.arange(min(mean_tbv), max(mean_tbv) + bin_width, bin_width)
            mean_tbv_vs_dist_to_bs_hist_ax.set_title(f'{save_as} {dir} {objective} Solution Mean TBV', fontsize="small")
            mean_tbv_vs_dist_to_bs_hist_ax.hist(mean_tbv, bins=bins, edgecolor='black')
            # Save Plot
            if save:
                mean_tbv_vs_dist_to_bs_dist_fig.savefig(f"{mean_tbv_vs_dist_to_bs_dist_folder}{save_as}-{dir}-{objective_with_underscore}-mean_tbv_dist.png")
                mean_tbv_vs_dist_to_bs_hist_fig.savefig(f"{mean_tbv_vs_dist_to_bs_hist_folder}{save_as}-{dir}-{objective_with_underscore}-mean_tbv_hist.png")
            if show:
                plt.show()


"""Plot Paret Fronts"""
# plot_pareto_fronts(save=False, show=True)
    
"""Plot Time Conn for Best TBV & Conn TBV for Best Mission Time"""
# plot_mission_time_percentage_connectivity_for_best_tbv(TCDT_MOO_NSGA2, [4,8,12,16], [2,2*sqrt(2),4], [1,2,3], show=False, save=True)
# plot_conn_tbv_for_best_mission_time(TCDT_MOO_NSGA2, [4,8,12,16], [2,2*sqrt(2),4], [1,2,3], show=False, save=True)


"""Plot Objetive Values for Best Mission Time"""
# plot_objs_for_best_mission_time(models=[TCDT_MOO_NSGA2, TCD_MOO_NSGA2], r=[2, 2*sqrt(2), 4], n=[4,8,12,16], v=[1,2,3], show=True, save=False)

"""Compare Min TBV Performance for Models Heatmap"""
# compare_tbvs_heatmap(models=[time_conn_disconn_tbv_nsga3_model, time_conn_disconn_nsga3_model], r=sqrt(8), numbers_of_drones=[4,8,12,16], numbers_of_visits=[2,3], show=True, save=True)

"""Compare Objective Values' Performance for Models Heatmap"""
# compare_objs_for_models_heatmap([time_conn_disconn_tbv_nsga2_model, time_conn_disconn_nsga2_model], r, numbers_of_drones, numbers_of_visits, show, save)

"""Lineplot for average runtimes"""
# lineplot_for_runtimes(alg, model_exp, comm_ranges, numbers_of_drones, numbers_of_visits,  show, save)

"""Average runtime comparson between algorithms"""
# compare_average_runtimes_for_different_algorithms(["NSGA2", "NSGA3"], "time_conn_disconn_tbv")

"""Plot TBV vs Number of Drones"""
# plot_time_between_visits_vs_number_of_drones(model=time_conn_disconn_tbv_nsga2_model, r_values=[2, 2*sqrt(2), 4], numbers_of_drones=[4,8,12,16], numbers_of_visits=[2,3], show=False, save=True)

"""Heatmap"""
# model_comparison_heatmap_for_best_objs([time_conn_disconn_tbv_nsga2_model, time_conn_disconn_tbv_nsga3_model], 2, [4,8,12,16], [1,2,3], show=True, save=False)

"""Plot Objs"""
plot_best_objs_for_nvisits(models=[TCDT_MOO_NSGA2, TCDT_WS, TCD_MOO_NSGA2, MTSP], r=[2, 2*sqrt(2)], n=[4,8,12,16], v=[1,2,3], show=False, save=True)

"""Pareto-Front"""
# plot_pareto_fronts(show=False, save=True)

""""New & Improved Objective Value Plot"""
# Remember to add mean_disonnected_tme and max_disconnected_time to all solution objects !!!
# plot_objective_values([time_conn_disconn_tbv_nsga2_model, time_conn_disconn_nsga2_model], objectives=["Mission Time","Percentage Connectivity","Max Mean TBV"], number_of_drones_values=[4,8,12,16], comm_cell_range_values=[2,2*sqrt(2),4], minv_values=[1,2,3], put_model_data_on_same_plot=True, show=True, save=False)


"""Plot TBV vs Dist to BS"""
# info = PathInfo()
# info.model = time_conn_disconn_tbv_nsga2_model
# numbers_of_drones_values = [4,8,12,16]
# comm_cell_range_values = [2, 2*sqrt(2), 4]
# n_visits_values = [2,3]
# for n in numbers_of_drones_values:
#     info.number_of_drones = n
#     for r in comm_cell_range_values:
#         info.comm_cell_range = r
#         for v in n_visits_values:
#             info.n_visits = v
#             plot_time_between_visits_vs_dist_to_bs(info=info, show=True, save=False)