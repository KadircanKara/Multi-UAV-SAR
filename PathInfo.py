import numpy as np
import pandas as pd
# import openpyxl
from math import sqrt
from pymoo.core.problem import ElementwiseProblem
from scipy.spatial import distance
from typing import List, Dict
import random
from math import floor
import pandas as pd

from PathInput import model, test_setup_scenario, comprehensive_setup_scenarios

# default_scenario = {
#     'grid_size': 8,
#     'cell_side_length': 50,
#     'number_of_drones': 4,
#     'max_drone_speed': 2.5, # m/s
#     'comm_cell_range': 2,  # 2 cells
#     'min_visits': 2,  # Minimum number of cell visits
#     'max_visits':5, # Maximum number of cell visits
#     'number_of_targets': 1,
#     'target_positions':12,
#     'true_detection_probability': 0.99,
#     'false_detection_probability': 0.01,
#     'detection_threshold': 0.9,
#     'max_isolated_time': 0,
# }

class PathInfo(object):

    def __init__(self, scenario_dict=None) -> None:

        self.model = model

        # print("-->", scenario_dict)
        self.grid_size = scenario_dict['grid_size'] if scenario_dict else test_setup_scenario['grid_size']
        self.number_of_cells = self.grid_size ** 2
        self.cell_side_length = scenario_dict['cell_side_length'] if scenario_dict else test_setup_scenario['cell_side_length']
        self.number_of_drones = scenario_dict['number_of_drones'] if scenario_dict else test_setup_scenario['number_of_drones']
        self.number_of_nodes = self.number_of_drones + 1
        self.max_drone_speed = scenario_dict['max_drone_speed'] if scenario_dict else test_setup_scenario['max_drone_speed']
        self.comm_cell_range = scenario_dict['comm_cell_range'] if scenario_dict else test_setup_scenario['comm_cell_range']
        self.comm_dist = self.comm_cell_range * self.cell_side_length
        self.min_visits = scenario_dict['min_visits'] if scenario_dict else test_setup_scenario['min_visits']
        self.max_visits = scenario_dict['max_visits'] if scenario_dict else test_setup_scenario['max_visits']
        self.number_of_targets = scenario_dict['number_of_targets'] if scenario_dict else test_setup_scenario['number_of_targets']
        self.target_positions = scenario_dict['target_positions'] if scenario_dict else test_setup_scenario['target_positions']
        self.true_detection_probability = scenario_dict['true_detection_probability'] if scenario_dict else test_setup_scenario['true_detection_probability']
        self.false_detection_probability = scenario_dict['false_detection_probability'] if scenario_dict else test_setup_scenario['false_detection_probability']
        self.detection_threshold = scenario_dict['detection_threshold'] if scenario_dict else test_setup_scenario['detection_threshold']
        self.max_isolated_time = scenario_dict['max_isolated_time'] if scenario_dict else test_setup_scenario['max_isolated_time']

        P = [[i, j] for i in range(self.grid_size) for j in range(self.grid_size)]
        P.append([-1, -1])
        self.D = distance.cdist(P, P) * self.cell_side_length

        pd.DataFrame(self.D).to_csv("Distance Matrix.csv")

        # print(f"Distance Matrix:\n{pd.DataFrame(self.D).to_string(index=False)}")

        self.min_subtour_length_threshold = (self.number_of_cells * self.min_visits / self.number_of_drones)*self.cell_side_length
        self.max_subtour_length_threshold = (self.number_of_cells * self.min_visits / self.number_of_drones)*self.cell_side_length # self.min_subtour_length_threshold + 20


    def __str__(self) -> str:

        # print(self.model)

        multi_line_scenario = f'''{self.model['Type']}_{self.model['Alg']}_g_{self.grid_size}_a_{self.cell_side_length}_n_{self.number_of_drones}_
v_{self.max_drone_speed}_r_{self.comm_cell_range}_minv_{self.min_visits}_
maxv_{self.max_visits}_Nt_{self.number_of_targets}_tarPos_{self.target_positions}_
ptdet_{self.true_detection_probability}_pfdet_{self.false_detection_probability}_
detTh_{self.detection_threshold}_maxIso_{self.max_isolated_time}
'''

#         multi_line_scenario = f'''{model['Type']}_{model['Exp']}_{model['Alg']}_g_{self.grid_size}_a_{self.cell_side_length}_n_{self.number_of_drones}_
# v_{self.max_drone_speed}_r_{self.comm_cell_range}_minv_{self.min_visits}_
# maxv_{self.max_visits}_Nt_{self.number_of_targets}_tarPos_{self.target_positions}_
# ptdet_{self.true_detection_probability}_pfdet_{self.false_detection_probability}_
# detTh_{self.detection_threshold}_maxIso_{self.max_isolated_time}
# '''

        lines = multi_line_scenario.splitlines()

        single_line_scenario = ''.join(lines)

        return single_line_scenario

        # return (
        # ''.join(f'''{model['Exp']}_g_{self.grid_size}_a_{self.cell_side_length}_n_{self.number_of_drones}_
        #     v_{self.max_drone_speed}_r_{self.comm_cell_range}_minv_{self.min_visits}_
        #     maxv_{self.max_visits}_Nt_{self.number_of_targets}_tarPos_{self.target_positions}_
        #     ptdet_{self.true_detection_probability}_pfdet_{self.false_detection_probability}_
        #     detTh_{self.detection_threshold}_maxIso_{self.max_isolated_time}
        # '''.splitlines())
        # )

    # def get_scenario_from_info(self):
    #     return (
    #     f'''g_{self.grid_size}_a_{self.cell_side_length}_n_{self.number_of_drones}_
    #         v_{self.max_drone_speed}_r_{self.comm_cell_range}_minv_{self.min_visits}_
    #         maxv_{self.max_visits}_Nt_{self.number_of_targets}_tarPos_{self.target_positions}_
    #         ptdet_{self.true_detection_probability}_pfdet_{self.false_detection_probability}_
    #         detTh_{self.detection_threshold}_maxIso_{self.max_isolated_time}
    #     '''
    #     )

'''
class PathInfo:
    def __init__(self, grid_size=None, A=None, Nd=None, V=None, rc=None, min_visits=None, max_visits=None, Nt=None, tar_cell=None, p=None, q=None,
                 Th=None, max_isolated_time=None, hovering=True, realtime_connectivity=False, realtime_connectivity_stepsize=None):

        self.grid_size = grid_size if grid_size else default_input_parameters['grid_size']
        self.Nc = self.grid_size ** 2
        self.A = A if A else default_input_parameters['A']
        self.Nd = Nd if Nd else default_input_parameters['Nd']
        self.Nn = self.Nd + 1
        self.V = Nd if V else default_input_parameters['V']
        self.rc = rc if rc else default_input_parameters['rc']
        self.min_visits = min_visits if min_visits else default_input_parameters['min_visits']
        self.max_visits = max_visits if max_visits else default_input_parameters['max_visits']
        self.Nt = Nt if Nt else default_input_parameters['Nt']
        self.tar_cell = tar_cell if tar_cell else default_input_parameters['tar_cell']
        self.p = p if p else default_input_parameters['p']
        self.q = q if q else default_input_parameters['q']
        self.Th = Th if Th else default_input_parameters['Th']
        self.hovering = hovering if hovering else default_input_parameters['hovering']
        self.realtime_connectivity = realtime_connectivity if realtime_connectivity else default_input_parameters['realtime_connectivity']
        self.realtime_connectivity_stepsize = realtime_connectivity_stepsize if realtime_connectivity_stepsize else default_input_parameters['realtime_connectivity_stepsize']

        # self.subtour_length_th = (int(round(self.Nc * self.max_visits/self.Nd)^2))*self.A
        self.subtour_length_th = 1000



        P = [[i, j] for i in range(self.grid_size) for j in range(self.grid_size)]
        P.append([-1, -1])
        self.D = distance.cdist(P, P) * self.A


# x = PathInfo()
# print(pd.DataFrame(x.D))
'''
