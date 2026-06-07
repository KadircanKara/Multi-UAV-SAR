# import openpyxl
from math import sqrt
from scipy.spatial import distance
import pandas as pd

from PathInput import *

default_scenario = {
    'grid_size': 8,
    'cell_side_length': 50,
    'number_of_drones': 4,
    'max_drone_speed': 2.5, # m/s
    'comm_cell_range': 2,  # 2 cells
    'n_visits': 2,  # Minimum number of cell visits
    'target_positions': [12],
    'th': 0.9,
    'detection_probability': 0.7,
}

class PathInfo(object):

    def __init__(self, scenario_dict=None) -> None:

        self.model = model
        self.pop_size = pop_size
        self.n_gen = n_gen

        # print("-->", scenario_dict)
        self.grid_size = scenario_dict['grid_size']  if scenario_dict else default_scenario['grid_size']
        self.number_of_cells = self.grid_size ** 2
        self.cell_side_length = scenario_dict['cell_side_length'] if scenario_dict else default_scenario['cell_side_length']
        self.number_of_drones = scenario_dict['number_of_drones'] if scenario_dict else default_scenario['number_of_drones']
        self.number_of_nodes = self.number_of_drones + 1
        self.max_drone_speed = scenario_dict['max_drone_speed'] if scenario_dict else default_scenario['max_drone_speed']
        self.comm_cell_range = scenario_dict['comm_cell_range'] if scenario_dict else default_scenario['comm_cell_range']
        self.comm_dist = self.comm_cell_range * self.cell_side_length
        self.n_visits = scenario_dict['n_visits'] if scenario_dict else default_scenario['n_visits']
        self.target_locations = scenario_dict['target_positions'] if scenario_dict else default_scenario['target_positions']
        # self.occ_grid = np.full(shape=(self.number_of_nodes, self.number_of_cells), fill_value=0.5, dtype=float) # Initial occupancy probabilities of cells, 0.5
        self.th = scenario_dict['th'] if scenario_dict else default_scenario['th']
        self.detection_probability = scenario_dict['detection_probability'] if scenario_dict else default_scenario['detection_probability']
        self.miss_probability = 1-self.detection_probability
        # self.true_detection_prob = scenario_dict['true_detection_probability'] if scenario_dict else default_scenario['true_detection_probability'] 
        # self.false_miss_prob = scenario_dict['false_miss_probability'] if scenario_dict else default_scenario['false_miss_probability'] 
        # self.true_miss_prob = scenario_dict['true_miss_probability'] if scenario_dict else default_scenario['true_miss_probability'] 

        P = [[i, j] for i in range(self.grid_size) for j in range(self.grid_size)]
        P.append([-1, -1])
        self.D = distance.cdist(P, P) * self.cell_side_length


    def __str__(self) -> str:

        if self.comm_cell_range == 2*sqrt(2):
            # comm_cell_range = sp.sqrt(round(self.comm_cell_range**2))
            comm_cell_range = "sqrt(8)"
        else:
            comm_cell_range = self.comm_cell_range

        multi_line_scenario = f'''{self.model['Type']}_{self.model['Alg']}_{self.model['Exp']}_g_{self.grid_size}_a_{self.cell_side_length}_n_{self.number_of_drones}_
v_{self.max_drone_speed}_r_{comm_cell_range}_nvisits_{self.n_visits}'''

        lines = multi_line_scenario.splitlines()
        single_line_scenario = ''.join(lines)
        return single_line_scenario