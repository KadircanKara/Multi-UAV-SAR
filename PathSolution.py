
import numpy as np
from numpy.lib.function_base import average
from pymoo.core.problem import ElementwiseProblem
from scipy.spatial import distance
from typing import List, Dict
import itertools
from math import sin, cos, atan2, ceil
from scipy import io
# from scipy.stats import itemfreq
import subprocess
import time
import copy
import matplotlib.pyplot as plt # 1.20.3
from collections import deque

from PathOptimizationModel import moo_model_with_disconn, distance_soo_model
from PathInput import model

# from distance import *
# from Conumber_of_nodesectivity import *
# from Time import *

# from PathAnimation import PathAnimation

default_input_parameters = {
    'grid_size': 8,
    'A': 50,
    'number_of_drones': 8,
    'V': 2.5, # m/s
    'rc': 2,  # 2 cells
    'min_visits':2,
    'max_visits': 5,  # So that a cell is not visited more than this amount (Inumber_of_cellsorporate as a constraint)
    'Nt': 1,
    'p': 0.99,
    'q': 0.01,
    'Th': 0.9,
    'max_isolated_time': 0,
}

from PathInfo import *

def split_list(lst, val):
    return [list(group) for k,
    group in
            itertools.groupby(lst, lambda x: x == val) if not k]

class PathSolution():

    def __str__(self):
        info = self.info
        return f"Scenario: number_of_cells_{info.number_of_cells}_A_{info.cell_side_length}_number_of_drones_{info.number_of_drones}_V_{info.max_drone_speed}_rc_{info.comm_cell_range}_maxVisits_{info.min_visits}\n" \
               f"Objective Values: totaldistance_{self.total_distance}_longestSubtour_{self.longest_subtour}_percentageConumber_of_nodesectivity_{self.percentage_connectivity}\n" \
               f"Chromosome: pathSequenumber_of_cellse_{self.path}_startPoints_{self.start_points}"

    def __init__(self, path, start_points, info:PathInfo):

        # self.hovering = info.hovering
        # self.realtime_conumber_of_nodesectivity = info.realtime_conumber_of_nodesectivity

        # Inputs
        self.path = path
        self.start_points = start_points
        self.info: PathInfo = info

        # print(f'path: {self.path}')
        # print(f"start points: {self.start_points}")

        # cell - path
        self.drone_dict_generated = False
        self.drone_dict = dict()
        self.interpolated_path_matrix = None # Interpolated cell matrix (discrete, cell by cell)
        self.realtime_path_matrix = None # Interpolated cell matrix (seconumber_of_droness)


        # xy coordinate - path
        self.x_coords_dict = dict()
        self.y_coords_dict = dict()
        self.x_coords_list = [None]*info.number_of_drones
        self.y_coords_list = [None]*info.number_of_drones
        self.x_matrix = None
        self.y_matrix = None

        # Time
        self.time_slots = None  # Continuous
        self.drone_timeslots = None
        self.time_steps = None  # Discrete
        self.mission_time = 0
        self.drone_timesteps = None
        self.occ_grid = np.full((self.info.number_of_nodes,self.info.number_of_cells), 0.5) # Initialize occupanumber_of_cellsy grid for each node inumber_of_cellsluding BS
        self.cell_visit_steps = None
        self.tbv = None
        self.min_tbv = None
        # distance
        self.subtour_lengths = None
        self.total_distance = None
        self.longest_subtour = None
        self.shortest_subtour = None
        self.subtour_range = None
        self.long_jump_violations = None
        self.long_jump_violations_constr = None
        self.cells_per_drone_constr = None
        self.max_visits_constr = None
        self.cell_nvisits = None

        self.time_slots = None
        self.time_steps = None

        # Connectivity
        self.connectivity_matrix = None
        self.num_connected_drones_to_base = None
        self.disconnected_time_steps = None
        # self.connectivity_to_base_matrix = None
        self.percentage_connectivity = None
        self.total_disconnected_timesteps = None
        self.max_disconnected_timesteps = None
        self.max_maxDisconnectedTime = None
        self.mean_maxDisconnectedTime = None


        # Call PathSolution funumber_of_cellstions
        self.get_pathplan() # Calculates drone dict and path matrix (not interpolated, directly from the path sequenumber_of_cellse and start points)
        self.get_real_paths_vectorized()
        # print(self.x_matrix.shape, self.y_matrix.shape, self.mission_time)
        
        # self.get_real_time_path_matrix_with_hovering()
        if model != distance_soo_model:
            self.calculate_connectivity_matrix()
            self.calculate_connectivity_to_base_matrix()
            self.calculate_percentage_connectivity()
            self.calculate_disconnected_timesteps()
            self.calculate_cell_visit_steps()
        # self.calculate_disconumber_of_nodesected_timesteps() # NOTE COMPLETE YET
        # self.calculate_total_distance_and_longest_subtour()
        # self.calculate_distance_constraints()

        # self.calculate_time_between_visits()
        #
        # min_time_between_visits(self)


    def calculate_connectivity_to_base_matrix(self):

        info = self.info

        connectivity = self.connectivity_matrix

        time_slots = self.time_slots

        connectivity_to_base = np.zeros((time_slots, info.number_of_drones + 1))

        #path_sums = np.zeros(info.number_of_drones + 1)

        for time in range(time_slots):
            adj_mat = connectivity[time, :, :]

            connectivity_to_base[time, BFS(adj_mat, self)] = 1

            # print(f"Adj Mat:\n{adj_mat}\nConn to BS:\n{connectivity_to_base[time]}")

            """path_sums[:] = 0

            for pow in range(info.max_hops):
                path_sums += np.linalg.matrix_power(adj_mat, pow)[0, :]
            connectivity_to_base[time] = path_sums > 0
            """
        self.connectivity_to_base_matrix = connectivity_to_base

        return connectivity_to_base, time_slots


    def calculate_percentage_connectivity(self):

        # For PathOutput Only, NOT USED IN F

        connectivity_to_base_percentage = np.zeros(self.time_slots)

        for time in range(self.time_slots):
            connectivity_to_base_percentage[time] = sum(self.connectivity_to_base_matrix[time, 1:])/(self.info.number_of_drones)

        self.percentage_connectivity  = np.mean(connectivity_to_base_percentage)

        return self.percentage_connectivity


    def calculate_cell_visit_steps(self):
        info = self.info
        path_matrix = self.real_time_path_matrix[1:,:].transpose()
        cell_visit_steps = dict()
        for i in range(info.number_of_cells):
            cell_visit_steps[i] = np.where(path_matrix==i)[0] # Steps at which the cell is visited

        self.cell_visit_steps = cell_visit_steps


    def calculate_connectivity_matrix(self):

        info = self.info

        comm_dist = info.comm_cell_range * info.cell_side_length

        real_time_path_matrix = self.real_time_path_matrix # % info.number_of_cells

        # print(f"real time path matrix:\n{real_time_path_matrix}")

        time_slots = real_time_path_matrix.shape[1]

        connectivity = np.zeros((time_slots, info.number_of_drones+1, info.number_of_drones+1))

        for drone_no in range(real_time_path_matrix.shape[0]):
            drone_path = real_time_path_matrix[drone_no, :]
            for drone_no_2 in range(real_time_path_matrix.shape[0]):
                if drone_no != drone_no_2:
                    drone_path_2 = real_time_path_matrix[drone_no_2, :]
                    for time in range(time_slots):
                        dist = info.D[drone_path[time], drone_path_2[time]]
                        if dist <= comm_dist:
                            connectivity[time, drone_no, drone_no_2] = 1
                        # print(f"comm dist: {comm_dist}, dist btw nodes: {dist}, conn: {connectivity[time, drone_no, drone_no_2]}")

        # for drone_no in range(real_time_path_matrix.shape[0]):
        #     drone_path = real_time_path_matrix[drone_no, :]
        #     for drone_no_2 in range(real_time_path_matrix.shape[0]):
        #         drone_path_2 = real_time_path_matrix[drone_no_2, :]
        #         for time in range(time_slots):
        #             if drone_no != drone_no_2 and info.D[drone_path[time], drone_path_2[time]] <= comm_dist:
        #                 if drone_no == 0 and drone_no_2 > 0 and drone_path_2[time] != -1:
        #                     connectivity[time, drone_no, drone_no_2] = 1
        #                 elif drone_no > 0 and drone_no_2 == 0 and drone_path[time] != -1:
        #                     connectivity[time, drone_no, drone_no_2] = 1
        #                 elif drone_no > 0 and drone_no_2 > 0 and drone_path[time] != -1 and drone_path_2[time] != -1:
        #                     connectivity[time, drone_no, drone_no_2] = 1
        #                 else:
        #                     continue



        self.connectivity_matrix = connectivity
        self.time_slots = time_slots

        return connectivity, time_slots


    def calculate_disconnected_timesteps(self):

        # Finds the maximum disconnected timesteps for each drone

        info = self.info

        time_steps = self.real_time_path_matrix.shape[1]

        disconnected_timesteps_matrix = np.zeros((info.number_of_drones, self.connectivity_matrix.shape[0]), dtype=int)

        drone_total_disconnected_timesteps = np.zeros(info.number_of_drones, dtype=int)

        for i in range(info.number_of_drones):
            # print(disconnected_timesteps_matrix[i].shape, connected_nodes(sol,i + 1))
            disconnected_timesteps_matrix[i] = connected_nodes(self,i + 1)  # To account for skipping the base station # 0,1 , 1,2 ... 7,8
            drone_total_disconnected_timesteps[i] = len(np.where(disconnected_timesteps_matrix[i] == 0)[0])

        self.disconnected_time_steps = drone_total_disconnected_timesteps

        self.mean_disconnected_time = np.mean(self.disconnected_time_steps)
        self.max_disconnected_time = np.max(self.disconnected_time_steps)

        # print(f"Disconnected Time Steps: {sol.disconnected_time_steps}")

        return self.disconnected_time_steps


    def get_real_time_path_matrix_with_hovering(self):

        end = self.info.number_of_cells*self.info.min_visits
        start = self.start_points[-1]
        self.longest_path = end - start
        end = start

        for i in range(self.info.number_of_drones-2, -1, -1):
            start = self.start_points[i]
            current_path = end - start
            if current_path > self.longest_path:
                self.longest_path = current_path
            end = start

        self.real_time_path_matrix = np.zeros((self.info.number_of_nodes, self.longest_path), int) - 1


        for drone_no in range(self.info.number_of_drones):

            if drone_no < self.info.number_of_drones-1:
                end_point = self.start_points[drone_no+1]
            else:
                end_point = self.info.number_of_cells*self.info.min_visits

            city_counts = np.zeros(self.info.number_of_cells*self.info.min_visits, int)

            path_len = 0

            # print(self.start_points[drone_no], end_point)
            # print(self.path[self.start_points[drone_no]:end_point])

            sub_path = self.path[self.start_points[drone_no]:end_point]

            path_len += len(sub_path)

            city_counts[sub_path] = 1

            # print(f"sub_path: {sub_path}")

            # print("----------------------------------------------------------------------------------------------------------------------------------------------------------------")

            while not path_len == self.longest_path:
                # print(f"path_len: {path_len}, longest_path: {self.longest_path}")
                for city in sub_path:
                    city_counts[city] += 1
                    path_len += 1
                    if(path_len == self.longest_path):
                        break

            i = 0
            for city in sub_path:
                for hover in range(city_counts[city]):
                    self.real_time_path_matrix[drone_no + 1, i] = city
                    i += 1
        bs_path_component = np.zeros((self.real_time_path_matrix.shape[0],1), dtype=int)-1
        self.real_time_path_matrix = np.hstack((bs_path_component, self.real_time_path_matrix, bs_path_component), dtype=int)

        # print("Real Time Path Matrix:\n",self.real_time_path_matrix)

        self.time_steps = self.real_time_path_matrix.shape[1]

        # print("path matrix with hovering:")
        # print(self.real_time_path_matrix % self.info.number_of_cells)


    def get_pathplan(self):

        self.drone_dict_generated = True
        self.drone_dict = dict()
        self.time_steps = 0
        info = self.info

        # GET CELL DICT
        for i in range(info.number_of_drones):
            if i < info.number_of_drones - 1:
                # self.drone_dict[i] = np.hstack(( np.array([-1,0]), self.path[self.start_points[i]:self.start_points[i + 1]], np.array([0,-1])))
                self.drone_dict[i] = np.hstack((np.array([-1]), (np.array(self.path) % info.number_of_cells)[self.start_points[i]:self.start_points[i + 1]], np.array([-1])))
            else:
                # self.drone_dict[i] = np.hstack(( np.array([-1,0]), self.path[self.start_points[i]:], np.array([0,-1])))
                self.drone_dict[i] = np.hstack((np.array([-1]), (np.array(self.path) % info.number_of_cells)[self.start_points[i]:], np.array([-1])))

            # Set longest "discrete" subtour
            if len(self.drone_dict[i]) > self.time_steps : self.time_steps = len(self.drone_dict[i]) # Set max subtour length

            # Add BS as a node to drone_dict (key=1)
            self.drone_dict[-1] = np.array([-1] * self.time_steps)

        # print(f"DRONE DICT: {self.drone_dict}")


        # GET CELL MATRIX
        self.path_matrix = np.zeros((info.number_of_drones+1, self.time_steps), dtype=int) - 1 # number_of_drones+1 bc. of BS (inumber_of_dronesex 0)
        for i in range(info.number_of_drones):
            if len(self.drone_dict[i]) == self.time_steps: # If this is the longest discrete tour drone
                self.path_matrix[i+1] = self.drone_dict[i]
            else : # If this is NOT the longest discrete tour drone
                len_diff = self.time_steps - len(self.drone_dict[i])
                filler = np.array([-1]*len_diff)
                self.path_matrix[i+1] = np.hstack( (self.drone_dict[i] , filler)  )

        self.real_time_path_matrix = self.path_matrix

        # Set Total Distance and Longest Subtour
        Nd, time_steps = self.real_time_path_matrix.shape
        Nd -= 1 # Remove base station

        subtour_lengths = dict()

        for i in range(info.number_of_drones):
            drone_path = self.real_time_path_matrix[i+1]
            drone_dist = 0
            for j in range(time_steps-1):
                drone_dist += info.D[drone_path[j],drone_path[j+1]]
            subtour_lengths[i] = drone_dist

        self.total_distance = sum(subtour_lengths.values())
        self.longest_subtour = max(subtour_lengths.values())

        # APPLY HOVERING TO DRONES WITH SHORTER PATHS (ONLY IF MOO)

        if model == moo_model_with_disconn:

            drone_dict = {k:v for k,v in self.drone_dict.items() if k!=-1}
            # print(f"Dict Values Object: {list(drone_dict.items())}")
            path_lens = [len(path) for path in list(drone_dict.values())]
            # hovering_drone_ids = [path_lens.index(i) for i in path_lens if i != max(path_lens)]
            # hovering_drone_ids = [i for i in range(info.number_of_drones) if i != path_lens.index(max(path_lens))]
            # Get Hovering Drones
            hovering_drone_ids = []
            shift = 0
            path_lens_temp = path_lens.copy()
            while len(path_lens_temp) > 0:
                if path_lens_temp[0] != max(path_lens):
                    hovering_drone_ids.append(shift)
                shift += 1
                path_lens_temp.pop(0)
            # print(f"Path Lens: {path_lens}")
            # print(f"Hovering Drone IDs: {hovering_drone_ids}")
            for drone in hovering_drone_ids:
                # print("----------------------------------------------------------")
                # print(f"Drone {drone}:")
                # print("----------------------------------------------------------")
                # APPLY HOVERING
                path_without_hovering = self.real_time_path_matrix[drone+1].copy()
                # Find last cell
                hovering_cell_idx = np.where(path_without_hovering==-1)[0][1] - 1
                hovering_cell = path_without_hovering[hovering_cell_idx]
                # print(f"Hovering Cell Idx: {hovering_cell_idx}, Hovering Cell: {hovering_cell}")
                hovering_component = np.array([hovering_cell] * (len(path_without_hovering) - hovering_cell_idx - 1))
                # print(f"Hovering Component: {hovering_component}")
                path_with_hovering = path_without_hovering.copy()
                path_with_hovering[hovering_cell_idx:len(path_without_hovering)-1] = hovering_component
                self.real_time_path_matrix[drone+1] = path_with_hovering


    def get_real_paths_vectorized(self):

        sync = True if self.info.model!=distance_soo_model else False
        info = self.info
        drone_path_matrix = self.real_time_path_matrix[1:,:]
        time_steps = drone_path_matrix.shape[1]
        # path_matrix = np.where(self.real_time_path_matrix != -1, self.real_time_path_matrix % info.number_of_cells, self.real_time_path_matrix)
        print(f"path matrix:\n{drone_path_matrix}")




    def get_real_paths(self):

        sync = True if self.info.model!=distance_soo_model else False

        info = self.info

        time_steps = self.real_time_path_matrix.shape[1]

        # path_matrix = self.real_time_path_matrix % info.number_of_cells

        path_matrix = np.where(self.real_time_path_matrix != -1, self.real_time_path_matrix % info.number_of_cells, self.real_time_path_matrix)
        # path_matrix = self.real_time_path_matrix

        # print("Original Path Matrix:",self.real_time_path_matrix)
        # print("Path Matrix:",path_matrix)

        # mission_time = 0

        for i in range(1, time_steps):
            current_step_cells , next_step_cells = path_matrix[1:,i-1].tolist() , path_matrix[1:,i].tolist()
            # Calculate Drone Speeds Based On Distance
            drone_dists = np.array([info.D[current_step_cells[j],next_step_cells[j]] for j in range(info.number_of_drones)])# Calculate Distance for Each Drone
            max_dist = max(drone_dists)
            step_time = max_dist / info.max_drone_speed
            # mission_time += step_time
            # print("-->",drone_dists, step_time)
            drone_speeds = drone_dists / step_time if sync else [info.max_drone_speed]*len(drone_dists)
            # print("->",drone_speeds)
            # print(f"Drone Dists: {drone_dists}\nStep Time: {step_time}\nDrone Speeds: {drone_speeds}")
            current_step_coords = list(map(self.get_coords, current_step_cells))
            next_step_coords = list(map(self.get_coords, next_step_cells))
            coord_diffs = [next_step_coords[j] - current_step_coords[j] for j in range(info.number_of_drones)]
            thetas = [atan2(j[1],j[0]) for j in coord_diffs]
            # Changes in current_to_next_step !!!
            if sync:
                current_to_next_step_x_coords = [ np.arange(current_step_coords[j][0], next_step_coords[j][0], drone_speeds[j] * cos(thetas[j])) if current_step_coords[j][0] != next_step_coords[j][0] else np.array([current_step_coords[j][0]]*ceil(step_time)) for j in range(info.number_of_drones) ]
                current_to_next_step_y_coords = [ np.arange(current_step_coords[j][1], next_step_coords[j][1], drone_speeds[j] * sin(thetas[j])) if current_step_coords[j][1] != next_step_coords[j][1] else np.array([current_step_coords[j][1]]*ceil(step_time)) for j in range(info.number_of_drones) ]
            else:
                current_to_next_step_x_coords = [ np.arange(current_step_coords[j][0], next_step_coords[j][0], drone_speeds[j] * cos(thetas[j])) if current_step_coords[j][0] != next_step_coords[j][0] else np.array([current_step_coords[j][0]]*2) for j in range(info.number_of_drones) ]
                current_to_next_step_y_coords = [ np.arange(current_step_coords[j][1], next_step_coords[j][1], drone_speeds[j] * sin(thetas[j])) if current_step_coords[j][1] != next_step_coords[j][1] else np.array([current_step_coords[j][1]]*2) for j in range(info.number_of_drones) ]

            # if i < 10:
            #     print(f"Step {i}")
            #     print(f"current step cells: {current_step_cells}, next step cells: {next_step_cells}")
            #     print(f"current_to_next_step_x_coords: {current_to_next_step_x_coords}, current_to_next_step_y_coords: {current_to_next_step_y_coords}")

            for j in range(info.number_of_drones):
                x_coords, y_coords = current_to_next_step_x_coords[j], current_to_next_step_y_coords[j]
                if len(x_coords) != len(y_coords):
                    xy_diff = abs(len(x_coords) - len(y_coords))
                    if len(x_coords) > len(y_coords): # Fill y
                        current_to_next_step_y_coords[j] = np.hstack((current_to_next_step_y_coords[j], np.array([y_coords[-1]]*xy_diff)))
                    else: # Fill x
                        current_to_next_step_x_coords[j] = np.hstack((current_to_next_step_x_coords[j], np.array([x_coords[-1]]*xy_diff)))
                else:
                    continue

            # if i==1:
            #     print(f"X - Current to Next Step: {current_to_next_step_x_coords}\nY - Current to Next Step: {current_to_next_step_y_coords}")

            self.x_coords_list = [current_to_next_step_x_coords[j] if i==1 else np.hstack((self.x_coords_list[j],current_to_next_step_x_coords[j])) for j in range(info.number_of_drones)]
            self.y_coords_list = [current_to_next_step_y_coords[j] if i==1 else np.hstack((self.y_coords_list[j],current_to_next_step_y_coords[j])) for j in range(info.number_of_drones)]

        # self.mission_time = mission_time if sync else self.longest_subtour/info.max_drone_speed
        self.drone_timeslots = [len(x) for x in self.x_coords_list]
        self.time_slots = max(self.drone_timeslots)

        # Initialize xy matrix
        x_sink,y_sink = self.get_coords(-1)
        self.x_matrix = np.full((info.number_of_drones + 1, self.time_slots), x_sink)  # Nd+1 rows in order to incorporate base station
        self.y_matrix = self.x_matrix.copy()
        # self.realtime_real_time_path_matrix = self.x_matrix.copy()
        # self.realtime_real_time_path_matrix.astype(int)
        # self.realtime_real_time_path_matrix[:, :] = -1
        # interpolated_path_dict = dict()
        # interpolated_path_max_len = 0

        # print(f"path matrix: {self.real_time_path_matrix}")

        # print(f"x_coords_list: {self.x_coords_list}\ny_coords_list: {self.y_coords_list}")

        for i in range(info.number_of_drones):
            self.x_matrix[i + 1] = np.hstack((self.x_coords_list[i], np.array([x_sink] * (self.time_slots - self.drone_timeslots[i]))))
            self.y_matrix[i + 1] = np.hstack((self.y_coords_list[i], np.array([y_sink] * (self.time_slots - self.drone_timeslots[i]))))


        self.mission_time = self.x_matrix.shape[1]

        return self.x_matrix, self.y_matrix




    def get_coords(self, cell):

        if cell == -1:
            x = -self.info.cell_side_length / 2
            y = -self.info.cell_side_length / 2
        else:
            # x = ((cell % n) % self.info.grid_size + 0.5) * self.info.cell_len
            x = (cell % self.info.grid_size + 0.5) * self.info.cell_side_length
            # y = ((cell % n) // self.info.grid_size + 0.5) * self.info.cell_len
            y = (cell // self.info.grid_size + 0.5) * self.info.cell_side_length
        # return [x,y]
        return np.array([x, y])


    def get_city(self, coords):

        if coords[0] < 0 and coords[1] < 0:
            return -1
        else:
            x, y = coords
            return floor(y / self.info.cell_side_length) * self.info.grid_size + floor(x / self.info.cell_side_length)


    def get_x_coords(self, cell):

        if cell == -1:
            x = -self.info.cell_side_length / 2
        else:
            # x = ((cell % n) % self.info.grid_size + 0.5) * self.info.cell_len
            x = (cell % self.info.grid_size + 0.5) * self.info.cell_side_length
        return x


    def get_y_coords(self, cell):

        if cell == -1:
            y = -self.info.cell_side_length / 2
        else:
            # y = ((cell % n) // self.info.grid_size + 0.5) * self.info.cell_len
            y = (cell // self.info.grid_size + 0.5) * self.info.cell_side_length
        return y
    
def get_path_coords(current_x, current_y, next_x, next_y, speed, theta, num_points):
    """
    Compute the real-time coordinates between two points (current_x, current_y) and (next_x, next_y)
    given the speed and angle of movement.
    """
    if current_x != next_x:
        x_coords = np.linspace(current_x, next_x, num_points)
    else:
        x_coords = np.full(num_points, current_x)
    
    if current_y != next_y:
        y_coords = np.linspace(current_y, next_y, num_points)
    else:
        y_coords = np.full(num_points, current_y)

    return x_coords, y_coords


def connected_nodes(sol:PathSolution, start_node):

    # start node: The node that we calculate connectivity to

    info = sol.info
    num_nodes = info.number_of_nodes

    num_connected_drones = np.zeros(sol.connectivity_matrix.shape[0], dtype=int)

    for i in range(sol.connectivity_matrix.shape[0]):

        connectivity_matrix = sol.connectivity_matrix[i]

        # num_nodes = len(connectivity_matrix)
        visited = [False] * num_nodes
        queue = deque([start_node])
        connected_count = 0

        # print(f"visited: {visited}")
        # print(f"start node: {start_node}")

        visited[start_node] = True  # Mark start node as visited

        while queue:
            node = queue.popleft()
            connected_count += 1

            for j in range(num_nodes):
                if connectivity_matrix[node][j] != 0 and not visited[j]:
                    queue.append(j)
                    visited[j] = True  # Mark the connected node as visited

        num_connected_drones[i] = connected_count - 1

        # print("---------------------------------------------------------")
        # print(f"step {i}")
        # print("---------------------------------------------------------")
        # print(f"Connectivity Matrix:\n{connectivity_matrix}")
        # print(f"Number of nodes connected to node {start_node}: {num_connected_drones[i]}")

    return num_connected_drones


def BFS(adj, sol:PathSolution):

    v = sol.info.number_of_nodes

    ctb = []
    start = 0
    # Visited vector to so that a
    # vertex is not visited more than
    # once Initializing the vector to
    # false as no vertex is visited at
    # the beginning
    visited = [False] * (sol.info.number_of_nodes)
    q = [start]

    # Set source as visited
    visited[start] = True

    while q:
        vis = q[0]

        # Print current node
        ctb.append(vis)

        q.pop(0)

        # For every adjacent vertex to
        # the current vertex
        for i in range(v):
            if (adj[vis][i] == 1 and
                  (not visited[i])):

                # Push the adjacent node
                # in the queue
                q.append(i)

                # set
                visited[i] = True

    return ctb



'''    def get_real_paths(self):
        sync = self.info.model != distance_soo_model

        info = self.info
        time_steps = self.real_time_path_matrix.shape[1]

        path_matrix = np.where(self.real_time_path_matrix != -1, self.real_time_path_matrix % info.number_of_cells, self.real_time_path_matrix)

        mission_time = 0

        # Preallocate coordinate lists
        self.x_coords_list = [[] for _ in range(info.number_of_drones)]
        self.y_coords_list = [[] for _ in range(info.number_of_drones)]

        # Extract coordinates
        coords = np.array([self.get_coords(i) for i in range(info.number_of_cells)])

        for i in range(1, time_steps):
            current_step_cells, next_step_cells = path_matrix[1:, i-1], path_matrix[1:, i]

            # Calculate distances and times
            current_step_coords = coords[current_step_cells]
            next_step_coords = coords[next_step_cells]
            coord_diffs = next_step_coords - current_step_coords
            thetas = np.arctan2(coord_diffs[:, 1], coord_diffs[:, 0])

            drone_dists = np.linalg.norm(coord_diffs, axis=1)
            max_dist = np.max(drone_dists)
            step_time = max_dist / info.max_drone_speed
            mission_time += step_time

            drone_speeds = drone_dists / step_time if sync else np.full(info.number_of_drones, info.max_drone_speed)

            for j in range(info.number_of_drones):
                current_x, current_y = current_step_coords[j]
                next_x, next_y = next_step_coords[j]
                speed = drone_speeds[j]
                theta = thetas[j]

                num_points = ceil(step_time) if sync else 2
                x_coords, y_coords = get_path_coords(current_x, current_y, next_x, next_y, speed, theta, num_points)
                
                self.x_coords_list[j].extend(x_coords)
                self.y_coords_list[j].extend(y_coords)

        self.mission_time = mission_time if sync else self.longest_subtour / info.max_drone_speed
        self.drone_timeslots = [len(x) for x in self.x_coords_list]
        self.time_slots = max(self.drone_timeslots)

        # Initialize xy matrix
        x_sink, y_sink = self.get_coords(-1)
        self.x_matrix = np.full((info.number_of_drones + 1, self.time_slots), x_sink)
        self.y_matrix = np.full((info.number_of_drones + 1, self.time_slots), y_sink)

        for i in range(info.number_of_drones):
            drone_time = self.drone_timeslots[i]
            self.x_matrix[i + 1, :drone_time] = self.x_coords_list[i]
            self.y_matrix[i + 1, :drone_time] = self.y_coords_list[i]

        return self.x_matrix, self.y_matrix
'''








'''
info = PathInfo(min_visits=5)
path = np.random.permutation(info.min_visits * info.number_of_cells).tolist()
# Random start points
start_points = sorted(random.sample([i for i in range(1, len(path))], info.number_of_drones - 1))
start_points.insert(0, 0)
sol = PathSolution(path, start_points, info)


info = PathInfo(number_of_drones=4, grid_size=8, min_visits=3)
# path = np.random.permutation(range(1,info.number_of_cells))
path = np.random.permutation(info.number_of_cells)
start_points = [0,16,32,48]
start_points = [0,12,22,30]
t = time.time()
sol = PathSolution(path, start_points, info)

print(f"time between visits:\n{sol.time_between_visits}")
print(f"cell visit steps:\n{sol.cell_nvisits}")
print("Hovering and realtime conumber_of_nodesectivity:", time.time()-t)

t = time.time()
sol = PathSolution(path, start_points, info, hovering=True, realtime_conumber_of_nodesectivity=False)
print("Hovering and discrete conumber_of_nodesectivity:", time.time()-t)
t = time.time()
sol = PathSolution(path, start_points, info, hovering=False, realtime_conumber_of_nodesectivity=False)
print("No Hovering and realtime conumber_of_nodesectivity:", time.time()-t)
t = time.time()
sol = PathSolution(path, start_points, info, hovering=False, realtime_conumber_of_nodesectivity=False)
print("No Hovering and discrete conumber_of_nodesectivity:", time.time()-t)


animation = PathAnimation(sol.x_matrix, sol.y_matrix, sol.info)

print("path sequenumber_of_cellse:", path)
print("start points:", start_points)
print("drone dict:", sol.drone_dict)
print("path matrix:", sol.path_matrix)
print("longest discrete subtour:", sol.longest_discrete_subtour)
df = pd.DataFrame(sol.interpolated_realtime_path_matrix)
print(f"intp realtime path matrix:\n{df.to_string(inumber_of_dronesex=False)}")

df = pd.DataFrame(sol.interpolated_path_matrix)
print(f"intp path matrix:\n{df.to_string(inumber_of_dronesex=False)}")
'''
