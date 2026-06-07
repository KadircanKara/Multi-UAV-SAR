
import numpy as np
# from numpy.lib.function_base import average
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
import multiprocessing as mp
from multiprocessing import Pool, cpu_count

from PathInfo import *
from PathOptimizationModel import *
# from PathInput import model

# from PathRepair import *

# from distance import *
# from Conumber_of_nodesectivity import *
# from Time import *

# from PathAnimation import PathAnimation

def sign(x):
    return 1 if x > 0 else -1 if x < 0 else 0  # Returns 0 if x is exactly 0

def split_list(lst, val):
    return [list(group) for k,
    group in
            itertools.groupby(lst, lambda x: x == val) if not k]


# if __name__ == '__main__' :

class PathSolution():

    def __str__(self):
        info = self.info
        return f"Scenario: number_of_cells_{info.number_of_cells}_A_{info.cell_side_length}_number_of_drones_{info.number_of_drones}_V_{info.max_drone_speed}_rc_{info.comm_cell_range}_maxVisits_{info.n_visits}\n" \
            f"Objective Values: totaldistance_{self.total_distance}_longestSubtour_{self.longest_subtour}_percentageConumber_of_nodesectivity_{self.percentage_connectivity}\n" \
            f"Chromosome: pathSequenumber_of_cellse_{self.path}_startPoints_{self.start_points}"

    def __init__(self, path, start_points,  info:PathInfo, calculate_pathplan=False, calculate_tbv=False, calculate_connectivity=False, calculate_disconnectivity=False):

        self.calculate_tbv = calculate_tbv
        self.calculate_connectivity = calculate_connectivity
        self.calculate_disconnectivity = calculate_disconnectivity

        # self.hovering = info.hovering
        # self.realtime_conumber_of_nodesectivity = info.realtime_conumber_of_nodesectivity

        # Inputs
        self.path = path
        self.start_points = start_points
        # self.relay_positions = relay_positions
        self.info: PathInfo = info

        # print(f'path: {self.path}')
        # print(f"start points: {self.start_points}")

        # cell - path
        # self.drone_dict = dict()

        # Time
        self.time_slots = None
        self.mission_time = None
        self.time_elapsed_at_steps = None
        self.visit_times = None
        self.tbv = None
        self.mean_tbv = None
        self.max_mean_tbv = 0
        # Distance
        self.subtour_lengths = None
        self.total_distance = None
        self.longest_subtour = None
        self.shortest_subtour = None
        self.subtour_range = None
        self.drone_speed_violations = None
        self.path_speed_violations = None
        self.speed_violations = None

        # Connectivity
        self.connectivity_matrix = None
        self.disconnected_time_steps = None
        self.percentage_connectivity = None

        # Smoothness
        self.drone_path_smoothness_penalties = None
        self.drone_tracebacks = None

        if calculate_pathplan:
            self.get_drone_dict()
            self.get_pathplan() # Calculates drone dict and path matrix (not interpolated, directly from the path sequenumber_of_cellse and start points)

        if calculate_tbv and info.n_visits>1:
            self.get_visit_times()
            self.get_tbv()
            self.get_mean_tbv()


        if calculate_connectivity:
            # print("connectivity calculations")
            self.do_connectivity_calculations()
        if calculate_disconnectivity:
            # print("disconnectivity calculations")
            self.do_disconnectivity_calculations()


    def get_drone_dict(self):

        self.drone_dict = dict()
        self.time_slots= 0
        info = self.info

        # GET CELL DICT
        for i in range(info.number_of_drones):
            if i < info.number_of_drones - 1:
                drone_path = (np.array(self.path) % info.number_of_cells)[self.start_points[i]:self.start_points[i + 1]]
            else:
                drone_path = (np.array(self.path) % info.number_of_cells)[self.start_points[i]:]
            # print(f"drone {i} path: {drone_path}")
            interpolated_first_step = interpolate_between_cities(self, -1, drone_path[0])[:-1]
            interpolated_last_step = interpolate_between_cities(self, drone_path[-1], 0)[1:]
            interpolated_last_step.append(-1)
            # self.drone_dict[i] = np.hstack(( np.array([-1,0]), self.path[self.start_points[i]:self.start_points[i + 1]], np.array([0,-1])))
            if "Percentage Connectivity" in info.model["F"]:
                self.drone_dict[i] = np.hstack((interpolated_first_step, drone_path, np.array([-1])))
            else:
                self.drone_dict[i] = np.hstack((interpolated_first_step, drone_path, interpolated_last_step))
            # print(f"city prev: {drone_path[-1]}, city: -1, interpolated last step: {interpolated_last_step}")

            # Set longest "discrete" subtour
            if len(self.drone_dict[i]) > self.time_slots : self.time_slots = len(self.drone_dict[i]) # Set max subtour length

            # Add BS as a node to drone_dict (key=1)
        # self.drone_dict[-1] = np.array([-1] * self.time_steps)


    def get_pathplan(self):

        info = self.info

        # GET CELL MATRIX
        self.path_matrix = np.zeros((info.number_of_drones+1, self.time_slots), dtype=int) - 1
        for i in range(info.number_of_drones):
            if len(self.drone_dict[i]) == self.time_slots: # If this is the longest discrete tour drone
                self.path_matrix[i+1] = self.drone_dict[i]
            else : # If this is NOT the longest discrete tour drone
                len_diff = self.time_slots - len(self.drone_dict[i])
                filler = np.array([-1]*len_diff)
                self.path_matrix[i+1] = np.hstack( (self.drone_dict[i] , filler)  )

        self.real_time_path_matrix = self.path_matrix

        # Set Total Distance and Longest Subtour
        Nd, time_steps = self.real_time_path_matrix.shape
        Nd -= 1 # Remove base station

        self.subtour_lengths = []

        for i in range(info.number_of_drones):
            drone_path = self.real_time_path_matrix[i+1]
            drone_dist = 0
            for j in range(time_steps-1):
                drone_dist += info.D[drone_path[j],drone_path[j+1]]
            self.subtour_lengths.append(drone_dist)

        self.total_distance = sum(self.subtour_lengths)
        self.longest_subtour = max(self.subtour_lengths)

        # APPLY HOVERING TO DRONES WITH SHORTER PATHS
        path_lens = [len(path) for path in list(self.drone_dict.values())]
        # Get Hovering Drones
        hovering_drone_ids = []
        shift = 0
        path_lens_temp = path_lens.copy()
        while len(path_lens_temp) > 0:
            if path_lens_temp[0] != max(path_lens):
                hovering_drone_ids.append(shift)
            shift += 1
            path_lens_temp.pop(0)
        self.hovering_drones = hovering_drone_ids
        for drone in hovering_drone_ids:
            # APPLY HOVERING
            path_without_hovering = self.real_time_path_matrix[drone+1].copy()
            hovering_cell_idx = np.where(path_without_hovering==-1)[0][1] - 1
            hovering_cell = path_without_hovering[hovering_cell_idx]
            hovering_component = np.array([hovering_cell] * (len(path_without_hovering) - hovering_cell_idx - 1))
            path_with_hovering = path_without_hovering.copy()
            path_with_hovering[hovering_cell_idx:len(path_without_hovering)-1] = hovering_component
            self.real_time_path_matrix[drone+1] = path_with_hovering
        # INTERPOLATE PATH BACK TO BS AFTER HOVERING
        drone_interpolated_last_step_list = []
        for drone in range(info.number_of_drones):
            drone_interpolated_last_step = interpolate_between_cities(self, self.real_time_path_matrix[drone+1][-2], 0)
            if self.real_time_path_matrix[drone+1][-2] != 0:
                drone_interpolated_last_step = drone_interpolated_last_step[1:]
            drone_interpolated_last_step_list.append(drone_interpolated_last_step)
        max_interpolated_last_step_len = max([len(x) for x in drone_interpolated_last_step_list])

        for drone in range(info.number_of_drones):
            if len(drone_interpolated_last_step_list[drone]) < max_interpolated_last_step_len:
                drone_interpolated_last_step_list[drone].extend([drone_interpolated_last_step_list[drone][-1]] * (max_interpolated_last_step_len - len(drone_interpolated_last_step_list[drone])))
        drone_interpolated_path_array = np.insert(np.array(drone_interpolated_last_step_list), 0, np.full((1,max_interpolated_last_step_len), -1, dtype=int), axis=0)
        self.real_time_path_matrix = np.hstack((self.real_time_path_matrix[:,:-1], drone_interpolated_path_array, np.full((info.number_of_nodes,1), -1, dtype=int)))             

        # Calculate Mission Time
        mission_time = 0
        time_elapsed_at_steps = []
        real_time_drone_path_matrix = self.real_time_path_matrix[1:,:].T
        for i in range(real_time_drone_path_matrix.shape[0]-1):
            drone_step_dists=[]
            # diagonal_path_exists=False
            drone_positions = real_time_drone_path_matrix[i]
            next_drone_positions = real_time_drone_path_matrix[i+1]
            for drone_no in range(len(drone_positions)):
                drone_step_dists.append(info.D[drone_positions[drone_no], next_drone_positions[drone_no]])
            max_dist_at_step = max(drone_step_dists)
            time_elapsed = max_dist_at_step / info.max_drone_speed
            mission_time += time_elapsed
            time_elapsed_at_steps.append(time_elapsed)
            #     if info.D[drone_positions[drone_no], next_drone_positions[drone_no]] > info.cell_side_length:
            #         diagonal_path_exists=True
            #         break
            # time_elapsed = info.cell_side_length*sqrt(2) / info.max_drone_speed if diagonal_path_exists else info.cell_side_length / info.max_drone_speed
            # mission_time += time_elapsed
            # time_elapsed_at_steps.append(time_elapsed)

        self.mission_time = mission_time
        self.time_elapsed_at_steps = time_elapsed_at_steps

        # print("Final Path Matrix:\n", self.real_time_path_matrix)


        self.time_slots = self.real_time_path_matrix.shape[1]


    def get_visit_times(self):
        info = self.info
        final_search_step = max([len(q) - 2 for q in list(self.drone_dict.values())])
        drone_path_matrix = self.real_time_path_matrix[1:,:final_search_step+1]
        visit_times = [[] for _ in range(info.number_of_cells)]
        for cell in range(info.number_of_cells):
            # visit_times[cell] = np.sort(np.where(drone_path_matrix==cell)[1])[:info.n_visits] # Last bit is to exclude hovering steps
            visit_times[cell] = np.sort(np.where(drone_path_matrix==cell)[1])

        self.visit_times = visit_times

        return visit_times
    

    def get_tbv(self):

        debug_mode = False

        real_time_tbv = []

        # Calculate REAL-TIME TBV instead of timestep TBV
        for cell_visit_steps in self.visit_times:
            print("cell visit steps:", cell_visit_steps) if debug_mode else None
            real_time_tbv.append([]) # Initialize the list for the cell
            for step in range(len(cell_visit_steps)-1):
                current_step = cell_visit_steps[step]
                next_step = cell_visit_steps[step+1]
                real_time_tbv[-1].append(sum(self.time_elapsed_at_steps[current_step:next_step+1]))
            print("real time tbv:", real_time_tbv[-1]) if debug_mode else None
        tbv = [np.diff(x) for x in self.visit_times]
        self.tbv = tbv
        self.real_time_tbv = real_time_tbv

        return real_time_tbv
    

    def get_mean_tbv(self):
        mean_tbv = list(map(lambda x: np.mean(x), self.real_time_tbv))
        self.mean_tbv = mean_tbv
        self.max_mean_tbv = max(self.mean_tbv)
        return self.mean_tbv

    
    def do_connectivity_calculations(self):

        info = self.info
        comm_dist = info.comm_cell_range * info.cell_side_length
        real_time_path_matrix = self.real_time_path_matrix
        time_slots = real_time_path_matrix.shape[1]
        
        connectivity_matrix = np.zeros((time_slots, info.number_of_nodes, info.number_of_nodes))
        connectivity_to_base_matrix = np.zeros((time_slots, info.number_of_nodes))
        connectivity_to_base_percentage = np.zeros(time_slots)
        
        # Create a distance matrix for all drones at all times
        for time in range(time_slots):
            paths_at_time = real_time_path_matrix[:, time]
            for node_no in range(info.number_of_nodes):
                node_pos = paths_at_time[node_no]
                # Calculate distances from current node to all other nodes
                distances = info.D[node_pos, paths_at_time]
                # Create the connectivity matrix row for this node at this time
                connectivity_matrix[time, node_no, :] = distances <= comm_dist
                connectivity_matrix[time, node_no, node_no] = 0  # No self-connection

            adj_mat = connectivity_matrix[time]
            connectivity_to_base_matrix[time, BFS(adj_mat, self)] = 1
            connectivity_to_base_percentage[time] = np.mean(connectivity_to_base_matrix[time, 1:])
        
        self.connectivity_matrix = connectivity_matrix
        self.connectivity_to_base_matrix = connectivity_to_base_matrix
        self.percentage_connectivity = np.mean(connectivity_to_base_percentage)

        return self.percentage_connectivity


    def do_disconnectivity_calculations(self):

        num_disconnected_nodes_array = np.zeros(self.time_slots)
        drone_disconnected_times = np.zeros(self.info.number_of_nodes)

        for time in range(self.time_slots):

            adj_mat = self.connectivity_matrix[time] # nxn array (n = number of nodes)

            # Find disconnected nodes
            disconnected_rows = np.all(adj_mat == 0, axis=1)
            # Get the indices of disconnected nodes
            disconnected_drones = np.where(disconnected_rows)[0]
            for drone in disconnected_drones:
                drone_disconnected_times[drone] += 1
            num_disconnected_nodes = len(np.where(disconnected_rows)[0])
            # Update disconnected node array
            num_disconnected_nodes_array[time] = num_disconnected_nodes

            # if disconnected_drones.any():
            #     print(f"disconnected drones: {disconnected_drones}\nadj mat:\n{adj_mat}")

        self.mean_disconnected_time = np.mean(drone_disconnected_times)
        self.max_disconnected_time = np.max(drone_disconnected_times)
        self.total_disconnected_time = np.sum(drone_disconnected_times)
        self.percentage_disconnectivity = np.sum(num_disconnected_nodes_array) / (self.time_slots * self.info.number_of_nodes)


        # print(f"adj mat:\n{adj_mat}disconnected rows:\n{disconnected_rows}")

        return self.percentage_disconnectivity


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
    

def dfs(connectivity_matrix, node, visited, component):
    visited[node] = True
    component.append(node)
    for neighbor, connected in enumerate(connectivity_matrix[node]):
        if connected == 1 and not visited[neighbor]:
            dfs(connectivity_matrix, neighbor, visited, component)


def connected_components(connectivity_matrix):
    n = len(connectivity_matrix)
    visited = [False] * n
    components = []

    for node in range(n):
        if not visited[node]:
            component = []
            dfs(connectivity_matrix, node, visited, component)
            components.append(component)

    return components


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


def interpolate_between_cities(sol:PathSolution, city_prev, city):

    interpolated_path = [city_prev]

    info = sol.info
    coords_prev = sol.get_coords(city_prev)
    coords = sol.get_coords(city)
    coords_delta = coords - coords_prev
    axis_inc = np.array([sign(coords_delta[0]), sign(coords_delta[1])])

    num_mid_cities = int(max(abs(coords_delta))/info.cell_side_length)

    coords_temp = coords_prev.copy()

    for _ in range(num_mid_cities):
        if coords_temp[0] != coords[0]:
            coords_temp[0] += info.cell_side_length * axis_inc[0]
        if coords_temp[1] != coords[1]:
            coords_temp[1] += info.cell_side_length * axis_inc[1]
        mid_city = sol.get_city(coords_temp)
        # print(f"Iteration {_+1} coords: {coords_temp}, corresponding city: {mid_city}")
        interpolated_path.append(mid_city)

    # interpolated_path.pop(-1)

    # print(f"city prev: {city_prev}, city: {city}, mid cities: {interpolated_path}")
    
    return interpolated_path


def produce_n_tour_sol(sol:PathSolution, n_tours:int):
    sol_copy = copy.deepcopy(sol)
    drone_dict = sol_copy.drone_dict
    sol_copy.time_slots = 0
    for i in range(sol_copy.info.number_of_drones):
        drone_path = drone_dict[i][1:-1]
        drone_dict[i] = np.hstack((-1, np.tile(drone_path, n_tours), -1))
        if len(sol_copy.drone_dict[i]) > sol_copy.time_slots : sol_copy.time_slots = len(sol_copy.drone_dict[i])

    # print(drone_dict)

    sol_copy.get_pathplan()

    print(f"Original Sol Timesteps: {sol.real_time_path_matrix.shape[1]} | {n_tours} Tours Sol Timesteps: {sol_copy.real_time_path_matrix.shape[1]}")

    # if sol_copy.calculate_tbv and sol_copy.info.n_visits>1:
    sol_copy.get_visit_times()
    sol_copy.get_tbv()
    sol_copy.get_mean_tbv()

    if sol_copy.calculate_connectivity:
        # print("connectivity calculations")
        sol_copy.do_connectivity_calculations()
    if sol_copy.calculate_disconnectivity:
        # print("disconnectivity calculations")
        sol_copy.do_disconnectivity_calculations()

    return sol_copy