import numpy as np
import random
from scipy.signal import convolve
from PathSolution import *
from pymoo.core.crossover import Crossover


def find_subarray_convolve(arr, subarr):
    subarr_len = len(subarr)
    if subarr_len > len(arr):
        return False

    # Convolve the array with the original and reversed subarray
    conv_result_original = convolve(arr, subarr[::-1], mode='valid')
    conv_result_reverse = convolve(arr, subarr, mode='valid')

    # Check for the sum of squares match
    match_original = (conv_result_original == np.sum(subarr ** 2))
    match_reverse = (conv_result_reverse == np.sum(subarr[::-1] ** 2))

    # Return True if either match is found
    return np.any(match_original) or np.any(match_reverse)


# Pymoo Defined Crossovers

def random_sequence(n):
    start, end = np.sort(np.random.choice(n, 2, replace=False))
    return tuple([start, end])


def scx_crossover(p1:PathSolution, p2:PathSolution):
    info = p1.info

    p1_path = p1.path
    p2_path = p2.path

    size = len(p1_path)
    offspring = [-1] * size  # Initialize offspring with -1 indicating unvisited cities
    offspring[0] = p1_path[0]  # Start with the first city of parent1

    current_city_index = 0  # Start from the first city
    for i in range(1, size):
        # Look for the next city in parent1 that is not already in offspring
        next_city1 = p1_path[(current_city_index + 1) % size]
        while next_city1 in offspring:
            current_city_index = (current_city_index + 1) % size
            next_city1 = p1_path[(current_city_index + 1) % size]

        # Look for the next city in parent2 that is not already in offspring
        next_city2 = p2_path[(current_city_index + 1) % size]
        while next_city2 in offspring:
            current_city_index = (current_city_index + 1) % size
            next_city2 = p2_path[(current_city_index + 1) % size]

        # Select the next city based on a predefined criterion (e.g., alternating between parents)
        if info.D[offspring[-1] % info.number_of_cells, next_city1 % info.number_of_cells] <= info.D[offspring[-1] % info.number_of_cells, next_city2 % info.number_of_cells]:
            offspring[i] = next_city1
        else:
            offspring[i] = next_city2

        current_city_index = (current_city_index + 1) % size

    p1_sp_sol, p2_sp_sol = PathSolution(offspring, p1.start_points, info), PathSolution(offspring, p2.start_points, info)

    return p1_sp_sol, p2_sp_sol


# Ordered Crossover (2-offsprings)
def ox_crossover(p1: PathSolution, p2: PathSolution, n_offsprings:int):
    
    info = p1.info
    p1_path = p1.path
    p2_path = p2.path
    
    # Randomly select the crossover points
    start, end = random_sequence(len(p1_path))

    # Get the subsequences from both parents
    p1_seq = p1_path[start:end]
    p2_seq = p2_path[start:end]

    # Initialize offspring arrays with None values
    offspring_1 = [None] * len(p1_path)
    offspring_2 = [None] * len(p2_path)

    # Insert the selected subsequences into the offspring
    offspring_1[start:end] = p1_seq
    offspring_2[start:end] = p2_seq

    def fill_offspring(offspring, parent_path, start, end):
        parent_index = 0
        for i in range(len(offspring)):
            # Only fill the positions outside the selected subsequence
            if i < start or i >= end:
                # Find the next element from the parent that maintains the full order
                while offspring.count(parent_path[parent_index]) >= p1_path.count(parent_path[parent_index]):
                    parent_index += 1
                offspring[i] = parent_path[parent_index]
                parent_index += 1

    # Fill the remaining elements for both offspring, preserving the order and allowing duplicates
    fill_offspring(offspring_1, p2_path, start, end)
    fill_offspring(offspring_2, p1_path, start, end)

    if n_offsprings == 2:
        return PathSolution(offspring_1, p1.start_points, info), PathSolution(offspring_2, p2.start_points, info)# , PathSolution(offspring_1, p2.start_points, info), PathSolution(offspring_2, p1.start_points, info)
    elif n_offsprings == 4:
        return PathSolution(offspring_1, p1.start_points, info), PathSolution(offspring_2, p2.start_points, info), PathSolution(offspring_1, p2.start_points, info), PathSolution(offspring_2, p1.start_points, info)


class PathCrossover(Crossover):


    def __init__(self, prob=0.9, ox_prob=0.5, n_parents=2, n_offsprings=2, **kwargs):
        super().__init__(n_parents=n_parents, n_offsprings=n_offsprings, **kwargs)

        self.n_offsprings = n_offsprings

        self.prob = prob # 0.9

        self.ox_prob = ox_prob


    def _do(self, problem, X, **kwargs):


        _, n_matings, n_var = X.shape

        Y = np.full((self.n_offsprings, n_matings, n_var), None, dtype=PathSolution)

        for i in range(n_matings):

            # print("Crossover")

            # ox_1, ox_2 = ox_crossover(X[0, i, 0],X[1, i, 0])
            # scx_1, scx_2 = scx_crossover(X[0, i, 0],X[1, i, 0])
            # ox_perf = ((ox_1.percentage_connectivity + ox_2.percentage_connectivity)/2) # + ((ox_1.total_distance + ox_2.total_distance)/2)
            # scx_perf = ((scx_1.percentage_connectivity + scx_2.percentage_connectivity)/2) # + ((scx_1.total_distance + scx_2.total_distance)/2)
            # self.ox_prob = ox_perf / (ox_perf + scx_perf)
            
            if random.random() <= self.prob:
                if random.random() <= self.ox_prob:
                    if self.n_offsprings == 2:
                        Y[0,i,0], Y[1,i,0] = ox_crossover(X[0, i, 0],X[1, i, 0], n_offsprings=self.n_offsprings)
                    elif self.n_offsprings == 4:
                        Y[0,i,0], Y[1,i,0], Y[2,i,0], Y[3,i,0], = ox_crossover(X[0, i, 0],X[1, i, 0], n_offsprings=self.n_offsprings)
                else:
                    if self.n_offsprings == 2:
                        Y[0,i,0], Y[1,i,0] = scx_crossover(X[0, i, 0],X[1, i, 0])
                    elif self.n_offsprings == 4:
                        Y[0,i,0], Y[1,i,0], Y[2,i,0], Y[3,i,0] = scx_crossover(X[0, i, 0],X[1, i, 0])
            else:
                if self.n_offsprings == 2:
                    Y[0,i,0], Y[1,i,0] = X[0, i, 0],X[1, i, 0]
                elif self.n_offsprings == 4:
                    Y[0,i,0], Y[1,i,0], Y[2,i,0], Y[3,i,0] = X[0, i, 0],X[1, i, 0], X[0, i, 0],X[1, i, 0]

        return Y