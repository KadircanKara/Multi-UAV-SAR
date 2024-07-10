# Comprehensive Sequential Constructive Crossover (CSCX)
from typing import Dict
import numpy as np
import random
from time import sleep
import os
from scipy.signal import convolve

from PathSolution import *

from pymoo.core.crossover import Crossover
from pymoo.core.variable import Real, get



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


def ox(receiver, donor, seq=None, shift=False):
    """
    The Ordered Crossover (OX) as explained in http://www.dmi.unict.it/mpavone/nc-cs/materiale/moscato89.pdf.

    Parameters
    ----------
    receiver : numpy.array
        The receiver of the sequence. The array needs to be repaired after the donation took place.
    donor : numpy.array
        The donor of the sequence.
    seq : tuple (optional)
        Tuple with two problems defining the start and the end of the sequence. Please note in our implementation
        the end of the sequence is included. The sequence is randomly chosen if not provided.

    shift : bool
        Whether during the repair the receiver should be shifted or not. Both version of it can be found in the
        literature.

    Returns
    -------

    y : numpy.array
        The offspring which was created by the ordered crossover.

    """
    assert len(donor) == len(receiver)

    # print(f"donor: {donor}\nreceiver: {receiver}")

    # the sequence which shall be use for the crossover
    seq = seq if not None else random_sequence(len(receiver))
    start, end = seq

    # print(f"seq: {seq}")

    # the donation and a set of it to allow a quick lookup
    donation = np.copy(donor[start:end + 1])
    donation_as_set = set(donation)

    # the final value to be returned
    y = []

    for k in range(len(receiver)):

        # print(f"iteration {k}: {y}")

        # do the shift starting from the swapped sequence - as proposed in the paper
        i = k if not shift else (start + k) % len(receiver)
        v = receiver[i]

        if v not in donation_as_set:
            y.append(v)

    # now insert the donation at the right place
    y = np.concatenate([y[:start], donation, y[start:]]).astype(copy=False, dtype=int).tolist()

    # print(f"Offspring: {y}")

    return y



# My Crossovers

# Update Crossover Probabilities

def scx_crossover_test(p1:PathSolution, p2:PathSolution):

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
        # print("-->",info.D[offspring[-1] % info.number_of_cells])
        if info.D[offspring[-1] % info.number_of_cells, next_city1 % info.number_of_cells] <= info.D[offspring[-1] % info.number_of_cells, next_city2 % info.number_of_cells]:
            offspring[i] = next_city1
        else:
            offspring[i] = next_city2
        # if i % 2 == 0:
        #     offspring[i] = next_city1
        # else:
        #     offspring[i] = next_city2

        current_city_index = (current_city_index + 1) % size

    p1_sp_sol, p2_sp_sol = PathSolution(offspring, p1.start_points, info), PathSolution(offspring, p2.start_points, info)

    return p1_sp_sol, p2_sp_sol

    # if p1_sp_sol.total_distance <= p2_sp_sol.total_distance:
    #     return p1_sp_sol
    # else:
    #     return p2_sp_sol

    # return offspring


# Sequential Combination Crossover (1-offspring)
def scx_crossover(p1:PathSolution, p2:PathSolution):
    # Start from a randomly selected city.
    # Select the next city from one of the parents based on minimum cost criteria.
    # Ensure that each city is visited exactly once.
    # Alternate between parents to select subsequent cities, ensuring a balance between both parents' routes.
    info = p1.info
    # print("-->", type(p1.path), type(p2.path))
    p1_path = np.asarray(p1.path)
    p2_path = np.asarray(p2.path)
    # Choose a random cell
    cell = random.randint(0,info.number_of_cells-1)
    # initialize offspring
    offspring = [cell]
    # Decide on the next cell depending on distances to next cell on both parents (Ensure no duplicates)
    while (len(offspring) < len(p1_path)):
        cell = offspring[-1]
        # print(np.where(p1_path==cell), np.where(p2_path==cell))
        p1_cell_ind, p2_cell_ind = np.where(p1_path==cell)[0][0], np.where(p2_path==cell)[0][0]
        if p1_cell_ind == len(p1_path)-1:
            p1_next_cell = p1_path[0]
        else:
            p1_next_cell = p1_path[p1_cell_ind+1]
        if p2_cell_ind == len(p2_path)-1:
            p2_next_cell = p2_path[0]
        else:
            p2_next_cell = p2_path[p2_cell_ind+1]
        # p2_next_cell = p2_path[p2_path.index(cell)+1]
        p1_dist_to_next_cell = info.D[cell%info.number_of_cells, p1_next_cell%info.number_of_cells]
        p2_dist_to_next_cell = info.D[cell%info.number_of_cells, p2_next_cell%info.number_of_cells]
        if p1_dist_to_next_cell <= p2_dist_to_next_cell:
            if p1_next_cell not in offspring:
                offspring.append(p1_next_cell)
            else:
                offspring.append(p2_next_cell)
        else:
            offspring.append(p2_next_cell)
    offspring = np.asarray(offspring)
    # Offspring generated, now decide on which start_points to use by comparing total distances
    p1_sp_sol = PathSolution(offspring, p1.start_points, info)
    p2_sp_sol = PathSolution(offspring, p2.start_points, info)

    return p1_sp_sol, p2_sp_sol

    # if p1_sp_sol.total_distance <= p2_sp_sol.total_distance:
    #     # print(p1_sp_sol.path)
    #     return p1_sp_sol
    # else:
    #     print(p2_sp_sol.path)
    #     return p2_sp_sol

# Ordered Crossover (2-offsprings)
def ox_crossover(p1:PathSolution, p2:PathSolution):

    # exchange the sequence in p1 with p2

    info = p1.info

    p1_path = p1.path
    p2_path = p2.path

    seq = random_sequence(len(p1_path))
    start, end = seq

    p1_seq = p1_path[start:end]
    p2_seq = p2_path[start:end]

    offspring_1 = [None] * len(p1_path) # path 1 seq, rest from path 2
    offspring_2 = [None] * len(p2_path) # path 2 seq, rest from path 1

    offspring_1[start:end] = p1_seq
    offspring_2[start:end] = p2_seq

    for i in range(len(offspring_1)):

        path_1_ind, path_2_ind = 0, 0

        if i < start or i >= end:

            cell_from_path_2 = p2_path[path_2_ind]
            while(cell_from_path_2 in offspring_1):
                path_2_ind += 1
                cell_from_path_2 = p2_path[path_2_ind]
            offspring_1[i] = cell_from_path_2

            cell_from_path_1 = p1_path[path_1_ind]
            while(cell_from_path_1 in offspring_2):
                path_1_ind += 1
                cell_from_path_1 = p1_path[path_1_ind]
            offspring_2[i] = cell_from_path_1


    # print(f"Offspring 1 # Unique Cells: {len(np.unique(offspring_1))}\nOffspring 2 # Unique Cells: {len(np.unique(offspring_2))}")

    # print(f"(start, end): {seq}\np1 seq: {p1_seq}\np2 seq: {p2_seq}\np1: {p1_path}\np2: {p2_path}\noffspring 1: {offspring_1}\noffspring 2: {offspring_2}")

    # offspring_1 = np.asarray(p1_path[:start] + p2_seq + p1_path[end:])
    # offspring_2 = np.asarray(p2_path[:start] + p1_seq + p2_path[end:])

    return PathSolution(offspring_1, p1.start_points, info), PathSolution(offspring_2, p2.start_points, info)

# Partially Mapped Crossover (2-offsprings) (NOT USED AT THE MOMENT !)
def pmx_crossover(p1:PathSolution, p2:PathSolution):

    info = p1.info

    p1_path = p1.path
    p2_path = p2.path

    offspring_1 = [None] * len(p1.path)
    offspring_2 = offspring_1.copy()

    ox_point = random.choice(np.arange(1, len(p1.path)-1, 1))
    offspring_1 = np.hstack((p1.path[:ox_point], p2.path[ox_point:]))
    offspring_2 = np.hstack((p2.path[:ox_point], p1.path[ox_point:]))

    # IF THERE ARE DUPLICATE CELLS, REPEAT THE PROCESS !!!
    while(len(np.unique(offspring_1)) != len(p1.path) and len(np.unique(offspring_2)) != len(p2.path)):
        print("IN PMX !!!")
        ox_point = random.choice(np.arange(1, len(p1.path)-1, 1))
        offspring_1 = np.hstack((p1.path[:ox_point], p2.path[ox_point:]))
        offspring_2 = np.hstack((p2.path[:ox_point], p1.path[ox_point:]))

    # print(f"1st component length: {len(p1.path[:ox_point])} 2nd component length: {len(p2.path[ox_point:])}")

    # print(f"ox point: {ox_point}\np1: {p1.path}\np2: {p2.path}\noffspring 1: {offspring_1}\noffspring 2: {offspring_2}")

    return PathSolution(offspring_1, p1.start_points, info), PathSolution(offspring_2, p2.start_points, info)

# Edge Recombination Crossover (1-offspring)
def erx_crossover(p1:PathSolution, p2:PathSolution):

    info = p1.info

    p1_path, p2_path = np.asarray(p1.path), np.asarray(p2.path)

    # Step 1: Get the neighbors for each cell for each parents (utilize dicts)
    # Initialize edge dicts for both parents
    p1_edges = dict()
    p2_edges = p1_edges.copy()
    edges_set = p1_edges.copy()
    # print(f"p1: {p1_path} p1 duplicate check: {np.unique(p1_path)}\np2: {p2_path} p2 len: {len(p2_path)==len(np.unique(p2_path))}")
    for cell in range(info.number_of_cells):
        # print(f"cell: {cell}")
        # print(np.where(p1_path==cell), np.where(p2_path==cell))
        p1_cell_ind, p2_cell_ind = np.where(p1_path==cell)[0][0], np.where(p2_path==cell)[0][0]
        # p1_cell_ind, p2_cell_ind = p1_path.index(cell), p2_path.index(cell)
        # Check if cell is located at the end-points or not (edge-case)
        # p1_isedge, p2_isedge = bool(p1_path.index(cell)==len(p1_path)-1 or p1_path.index(cell)==0), bool(p2_path.index(cell)==len(p2_path)-1 or p2_path.index(cell)==0)
        # Neighbors for parent 1
        if p1_cell_ind==len(p1_path)-1:
            p1_n1, p1_n2 = p1_path[0], p1_path[p1_cell_ind-1]
        elif p1_cell_ind==0:
            p1_n1, p1_n2 = p1_path[-1], p1_path[1]
        else:
            p1_n1, p1_n2 = p1_path[p1_cell_ind-1], p1_path[p1_cell_ind+1]
        # Neighbors for parent 2
        if p2_cell_ind==len(p2_path)-1:
            p2_n1, p2_n2 = p2_path[0], p2_path[p2_cell_ind-1]
        elif p1_cell_ind==0:
            p2_n1, p2_n2 = p2_path[-1], p2_path[1]
        else:
            p2_n1, p2_n2 = p2_path[p2_cell_ind-1], p2_path[p2_cell_ind+1]
        # Set dict values for both parents
        p1_edges[cell] = [p1_n1, p1_n2]
        p2_edges[cell] = [p2_n1, p2_n2]
        edges_set[cell] = list(set([p1_n1, p1_n2, p2_n1, p2_n2]))
    # Edge dicts are set, now pick a starting node at random (either the first parent's or the second parent's)
    if random.random() <= 0.5:
        offspring = [p1_path[0]]
    else:
        offspring = [p2_path[0]]
    # print(f"starting cell: {offspring[-1]}")
    # print(f"initial edges set:\n{edges_set}")
    # Remove starting node from the edge lists for each cell key
    for cell in edges_set:
        edges_set[cell] = [neighbor_list for neighbor_list in edges_set[cell] if neighbor_list != offspring[-1]]
    # print(f"edges set first update:\n{edges_set}")
    # Do the recombination from the edges dict
    while(len(offspring) < len(p1_path)):
        # print(f"offspring duplicate check: {len(offspring)==len(np.unique(offspring))}")
        # print(f"offspring: {offspring}")
        # print(f"-->", edges_set[offspring[-1]])
        neighbors = edges_set[offspring[-1]]
        # If neighbor list is empty, pick a random cell that has neighbors
        if len(neighbors) == 0:
            # Get cells that still have neighbors
            non_empty_edges_set = {c: n for c, n in edges_set.items() if n}
            neighbors = random.choice(list(non_empty_edges_set.values()))
            # neighbors = non_empty_edges_set[]
        neighbor_choice = neighbors[0]
        for i in range(len(neighbors)-1):
            if len(edges_set[neighbors[i+1]]) < len(edges_set[neighbors[i]]):
                neighbor_choice = neighbors[i]
        # If neighbor edge lengths are all equal, choose at random
        if neighbor_choice == neighbors [0]:
            neighbor_choice = random.choice(neighbors)
        # print(f"neighbor choice: {neighbor_choice}")
            # if info.D[offspring[-1] % info.number_of_cells, n % info.number_of_cells] < info.D[offspring[-1] % info.number_of_cells, neighbor_choice % info.number_of_cells]:
            #     neighbor_choice = n
        # Remove all occurances of neighbor_choice from the edge set
        for cell in edges_set:
            edges_set[cell] = [neighbor_list for neighbor_list in edges_set[cell] if neighbor_list != neighbor_choice]
        # print(f"edges set later update:\n{edges_set}")
        # Update offspring
        offspring.append(neighbor_choice)
    # print(f"offspring length {len(offspring)}")
    # Try solutions with both start_points, return the one with less total_distance
    p1_sp_sol, p2_sp_sol = PathSolution(offspring, p1.start_points, info), PathSolution(offspring, p2.start_points, info)
    # print(p1_sp_sol, p2_sp_sol)

    return p1_sp_sol, p2_sp_sol

    # if p1_sp_sol.total_distance <= p2_sp_sol.total_distance:
    #     return p1_sp_sol
    # else:
    #     return p2_sp_sol

# Distance Preserving Crossover (1-offspring) (NOT USED AT THE MOMENT !)
def dpx_crossover(p1:PathSolution, p2:PathSolution):
    # Get common edges of both parents and shuffle them to generate the offspring
    info = p1.info
    # fragments = np.array([]) # initialize empty array
    fragments = []
    p1_path, p2_path = p1.path, p2.path
    # print(f"p1: {p1_path}\np2: {p2_path}")
    i = 0
    j = 0
    for i in range(len(p1_path)-1):
        fragment = np.array([p1_path[i]])
        fragments.append(fragment)
        # print(f"initial fragment: {fragment}")
        while(find_subarray_convolve(p2_path, np.hstack((fragment, p1_path[i+1])))):
            fragment = np.hstack((fragment, p2_path[j]))
            i += 1
        # for j in range(i+1,len(p1_path)):
        #     # subpath = fragment + p1_path[j]
        #     subpath = np.hstack((fragment, p1_path[j])) # fragment.append(p1_path[j])
        #     if find_subarray_convolve(p2_path, subpath):
        #         # fragment.append(p1_path[j])
        #         fragment = np.hstack((fragment, p2_path[j]))
        #     else:
        #         break
        # print(f"final fragment {i+1}: {fragment}")
        # fragments = np.hstack((fragments, fragment))
        fragments.append(fragment)
    # print(f"fragments: {fragments}")
    if p1_path[-1] not in fragments[-1]:
        # fragments = np.hstack((fragments, np.array([p1_path[-1]])))
        fragments.append([p1_path[-1]])
    # Now shuffle the fragments
    # Create a list of indices
    indices = list(range(len(fragments)))
    # Shuffle the list of indices
    random.shuffle(indices)
    # Reorder the sublists using the shuffled indices
    shuffled_fragments = [fragments[i] for i in indices]
    offspring = [item for sublist in shuffled_fragments for item in sublist]

    if not len(np.unique(offspring))==len(p1_path):
        print("Offspring Invalid !")
        print(f"p1: {p1_path}\np2: {p2_path}\nfragments: {fragments}\nshuffled fragments: {shuffled_fragments}\noffspring: {offspring}")
        # sleep(0.5)
    else:
        print("Offspring Valid !")
        print(f"p1: {p1_path}\np2: {p2_path}\nfragments: {fragments}\nshuffled fragments: {shuffled_fragments}\noffspring: {offspring}")
        # sleep(0.5)
    #
    # print(f"offspring duplicate check: {len(np.unique(offspring))==len(p1_path)}")
    # print(f"p1: {p1_path}\np2: {p2_path}\nfragments: {fragments}\nshuffled fragments: {shuffled_fragments}\noffspring: {offspring}")
    # Try solutions with both start_points, return the one with less total_distance
    p1_sp_sol, p2_sp_sol = PathSolution(offspring, p1.start_points, info), PathSolution(offspring, p2.start_points, info)
    if p1_sp_sol.total_distance <= p2_sp_sol.total_distance:
        return p1_sp_sol
    else:
        return p2_sp_sol




    # for ind, cell in enumerate(p1_path):
    #     if ind==0: # Edge-case 1

    #     elif ind==len(p1_path)-1: # Edge-case 2

    #     else:
    #         n1,n2 = p1_path[ind-1], p1_path[ind+1]
    #         if list(set([cell,n1])):

'''
1) Use 5 Crossover Operators initially at the same probability
2) At each generation, calculate the offsprings generated in every generation
3) Evaluate performances, return the best performing in terms of BOTH distance and connectivity
4) Update probabilities based on the performances
'''

class PathCrossover(Crossover):

    # rnd = random.uniform(0, 1)
    #
    # rnd = 0.7 # ENFORCE SCX FOR NOW !!!
    #
    # if rnd < 0.5:
    #     n_offsprings = 2 # OX
    #     print("OX")
    # else :
    #     n_offsprings = 1 # SCX
    #     print("SCX")


    def __init__(self, n_parents=2, n_offsprings=2, test_operator_performance=False, multi_operators = False, **kwargs):
        super().__init__(n_parents=n_parents, n_offsprings=n_offsprings, **kwargs)

        self.test_operator_performance = test_operator_performance
        self.multi_operators = multi_operators

        self.n_offsprings = n_offsprings

        if self.multi_operators:
            self.operator_info = {
                "SCX": [scx_crossover_test, 1/3],
                "OX": [ox_crossover, 1/3],
                # "PMX": [2, pmx_crossover, 0.2],
                "ERX": [erx_crossover, 1/3]
            }


    def _do(self, problem, X, **kwargs):

        if self.multi_operators:
            probabilities = [info[1] for info in list(self.operator_info.values())]
            selected_operator = random.choices(list(self.operator_info.keys()), probabilities)[0]
            selected_operator_func = self.operator_info[selected_operator][0]

        _, n_matings, n_var = X.shape

        if self.test_operator_performance:
            sols = np.hstack((X[0,:,0], X[1,:,0]))
            # Find min and max dist for normalization when calculating performance
            dist_performances = [sol.total_distance for sol in sols]
            max_dist = max(dist_performances)
            min_dist = min(dist_performances)
            performances = dict()

        Y = np.full((self.n_offsprings, n_matings, n_var), None, dtype=PathSolution)

        for i in range(n_matings):

            selected_operator_func = ox_crossover
            Y[0,i,0], Y[1,i,0] = selected_operator_func(X[0, i, 0],X[1, i, 0])

            if self.test_operator_performance:
                # UPDATE OPERATOR PROBABILITIES
                # Calculate operator performances
                for operator_name, operator_info in self.operator_info.items():
                    offspring_1, offspring_2 = operator_info[0](X[0, i, 0],X[1, i, 0])
                    offspring_1_performance = (offspring_1.percentage_connectivity - (offspring_1.total_distance - min_dist) / (max_dist - min_dist)) / 2
                    offspring_2_performance = (offspring_2.percentage_connectivity - (offspring_2.total_distance - min_dist) / (max_dist - min_dist)) / 2
                    operator_performance = offspring_1_performance + offspring_2_performance
                    performances[operator_name] = operator_performance
                prob_sum = sum(list(performances.values()))
                # Update operator probabilities
                for operator_name, operator_info in self.operator_info.items():
                    operator_info[1] = performances[operator_name] / prob_sum

        return Y


# DEFAULT OX (GOES under for i in range(n_matings))
'''                path_1, path_2 = X[0, i, 0].path, X[1, i, 0].path
                start_points_1, start_points_2 = X[0, i, 0].start_points, X[1, i, 0].start_points

                start, end = random_sequence(problem.info.number_of_cities)

                path_cross_1 = ox(path_1, path_2, seq=(start, end), shift=self.shift)
                path_cross_2 = ox(path_2, path_1, seq=(start, end), shift=self.shift)

                Y[0, i, 0], Y[1, i, 0] = PathSolution(path_cross_1, start_points_1, problem.info), PathSolution(
                    path_cross_2, start_points_2, problem.info)
'''
