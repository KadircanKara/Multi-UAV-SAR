import numpy as np
from pymoo.core.mutation import Mutation
from pymoo.operators.crossover.ox import random_sequence
from pymoo.operators.mutation.inversion import inversion_mutation
from scipy.spatial import distance
from typing import List, Dict
import random
from copy import copy, deepcopy

from PathSolution import *
from PathInfo import *
from PathProblem import *

class PathMutation(Mutation):

    def __init__(self,
                mutation_info={
                    "swap": (0.3, 3),
                    "inversion": (0.1, 1),
                    "scramble": (0.1, 1),
                    "insertion": (0.1, 1),
                    "displacement": (0.1, 1),
                    # "reverse sequence": (0.3, 1),
                    "block inversion": (0.1, 1),
                    # "shift": (0.3, 1),
                }
    ) -> None:

        super().__init__()

        self.mutation_info = mutation_info

    def _do(self, problem : PathProblem, X, **kwargs):

        Y = X.copy()

        for i, y in enumerate(X):
            # print("-->", y)
            sol : PathSolution = y[0]

            start_points = sol.start_points
            path = np.copy(sol.path)
            mut_path = path

            # print("Original Start Points:",start_points)
            #
            # PATH MUTATIONS
            if np.random.random() <= self.mutation_info["swap"][0]:
                for _ in range(self.mutation_info["swap"][1]):
                    seq = random_sequence(len(path))
                    # print(f"swapped cells: {mut_path[seq[0]]} and {mut_path[seq[1]]}")
                    # print(f"pre-swap path: {mut_path}")
                    mut_path = np.hstack((
                        mut_path[:seq[0]], np.array([mut_path[seq[1]]]), mut_path[seq[0]+1:seq[1]], np.array([mut_path[seq[0]]]), mut_path[seq[1]+1:]
                    ))
                    # print(f"post-swap path: {mut_path}")


            if np.random.random() <= self.mutation_info["inversion"][0]:
                for _ in range(self.mutation_info["inversion"][1]):
                    seq = random_sequence(len(path))
                    mut_path = inversion_mutation(mut_path, seq, inplace=True)


            if np.random.random() <= self.mutation_info["scramble"][0]:
                for _ in range(self.mutation_info["scramble"][1]):
                    seq = random_sequence(len(path))
                    random.shuffle(mut_path[seq[0]:seq[1]])


            if np.random.random() <= self.mutation_info["insertion"][0]:
                for _ in range(self.mutation_info["insertion"][1]):
                    cell = np.random.choice(mut_path)
                    cell_ind = np.where(mut_path == cell)[0][0]
                    mut_path = np.delete(mut_path, cell_ind)
                    new_position = np.random.choice(np.array([i for i in range(len(mut_path) + 1) if i != cell_ind]))
                    mut_path = np.insert(mut_path, new_position, cell)


            if np.random.random() <= self.mutation_info["displacement"][0]:
                for _ in range(self.mutation_info["displacement"][1]):
                    start, end = random_sequence(len(path))
                    seq = mut_path[start:end]
                    indices = np.arange(start, end)
                    mut_path = np.delete(mut_path, indices)
                    new_position = np.random.choice(np.array([i for i in range(len(mut_path) + 1) if i < start or i > start]))
                    mut_path = np.insert(mut_path, new_position, seq)


            if np.random.random() <= self.mutation_info["block inversion"][0]:
                for _ in range(self.mutation_info["block inversion"][1]):
                    start, end = random_sequence(len(path))
                    seq = np.flip(mut_path[start:end])
                    indices = np.arange(start, end)
                    mut_path = np.delete(mut_path, indices)
                    new_position = np.random.choice(np.array([i for i in range(len(mut_path) + 1) if i < start or i > start]))
                    mut_path = np.insert(mut_path, new_position, seq)



            # START POINTS MUTATIONS

                    # cell = np.random.choice(mut_path)
                    # print("cell: ", cell)
                    # cell_ind = np.where(mut_path==cell)[0][0]
                    # print("pre-insertion: ", mut_path)
                    # mut_path = np.delete(arr=mut_path, obj=cell, axis=0)
                    # mut_path = np.insert(mut_path, np.random.choice(np.array([i for i in range(len(mut_path)+1) if i != cell_ind])), cell)
                    # print("post-insertion: ", mut_path)

            mut_start_points = np.copy(start_points)

            Y[i][0] = PathSolution(mut_path, mut_start_points, problem.info)

        return Y


        #     if np.random.random() <= self.prob_scramble:
        #         for _ in range(self.num_scrambles):
        #             seq = random_sequence(len(path))
        #             random.shuffle(mut_path[seq[0]:seq[1]])

        #     if np.random.random() <= self.prob_inversion:
        #         for _ in range(self.num_inversions):
        #             seq = random_sequence(len(path))
        #             mut_path = inversion_mutation(mut_path, seq, inplace=True)

        #     if np.random.random() <= self.prob_random_swap:
        #         for _ in range(self.num_random_swaps):
        #                 seq = random_sequence(len(path))
        #                 temp = path[seq[0]]
        #                 mut_path[seq[0]] = mut_path[seq[1]]
        #                 mut_path[seq[1]] = temp

        #     mut_start_points = np.copy(start_points)

        #     prob = 1 / (len(start_points)-1) if len(start_points) > 1 else 0

        #     # random_start_points = random_start_points_from_ranges(problem.start_points_ranges, problem.number_of_drones)

        #     for j in range(1, len(start_points)):
        #         if np.random.random() <= prob:

        #             randomStart = np.random.randint(1, len(path))

        #             if randomStart not in mut_start_points:
        #                 mut_start_points[j] = randomStart

        #     sorted_mut_start_points = np.sort(mut_start_points)

        #     # print("Mutated Start Points:",sorted_mut_start_points)

        #     Y[i][0] = PathSolution(mut_path, sorted_mut_start_points, problem.info)

        # return Y


'''
class PathMutation(Mutation):

    def __init__(self, prob_random_swap=0.6, num_random_swaps=1,
                            prob_inversion=1, num_inversions=1,
                            prob_scramble=0.3, num_scrambles=1) -> None:
        super().__init__()
        self.prob_random_swap = prob_random_swap
        self.num_random_swaps = num_random_swaps
        self.prob_inversion = prob_inversion
        self.num_inversions = num_inversions
        self.prob_scramble = prob_scramble
        self.num_scrambles = num_scrambles

    def _do(self, problem : PathProblem, X, **kwargs):
        Y = X.copy()

        for i, y in enumerate(X):
            # print("-->", y)
            sol : PathSolution = y[0]

            start_points = sol.start_points
            path = np.copy(sol.path)
            mut_path = path

            # print("Original Start Points:",start_points)

            if np.random.random() <= self.prob_scramble:
                for _ in range(self.num_scrambles):
                    seq = random_sequence(len(path))
                    random.shuffle(mut_path[seq[0]:seq[1]])

            if np.random.random() <= self.prob_inversion:
                for _ in range(self.num_inversions):
                    seq = random_sequence(len(path))
                    mut_path = inversion_mutation(mut_path, seq, inplace=True)

            if np.random.random() <= self.prob_random_swap:
                for _ in range(self.num_random_swaps):
                        seq = random_sequence(len(path))
                        temp = path[seq[0]]
                        mut_path[seq[0]] = mut_path[seq[1]]
                        mut_path[seq[1]] = temp

            mut_start_points = np.copy(start_points)

            prob = 1 / (len(start_points)-1) if len(start_points) > 1 else 0

            # random_start_points = random_start_points_from_ranges(problem.start_points_ranges, problem.number_of_drones)

            for j in range(1, len(start_points)):
                if np.random.random() <= prob:

                    randomStart = np.random.randint(1, len(path))

                    if randomStart not in mut_start_points:
                        mut_start_points[j] = randomStart

            sorted_mut_start_points = np.sort(mut_start_points)

            # print("Mutated Start Points:",sorted_mut_start_points)

            Y[i][0] = PathSolution(mut_path, sorted_mut_start_points, problem.info)

        return Y
'''
