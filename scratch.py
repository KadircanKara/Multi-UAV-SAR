import os

from numpy.lib.function_base import insert
os.system('clear')
import numpy as np
import pandas as pd
from scipy import io
import pickle
from scipy.signal import convolve
import random

from PathOptimizationModel import moo_model_with_disconn, distance_soo_model
from PathSolution import PathSolution
from PathInfo import PathInfo
from Time import get_real_paths
from Connectivity import *
from FileManagement import load_pickle
from PathInput import test_setup_scenario
from PathAnimation import *

'''

operators = ['SCX', 'OX', 'PMX', 'ERX', 'DPX']
probabilities = [0.2, 0.2, 0.2, 0.2, 0.2]  # Initial probabilities, can be adjusted
for _ in range(10):
    selected_operator = random.choices(operators, probabilities)[0]
    print(selected_operator)


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

# print(find_subarray_convolve(np.array([5,4,3,2,1]), np.array([1])))

'''

# Load saved animation objects and call them
# animation = pickle.load("Results/Animations/MOO_NSGA2_g_8_a_50_n_8_v_2.5_r_2_minv_1_maxv_5_Nt_1_tarPos_12_ptdet_0.99_pfdet_0.01_detTh_0.9_maxIso_0_BestPercentage_Connectivity_Animation.pkl")
# test_anim = load_pickle("Results/Animations/MOO_NSGA2_g_8_a_50_n_8_v_2.5_r_2_minv_1_maxv_5_Nt_1_tarPos_12_ptdet_0.99_pfdet_0.01_detTh_0.9_maxIso_0_BestPercentage_Connectivity_Animation.pkl")

scenario = str(PathInfo(test_setup_scenario))
direction = "Best"
obj = "Total_Distance"
sol = load_pickle(f"Results/Solutions/{scenario}-{direction}-{obj}-Solution.pkl")
anim = PathAnimation(sol)
anim()
# print("start_points:", sol.start_points)


# test_anim = load_pickle(f"Results/Animations/{scenario}_{direction}{obj}_Animation.pkl")
# test_anim()
