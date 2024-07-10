import numpy as np
from scipy.io import savemat

from PathAnimation import PathAnimation
from PathInput import *
from PathInfo import *
from FilePaths import *
from FileManagement import *

scenario = "SOO_GA_g_8_a_50_n_4_v_2.5_r_2_minv_1_maxv_5_Nt_1_tarPos_12_ptdet_0.99_pfdet_0.01_detTh_0.9_maxIso_0-SolutionObjects.pkl"
sol = load_pickle(f"{solutions_filepath}{scenario}")[0]
anim = PathAnimation(sol)
anim()
print(f"Mission Time: {sol.mission_time/60}")
# print(f"Path Matrix:\n{sol.real_time_path_matrix}")

'''
scenario = str(PathInfo(default_scenario))
direction = "Mid"
obj = "Percentage_Connectivity"

# Load the .npz file
npz_file = np.load(f'{paths_filepath}{scenario}-{direction}-{obj}-Paths.npz')

# Convert it to a dictionary
npz_dict = {file: npz_file[file] for file in npz_file.files}

# Save the dictionary as a .mat file
savemat(f'{matlab_filepath}{scenario}-{direction}-{obj}-Paths.mat', npz_dict)
'''
