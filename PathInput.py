from PathOptimizationModel import moo_model_with_disconn, moo_model_without_disconn, distance_soo_model, moo_model_mtsp

# CHANGE INPUTS FROM HERE !!!

model = distance_soo_model

algorithm = model['Alg']

# if model == distance_soo_model:
#     algorithm = 'GA'
# elif model == moo_model_with_disconn:
#     algorithm = ['NSGA2','NSGA3']

# algorithm = 'NSGA2' if model==moo_model else 'GA'

# One single scenario for testing
test_setup_scenario = {
'grid_size': 8,
'cell_side_length': 50,
'number_of_drones': 4,
'max_drone_speed': 2.5, # m/s
'comm_cell_range': 2,  # 2 cells
'min_visits': 3,  # Minimum number of cell visits
'max_visits':5, # Maximum number of cell visits
'number_of_targets': 1,
'target_positions':12,
'true_detection_probability': 0.99,
'false_detection_probability': 0.01,
'detection_threshold': 0.9,
'max_isolated_time': 0,
}

# For n=4,8,12,16 and r=2,4 with min_visits=1 (fixed)
comprehensive_setup_scenarios = [
    {
    'grid_size': 8,
    'cell_side_length': 50,
    'number_of_drones': 2,
    'max_drone_speed': 2.5, # m/s
    'comm_cell_range': 2,  # 2 cells
    'min_visits': 1,  # Minimum number of cell visits
    'max_visits':5, # Maximum number of cell visits
    'number_of_targets': 1,
    'target_positions':12,
    'true_detection_probability': 0.99,
    'false_detection_probability': 0.01,
    'detection_threshold': 0.9,
    'max_isolated_time': 0,
},
    {
    'grid_size': 8,
    'cell_side_length': 50,
    'number_of_drones': 2,
    'max_drone_speed': 2.5, # m/s
    'comm_cell_range': 4,  # 2 cells
    'min_visits': 1,  # Minimum number of cell visits
    'max_visits':5, # Maximum number of cell visits
    'number_of_targets': 1,
    'target_positions':12,
    'true_detection_probability': 0.99,
    'false_detection_probability': 0.01,
    'detection_threshold': 0.9,
    'max_isolated_time': 0,
},
    {
    'grid_size': 8,
    'cell_side_length': 50,
    'number_of_drones': 4,
    'max_drone_speed': 2.5, # m/s
    'comm_cell_range': 2,  # 2 cells
    'min_visits': 1,  # Minimum number of cell visits
    'max_visits':5, # Maximum number of cell visits
    'number_of_targets': 1,
    'target_positions':12,
    'true_detection_probability': 0.99,
    'false_detection_probability': 0.01,
    'detection_threshold': 0.9,
    'max_isolated_time': 0,
},
    {
    'grid_size': 8,
    'cell_side_length': 50,
    'number_of_drones': 4,
    'max_drone_speed': 2.5, # m/s
    'comm_cell_range': 4,  # 2 cells
    'min_visits': 1,  # Minimum number of cell visits
    'max_visits':5, # Maximum number of cell visits
    'number_of_targets': 1,
    'target_positions':12,
    'true_detection_probability': 0.99,
    'false_detection_probability': 0.01,
    'detection_threshold': 0.9,
    'max_isolated_time': 0,
},
    {
    'grid_size': 8,
    'cell_side_length': 50,
    'number_of_drones': 8,
    'max_drone_speed': 2.5, # m/s
    'comm_cell_range': 2,  # 2 cells
    'min_visits': 1,  # Minimum number of cell visits
    'max_visits':5, # Maximum number of cell visits
    'number_of_targets': 1,
    'target_positions':12,
    'true_detection_probability': 0.99,
    'false_detection_probability': 0.01,
    'detection_threshold': 0.9,
    'max_isolated_time': 0,
},
    {
    'grid_size': 8,
    'cell_side_length': 50,
    'number_of_drones': 8,
    'max_drone_speed': 2.5, # m/s
    'comm_cell_range': 4,  # 2 cells
    'min_visits': 1,  # Minimum number of cell visits
    'max_visits':5, # Maximum number of cell visits
    'number_of_targets': 1,
    'target_positions':12,
    'true_detection_probability': 0.99,
    'false_detection_probability': 0.01,
    'detection_threshold': 0.9,
    'max_isolated_time': 0,
},
    {
    'grid_size': 8,
    'cell_side_length': 50,
    'number_of_drones': 12,
    'max_drone_speed': 2.5, # m/s
    'comm_cell_range': 2,  # 2 cells
    'min_visits': 1,  # Minimum number of cell visits
    'max_visits':5, # Maximum number of cell visits
    'number_of_targets': 1,
    'target_positions':12,
    'true_detection_probability': 0.99,
    'false_detection_probability': 0.01,
    'detection_threshold': 0.9,
    'max_isolated_time': 0,
},
    {
    'grid_size': 8,
    'cell_side_length': 50,
    'number_of_drones': 12,
    'max_drone_speed': 2.5, # m/s
    'comm_cell_range': 4,  # 2 cells
    'min_visits': 1,  # Minimum number of cell visits
    'max_visits':5, # Maximum number of cell visits
    'number_of_targets': 1,
    'target_positions':12,
    'true_detection_probability': 0.99,
    'false_detection_probability': 0.01,
    'detection_threshold': 0.9,
    'max_isolated_time': 0,
},
    {
    'grid_size': 8,
    'cell_side_length': 50,
    'number_of_drones': 16,
    'max_drone_speed': 2.5, # m/s
    'comm_cell_range': 2,  # 2 cells
    'min_visits': 1,  # Minimum number of cell visits
    'max_visits':5, # Maximum number of cell visits
    'number_of_targets': 1,
    'target_positions':12,
    'true_detection_probability': 0.99,
    'false_detection_probability': 0.01,
    'detection_threshold': 0.9,
    'max_isolated_time': 0,
},
    {
    'grid_size': 8,
    'cell_side_length': 50,
    'number_of_drones': 16,
    'max_drone_speed': 2.5, # m/s
    'comm_cell_range': 4,  # 2 cells
    'min_visits': 1,  # Minimum number of cell visits
    'max_visits':5, # Maximum number of cell visits
    'number_of_targets': 1,
    'target_positions':12,
    'true_detection_probability': 0.99,
    'false_detection_probability': 0.01,
    'detection_threshold': 0.9,
    'max_isolated_time': 0,
},
    {
    'grid_size': 8,
    'cell_side_length': 50,
    'number_of_drones': 20,
    'max_drone_speed': 2.5, # m/s
    'comm_cell_range': 2,  # 2 cells
    'min_visits': 1,  # Minimum number of cell visits
    'max_visits':5, # Maximum number of cell visits
    'number_of_targets': 1,
    'target_positions':12,
    'true_detection_probability': 0.99,
    'false_detection_probability': 0.01,
    'detection_threshold': 0.9,
    'max_isolated_time': 0,
},
    {
    'grid_size': 8,
    'cell_side_length': 50,
    'number_of_drones': 20,
    'max_drone_speed': 2.5, # m/s
    'comm_cell_range': 4,  # 2 cells
    'min_visits': 1,  # Minimum number of cell visits
    'max_visits':5, # Maximum number of cell visits
    'number_of_targets': 1,
    'target_positions':12,
    'true_detection_probability': 0.99,
    'false_detection_probability': 0.01,
    'detection_threshold': 0.9,
    'max_isolated_time': 0,
}

]
