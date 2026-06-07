from PathOptimizationModel import *

# Select the optimization model by registry name (see list_models()).
model_name = "TCDT_MOO_NSGA2"
model = AVAILABLE_MODELS[model_name]
pop_size = 250
n_gen = 800
