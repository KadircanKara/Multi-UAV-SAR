from Distance import *
from Connectivity import *
from Time import *

from PathSolution import *

model_metric_info = {
    "Objectives": {
        'Mission Time': (get_mission_time, 1, "seconds"),
        'Percentage Connectivity': (get_percentage_connectivity, -1, "%"),
        'Max Disconnected Time': (get_max_disconnected_time, 1, "timesteps"),
        'Mean Disconnected Time': (get_mean_disconnected_time, 1, "timesteps"),
        'Max Mean TBV': (get_max_mean_tbv, 1, "seconds"),
    },
    "Constraints": {
        'Max Mission Time': max_mission_time,
        'Min Percentage Connectivity': min_perc_conn_constraint,
        'Path Speed Violations as Constraint': path_speed_violations_as_constraint
    }
}