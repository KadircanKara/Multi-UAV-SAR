from Distance import *
from Connectivity import *
from Time import *

model_metric_info = {
    'Total Distance': get_total_distance,
    # 'Total Distance with Revisit Penalty':get_total_distance_with_revisit_penalty,
    # 'Max Visits': get_max_visits,
    'Time Penalties': calculate_time_penalty,
    'Longest Subtour': get_longest_subtour,
    'Percentage Connectivity': calculate_percentage_connectivity,# bfs_connectivity
    'Total Disconnected Time': calculate_total_disconnected_time,
    'Max Disconnected Time': calculate_max_disconnected_time,
    'Mean Disconnected Time': calculate_mean_disconnected_time,
    'Max Number of Visits':calculate_max_visits,
    'Limit Long Jumps': long_jumps_ieq_constr,
    'No Long Jumps': long_jumps_eq_constr,
    'Limit Cell per Drone': min_cells_per_drone_constr,
    'Limit Max Longest Subtour': max_longest_subtour_constr,
    'Limit Min Longest Subtour': min_longest_subtour_constr,
    'Limit Subtour Range': max_subtour_range_constr,
    'Enforce Hovering Connectivity':enforce_hovering_connectivity,
    'Number of Visits Hard Constraint':nvisits_hard_constraint
    # 'Limit Max Visits': sol.max_visits_constr # Not Neccessary alongside limit long jumps cv
}
