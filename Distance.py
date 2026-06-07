import numpy as np
from math import floor, sqrt

# from Time import calculate_visit_times, get_real_paths

from PathSolution import *

def min_cell_visits(sol:PathSolution):
    drone_num_cells = np.append(abs(np.diff(np.array(sol.start_points))), (sol.info.number_of_cells * sol.info.n_visits - sol.start_points[-1]))
    return - min(drone_num_cells) + ( (sol.info.number_of_cells * sol.info.n_visits) // sol.info.number_of_drones - 5 )

def max_cell_visits(sol:PathSolution):
    drone_num_cells = np.append(abs(np.diff(np.array(sol.start_points))), (sol.info.number_of_cells * sol.info.n_visits - sol.start_points[-1]))
    return max(drone_num_cells) - (sol.info.number_of_cells * sol.info.n_visits) // sol.info.number_of_drones

def limit_cell_range(sol:PathSolution):
    drone_num_cells = np.append(abs(np.diff(np.array(sol.start_points))), (sol.info.number_of_cells - sol.start_points[-1]))
    cell_range = max(drone_num_cells) - min(drone_num_cells)

    return cell_range - 3

def max_mission_time(sol:PathSolution):
    if sol.info.n_visits < 4:
        return sol.mission_time - 3600
    else:
        return sol.mission_time - (sol.info.n_visits * 600)

def get_mission_time(sol:PathSolution):
    return sol.mission_time

def get_total_diagonal_steps(sol:PathSolution):
    info = sol.info
    path = sol.path
    violations = 0
    for i in range(len(sol.path)-1):
        current_x_coord, current_y_coord = sol.get_coords(path[i])
        next_x_coord, next_y_coord = sol.get_coords(path[i+1])
        violations += int(bool(current_x_coord != next_x_coord and current_y_coord != next_y_coord))
    return violations

def get_mean_turning_angle(sol:PathSolution):
    info = sol.info
    drone_paths = [sol.drone_dict[key] for key in list(sol.drone_dict.keys()) if key != -1]
    penalties = np.array([])
    for path in drone_paths:
        drone_penalties = 0
        for i in range(len(path)-1):
            current_x_coord, current_y_coord = sol.get_coords(path[i])
            next_x_coord, next_y_coord = sol.get_coords(path[i+1])
            y_diff = next_y_coord - current_y_coord
            x_diff = next_x_coord - current_x_coord
            theta = np.arctan2(y_diff, x_diff)
            drone_penalties += int(bool(not (np.isclose(theta, 0) or np.isclose(theta, np.pi/2))))
        penalties = np.append(penalties, drone_penalties)
    print(f"mean turning angle{np.mean(penalties)}")
    return np.mean(penalties) - 2
            
            
def get_path_matrix(sol:PathSolution):
    info = sol.info
    return np.where(sol.real_time_path_matrix != -1, sol.real_time_path_matrix % info.number_of_cells, sol.real_time_path_matrix)


def get_total_distance(sol:PathSolution):
    if not sol.total_distance:
        if not sol.subtour_lengths:
            calculate_subtour_lengths(sol)
        sol.total_distance = sum(sol.subtour_lengths.values())
    return sol.total_distance


def get_subtour_range(sol:PathSolution):
    if not sol.longest_subtour:
        get_longest_subtour(sol)
    if not sol.shortest_subtour:
        get_shortest_subtour(sol)
    sol.subtour_range = sol.longest_subtour - sol.shortest_subtour
    return sol.subtour_range


def get_longest_subtour(sol:PathSolution):
    if not sol.longest_subtour:
        if not sol.subtour_lengths:
            calculate_subtour_lengths(sol)
        sol.longest_subtour = max(sol.subtour_lengths)
    return sol.longest_subtour


def get_shortest_subtour(sol:PathSolution):
    if not sol.shortest_subtour:
        if not sol.subtour_lengths:
            calculate_subtour_lengths(sol)
        sol.shortest_subtour = min(sol.subtour_lengths)
    return sol.shortest_subtour


def calculate_subtour_lengths(sol:PathSolution):

    if not sol.subtour_lengths:

        info = sol.info

        path_matrix = sol.real_time_path_matrix

        Nd, time_steps = path_matrix.shape
        Nd -= 1 # Remove base station

        subtour_lengths = dict()

        for i in range(info.number_of_drones):
            drone_path = path_matrix[i+1]
            drone_dist = 0
            for j in range(time_steps-1):
                drone_dist += info.D[drone_path[j],drone_path[j+1]]
            subtour_lengths[i] = drone_dist

        sol.subtour_lengths = subtour_lengths

    return sol.subtour_lengths


def calculate_drone_speed_violations(sol:PathSolution):

    info = sol.info
    start_points = deepcopy(sol.start_points)
    start_points = np.append(start_points, sol.info.number_of_cells * sol.info.n_visits)
    # start_points.append(sol.info.number_of_cells * sol.info.n_visits)
    drone_speed_violations = []

    path = list(map(lambda x: x%info.number_of_cells, sol.path))

    # print(f"Path: {path}\nStart Points: {start_points}")

    for i in range(len(start_points)-1):
        counter = 0
        drone_path = path[start_points[i]:start_points[i+1]]
        for j in range(len(drone_path)-1):
            if info.D[drone_path[j], drone_path[j+1]] > info.cell_side_length*sqrt(2):
                counter += 1
        drone_speed_violations.append(counter)

    sol.drone_speed_violations = drone_speed_violations

    return sol.drone_speed_violations


def calculate_path_speed_violations(sol:PathSolution):

    info = sol.info

    path = list(map(lambda x: x%info.number_of_cells, sol.path))
    total_speed_violations = 0
    for i in range(len(path)-1):
        if info.D[path[i], path[i+1]] > info.cell_side_length * sqrt(2):
            total_speed_violations += 1

    sol.path_speed_violations = total_speed_violations

    return sol.path_speed_violations

def total_drone_speed_violations_as_objective(sol:PathSolution):

    if not sol.drone_speed_violations:
        calculate_drone_speed_violations(sol)
    
    return sum(sol.drone_speed_violations)

def total_drone_speed_violations_as_constraint(sol:PathSolution):

    if not sol.drone_speed_violations:
        calculate_drone_speed_violations(sol)
    
    return sum(sol.drone_speed_violations) - 0 * (sol.info.n_visits - 1)

def max_drone_speed_violations_as_objective(sol:PathSolution):

    if not sol.drone_speed_violations:
        calculate_drone_speed_violations(sol)
    
    return max(sol.drone_speed_violations)

def max_drone_speed_violations_as_constraint(sol:PathSolution):

    if not sol.drone_speed_violations:
        calculate_drone_speed_violations(sol)
    
    return max(sol.drone_speed_violations) - 2 * (sol.info.n_visits - 1)


def path_speed_violations_as_objective(sol:PathSolution):

    if not sol.path_speed_violations:
        calculate_path_speed_violations(sol)
    
    return sol.path_speed_violations

def path_speed_violations_as_constraint(sol:PathSolution):

    if not sol.path_speed_violations:
        calculate_path_speed_violations(sol)

    return sol.path_speed_violations
    
def total_speed_violations_constr(sol:PathSolution):

    if not sol.drone_speed_violations:
        drone_speed_violations = calculate_drone_speed_violations(sol)
    
    total_speed_violations = sum(drone_speed_violations)

    if sol.info.n_visits == 1:
        return total_speed_violations
    elif sol.info.n_visits == 2:
        return total_speed_violations# - 1
    else:
        return total_speed_violations * (sol.info.n_visits - 2) * 6
    
def max_speed_violations_constr(sol:PathSolution):

    if not sol.drone_speed_violations:
        drone_speed_violations = calculate_drone_speed_violations(sol)

    max_speed_violations = max(drone_speed_violations)

    if sol.info.n_visits > 1:
        return max_speed_violations - 2 # Allows max 1 long jump per drone
    else:
        return max_speed_violations


def calculate_max_long_jumps_per_drone(sol:PathSolution):

    if not sol.drone_long_jump_violations:
        calculate_drone_speed_violations(sol)

    print(f"max long jumps: {max(sol.drone_long_jump_violations)}")
    
    return max(sol.drone_long_jump_violations) - 2 # Allows max 2 long jumps per drone



def min_cells_per_drone_constr(sol:PathSolution):

    info = sol.info

    start_points = sol.start_points
    last_start_point_subtractor = info.n_visits * info.number_of_cells

    cells_per_drone = []

    for i in range(len(start_points)-1):
        num_cells = start_points[i+1] - start_points[i]
        cells_per_drone.append(num_cells)

    cells_per_drone.append(last_start_point_subtractor - start_points[-1])

    constr = (info.number_of_cells * info.n_visits // info.number_of_drones) - 1

    cv = -min(cells_per_drone) + constr

    return cv


def max_subtour_range_constr(sol:PathSolution):
    if not sol.subtour_range:
        get_subtour_range(sol)
    return sol.subtour_range - sol.info.grid_size*(sol.info.cell_side_length*sqrt(2))


def max_longest_subtour_constr(sol:PathSolution):
    if not sol.longest_subtour :
        get_longest_subtour(sol)
    return sol.longest_subtour - 1000


def min_longest_subtour_constr(sol:PathSolution):
    if not sol.shortest_subtour :
        get_shortest_subtour(sol)
    return - sol.shortest_subtour + sol.info.min_subtour_length_threshold


def get_coords(sol:PathSolution, cell):

    grid_size = sol.info.grid_size
    A = sol.info.cell_side_length

    if cell == -1:
        x = -A / 2
        y = -A / 2
    else:
        x = (cell % grid_size + 0.5) * A
        y = (cell // grid_size + 0.5) * A
    return np.array([x, y])


def get_city(coords, grid_size, A):

    if coords[0] < 0 and coords[1] < 0:
        return -1
    else:
        x, y = coords
        return floor(y / A) * grid_size + floor(x / A)


def get_x_coords(cell, grid_size, A):

    if cell == -1:
        x = -A / 2
    else:
        # x = ((cell % n) % self.info.grid_size + 0.5) * self.info.cell_len
        x = (cell %grid_size + 0.5) * A
    return x


def get_y_coords(self, cell, grid_size, A):

    if cell == -1:
        y = -A / 2
    else:
        # y = ((cell % n) // grid_size + 0.5) * A
        y = (cell // grid_size + 0.5) * self.info.cell_side_length
    return y