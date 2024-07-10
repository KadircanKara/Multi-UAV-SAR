# from Connectivity import *
from PathSolution import *
from Distance import *
from scipy.io import savemat
from statistics import median_low
from copy import deepcopy

def calculate_max_visits(sol:PathSolution):
    info = sol.info
    xs, ys = get_real_paths(sol)
    nvisits = calculate_nvisits_and_visit_times_and_tbv(xs, ys, info)[0]
    return max(nvisits)

def nvisits_hard_constraint(sol:PathSolution):
   info = sol.info
   xs, ys = get_real_paths(sol)
   nvisits = calculate_nvisits_and_visit_times_and_tbv(xs, ys, info)[0]
   return max(nvisits)

def calculate_nvisits_and_visit_times_and_tbv(xs, ys, info:PathInfo):
   
    number_of_nodes, time_slots = xs.shape
   
    real_time_coords = np.empty(xs.shape, dtype=tuple)
    real_time_cells = np.empty(xs.shape, dtype=int)

    nvisits = np.zeros(info.number_of_cells, dtype=int)
    visit_times = list(map(lambda x: [], np.empty(info.number_of_cells, dtype=list))) # Empty list of lists
    time_between_visits = deepcopy(visit_times)

    for node in range(number_of_nodes):
      real_time_coords[node] = list(zip(xs[node],ys[node]))
      real_time_cells[node] = list(map(lambda x: get_city(x, info.grid_size, info.cell_side_length), real_time_coords[node]))
      for i in range(1,time_slots):
         current_cell = real_time_cells[node][i]
         if current_cell == -1:
            continue
         prev_cell = real_time_cells[node][i-1]
         if current_cell != prev_cell:
            # Update nvisits
            nvisits[current_cell] += 1
            # Update visit_times
            visit_times[current_cell].append(i)
    
    visit_times = list(map(lambda x: np.unique(x).tolist(), visit_times))

    for cell_no, cell_visit_times in enumerate(visit_times):
        # sorted_cell_visit_times = sorted(cell_visit_times)
        # print(f"time_between_visits[cell_no]: {time_between_visits[cell_no]}")
        # print(f"cell {cell_no} visit times: {cell_visit_times}")
        for i in range(1,len(cell_visit_times)):
           # print("->", cell_visit_times[i]-cell_visit_times[i-1])
           time_between_visits[cell_no].append(cell_visit_times[i]-cell_visit_times[i-1])
        if len(time_between_visits[cell_no])==0:
           time_between_visits[cell_no] = [0]
        # print(f"cell {cell_no} tbv: {time_between_visits[cell_no]}")

    return nvisits, visit_times, time_between_visits
    


    # print(f"nvisits: {nvisits}")
    # print(f"visit_times: {visit_times}")
    # print(f"tbv: {time_between_visits}")

def get_cartesion_drone_path(sol:PathSolution):

    real_time_drone_mat = sol.real_time_path_matrix

    real_time_cartesian_drone_dict = dict()

    time_slot = len(real_time_drone_mat[0])+2

    drone_no = 0

    for drone_path in real_time_drone_mat:

        cartesian_path = [[-1, -1]]
        for city in drone_path:
            cartesian_path.append(sol.get_coords(sol, city))

        cartesian_path.append([-1,-1])
        real_time_cartesian_drone_dict[drone_no] = cartesian_path

        drone_no += 1

    x_values = np.zeros((time_slot, sol.info.number_of_drones+1))
    y_values = np.zeros((time_slot, sol.info.number_of_drones+1))

    total_len = 0

    path_start_points = [0]

    for key in real_time_cartesian_drone_dict:
        path = real_time_cartesian_drone_dict[key]

        total_len += len(path)

        path_start_points.append(total_len)

        for time in range(time_slot):
            coord = path[time]
            x_values[time, key] = coord[0]
            y_values[time, key] = coord[1]

    path_start_points.pop(-1)

    return x_values, y_values, path_start_points

def get_real_real_path(xs, ys, path_start_points):

  n_drones = len(path_start_points)

  lens = dict()

  x = xs[:, 0]

  for i in range(len(xs[1:])):
    lens[i] = []

  for n in range(n_drones):
    x      = xs[:,n]
    el_prev = x[0]
    interp_x = np.array([])

    for i, el in enumerate(x[1:]):

      step = 1 if el>=el_prev else -1
      interp_mid = np.arange(el_prev*20, el*20+1, step) / 20
      interp_x = np.concatenate((interp_x, interp_mid))
      el_prev = el
      lens[i].append(len(interp_mid))

    y = ys[:,n]
    el_prev = y[0]
    interp_y = np.array([])

    for i, el in enumerate(y[1:]):

      step = 1 if el>=el_prev else -1
      interp_mid = np.arange(el_prev*20, el*20+1, step) / 20
      interp_y = np.concatenate((interp_y, interp_mid))
      el_prev = el
      lens[i].append(len(interp_mid))

  max_lens = []

  for i, key in enumerate(lens):
    #print(lens[key], "\t \t", xs[i+1, :])
    max_lens.append(max(lens[key]))

  lens = dict()

  for i in range(len(x[1:])):
    lens[i] = []

  final_interp_x = []
  final_interp_y = []

  for n in range(n_drones):
    x = xs[:,n]
    el_prev = x[0]
    interp_x = np.array([])

    for i, el in enumerate(x[1:]):
      interp_mid = np.linspace(el_prev*20, el*20+1, max_lens[i]) / 20
      interp_x = np.concatenate((interp_x, interp_mid))
      el_prev = el
      lens[i].append(len(interp_mid))

    final_interp_x.append(interp_x)

    y = ys[:,n]
    el_prev = y[0]
    interp_y = np.array([])

    for i, el in enumerate(y[1:]):
      # print("-->",max_lens[i])
      interp_mid = np.linspace(el_prev*20, el*20+1, max_lens[i]) / 20
      interp_y = np.concatenate((interp_y, interp_mid))
      el_prev = el
      lens[i].append(len(interp_mid))

    final_interp_y.append(interp_y)


  return np.array(final_interp_x), np.array(final_interp_y)


def get_real_paths(sol):
    sync = sol.info.model != distance_soo_model

    info = sol.info
    time_steps = sol.real_time_path_matrix.shape[1]

    path_matrix = np.where(sol.real_time_path_matrix != -1, sol.real_time_path_matrix % info.number_of_cells, sol.real_time_path_matrix)

    mission_time = 0

    # Preallocate coordinate lists
    sol.x_coords_list = [[] for _ in range(info.number_of_drones)]
    sol.y_coords_list = [[] for _ in range(info.number_of_drones)]

    # Extract coordinates
    coords = np.array([sol.get_coords(i) for i in range(info.number_of_cells)])

    for i in range(1, time_steps):
        current_step_cells, next_step_cells = path_matrix[1:, i-1], path_matrix[1:, i]

        # Calculate distances and times
        current_step_coords = coords[current_step_cells]
        next_step_coords = coords[next_step_cells]
        coord_diffs = next_step_coords - current_step_coords
        thetas = np.arctan2(coord_diffs[:, 1], coord_diffs[:, 0])

        drone_dists = np.linalg.norm(coord_diffs, axis=1)
        max_dist = np.max(drone_dists)
        step_time = max_dist / info.max_drone_speed
        mission_time += step_time

        drone_speeds = drone_dists / step_time if sync else np.full(info.number_of_drones, info.max_drone_speed)

        for j in range(info.number_of_drones):
            current_x, current_y = current_step_coords[j]
            next_x, next_y = next_step_coords[j]
            speed = drone_speeds[j]
            theta = thetas[j]

            num_points = ceil(step_time) if sync else 2
            x_coords, y_coords = get_path_coords(current_x, current_y, next_x, next_y, speed, theta, num_points)
            
            sol.x_coords_list[j].extend(x_coords)
            sol.y_coords_list[j].extend(y_coords)

    sol.mission_time = mission_time if sync else sol.longest_subtour / info.max_drone_speed
    sol.drone_timeslots = [len(x) for x in sol.x_coords_list]
    sol.time_slots = max(sol.drone_timeslots)

    # Initialize xy matrix
    x_sink, y_sink = sol.get_coords(-1)
    sol.x_matrix = np.full((info.number_of_drones + 1, sol.time_slots), x_sink)
    sol.y_matrix = np.full((info.number_of_drones + 1, sol.time_slots), y_sink)

    for i in range(info.number_of_drones):
        drone_time = sol.drone_timeslots[i]
        sol.x_matrix[i + 1, :drone_time] = sol.x_coords_list[i]
        sol.y_matrix[i + 1, :drone_time] = sol.y_coords_list[i]

    return sol.x_matrix, sol.y_matrix

def get_path_coords(current_x, current_y, next_x, next_y, speed, theta, num_points):
    """
    Compute the real-time coordinates between two points (current_x, current_y) and (next_x, next_y)
    given the speed and angle of movement.
    """
    if current_x != next_x:
        x_coords = np.linspace(current_x, next_x, num_points)
    else:
        x_coords = np.full(num_points, current_x)
    
    if current_y != next_y:
        y_coords = np.linspace(current_y, next_y, num_points)
    else:
        y_coords = np.full(num_points, current_y)

    return x_coords, y_coords


'''def get_real_paths(sol:PathSolution):

    sync = True if sol.info.model!=distance_soo_model else False

    info = sol.info

    time_steps = sol.real_time_path_matrix.shape[1]

    # path_matrix = sol.real_time_path_matrix % info.number_of_cells

    path_matrix = np.where(sol.real_time_path_matrix != -1, sol.real_time_path_matrix % info.number_of_cells, sol.real_time_path_matrix)
    # path_matrix = sol.real_time_path_matrix

    # print("Original Path Matrix:",sol.real_time_path_matrix)
    # print("Path Matrix:",path_matrix)

    mission_time = 0

    for i in range(1, time_steps):
        current_step_cells , next_step_cells = path_matrix[1:,i-1].tolist() , path_matrix[1:,i].tolist()
        # Calculate Drone Speeds Based On Distance
        drone_dists = np.array([info.D[current_step_cells[j],next_step_cells[j]] for j in range(info.number_of_drones)])# Calculate Distance for Each Drone
        max_dist = max(drone_dists)
        step_time = max_dist / info.max_drone_speed
        mission_time += step_time
        # print("-->",drone_dists, step_time)
        drone_speeds = drone_dists / step_time if sync else [info.max_drone_speed]*len(drone_dists)
        # print("->",drone_speeds)
        # print(f"Drone Dists: {drone_dists}\nStep Time: {step_time}\nDrone Speeds: {drone_speeds}")
        current_step_coords = list(map(sol.get_coords, current_step_cells))
        next_step_coords = list(map(sol.get_coords, next_step_cells))
        coord_diffs = [next_step_coords[j] - current_step_coords[j] for j in range(info.number_of_drones)]
        thetas = [atan2(j[1],j[0]) for j in coord_diffs]
        # Changes in current_to_next_step !!!
        if sync:
            current_to_next_step_x_coords = [ np.arange(current_step_coords[j][0], next_step_coords[j][0], drone_speeds[j] * cos(thetas[j])) if current_step_coords[j][0] != next_step_coords[j][0] else np.array([current_step_coords[j][0]]*ceil(step_time)) for j in range(info.number_of_drones) ]
            current_to_next_step_y_coords = [ np.arange(current_step_coords[j][1], next_step_coords[j][1], drone_speeds[j] * sin(thetas[j])) if current_step_coords[j][1] != next_step_coords[j][1] else np.array([current_step_coords[j][1]]*ceil(step_time)) for j in range(info.number_of_drones) ]
        else:
            current_to_next_step_x_coords = [ np.arange(current_step_coords[j][0], next_step_coords[j][0], drone_speeds[j] * cos(thetas[j])) if current_step_coords[j][0] != next_step_coords[j][0] else np.array([current_step_coords[j][0]]*2) for j in range(info.number_of_drones) ]
            current_to_next_step_y_coords = [ np.arange(current_step_coords[j][1], next_step_coords[j][1], drone_speeds[j] * sin(thetas[j])) if current_step_coords[j][1] != next_step_coords[j][1] else np.array([current_step_coords[j][1]]*2) for j in range(info.number_of_drones) ]


        # if i < 10:
        #     print(f"Step {i}")
        #     print(f"current step cells: {current_step_cells}, next step cells: {next_step_cells}")
        #     print(f"current_to_next_step_x_coords: {current_to_next_step_x_coords}, current_to_next_step_y_coords: {current_to_next_step_y_coords}")

        for j in range(info.number_of_drones):
            x_coords, y_coords = current_to_next_step_x_coords[j], current_to_next_step_y_coords[j]
            if len(x_coords) != len(y_coords):
                xy_diff = abs(len(x_coords) - len(y_coords))
                if len(x_coords) > len(y_coords): # Fill y
                    current_to_next_step_y_coords[j] = np.hstack((current_to_next_step_y_coords[j], np.array([y_coords[-1]]*xy_diff)))
                else: # Fill x
                    current_to_next_step_x_coords[j] = np.hstack((current_to_next_step_x_coords[j], np.array([x_coords[-1]]*xy_diff)))
            else:
                continue

        # if i==1:
        #     print(f"X - Current to Next Step: {current_to_next_step_x_coords}\nY - Current to Next Step: {current_to_next_step_y_coords}")


        sol.x_coords_list = [current_to_next_step_x_coords[j] if i==1 else np.hstack((sol.x_coords_list[j],current_to_next_step_x_coords[j])) for j in range(info.number_of_drones)]
        sol.y_coords_list = [current_to_next_step_y_coords[j] if i==1 else np.hstack((sol.y_coords_list[j],current_to_next_step_y_coords[j])) for j in range(info.number_of_drones)]

    sol.mission_time = mission_time if sync else sol.longest_subtour/info.max_drone_speed
    sol.drone_timeslots = [len(x) for x in sol.x_coords_list]
    sol.time_slots = max(sol.drone_timeslots)

    # Initialize xy matrix
    x_sink,y_sink = sol.get_coords(-1)
    sol.x_matrix = np.full((info.number_of_drones + 1, sol.time_slots), x_sink)  # Nd+1 rows in order to incorporate base station
    sol.y_matrix = sol.x_matrix.copy()
    # sol.realtime_real_time_path_matrix = sol.x_matrix.copy()
    # sol.realtime_real_time_path_matrix.astype(int)
    # sol.realtime_real_time_path_matrix[:, :] = -1
    # interpolated_path_dict = dict()
    # interpolated_path_max_len = 0

    # print(f"path matrix: {sol.real_time_path_matrix}")

    # print(f"x_coords_list: {sol.x_coords_list}\ny_coords_list: {sol.y_coords_list}")

    for i in range(info.number_of_drones):
        sol.x_matrix[i + 1] = np.hstack((sol.x_coords_list[i], np.array([x_sink] * (sol.time_slots - sol.drone_timeslots[i]))))
        sol.y_matrix[i + 1] = np.hstack((sol.y_coords_list[i], np.array([y_sink] * (sol.time_slots - sol.drone_timeslots[i]))))

    return sol.x_matrix, sol.y_matrix
'''
def get_real_connectivity_matrix(real_x, real_y, sol:PathSolution):

    info = sol.info
    comm_range = info.comm_cell_range * info.cell_side_length
    number_of_nodes, time_steps = real_x.shape
    real_connectivity_matrix = np.zeros((time_steps, number_of_nodes, number_of_nodes))

    for step in range(real_x.shape[1]):
       real_x_coords, real_y_coords = real_x[:,step], real_y[:,step]
       for node_1 in range(number_of_nodes):
          node_1_x_coord, node_1_y_coord = real_x_coords[node_1], real_y_coords[node_1]
          # node_1_cell = sol.get_city([node_1_x_coord,node_1_y_coord])
          for node_2 in range(node_1+1, info.number_of_nodes):
             node_2_x_coord, node_2_y_coord = real_x_coords[node_2], real_y_coords[node_2]
             # node_2_cell = sol.get_city([node_2_x_coord,node_2_y_coord])
             # print( "-->", (node_1_x_coord - node_2_x_coord)**2)
             if sqrt( (node_1_x_coord - node_2_x_coord)**2 + (node_1_y_coord - node_2_y_coord)**2 ) <= comm_range:
             # if info.D[node_1_cell, node_2_cell] <= comm_range:
                real_connectivity_matrix[step, node_1, node_2] = 1

    return real_connectivity_matrix

def calculate_time_penalty(sol:PathSolution):
    return get_visit_time_variance(sol) + get_max_visits(sol) + get_min_time_between_visits_variance(sol)

def get_visit_time_variance(sol:PathSolution):
    if not sol.cell_nvisits :
        calculate_visit_times(sol)
    return np.var(sol.cell_nvisits)

def get_max_visits(sol:PathSolution):
    if not sol.cell_nvisits :
        calculate_visit_times(sol)
    return max(sol.cell_nvisits) - sol.info.max_visits

def calculate_visit_times(sol:PathSolution):
    sol.cell_nvisits = [len(sol.cell_visit_steps[i]) for i in range(sol.info.number_of_cells)]
    return sol.cell_nvisits

def get_min_time_between_visits_variance(sol:PathSolution):
    if not sol.min_tbv:
        get_min_time_between_visits(sol)
    return np.var(sol.min_tbv)

def get_min_time_between_visits(sol:PathSolution):
    if not sol.tbv:
        calculate_time_between_visits(sol)
    sol.min_tbv = [min(sol.tbv[i]) for i in range(sol.info.Nc)]
    return sol.min_tbv

def calculate_time_between_visits(sol:PathSolution):
    info = sol.info
    tbv = dict()
    for i in range(info.Nc):
        tbv[i] = [] # Initialize tbv for every cell
        for j in range(1,len(sol.cell_visit_steps[i])):
            tbv[i].append( sol.cell_visit_steps[i][j] - sol.cell_visit_steps[i][j-1] )
        if len(tbv[i])==0: # For cells only visited once
            tbv[i].append(0)

    sol.tbv = tbv

    return sol.tbv
