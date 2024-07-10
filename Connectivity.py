import numpy as np
from collections import deque
from PathSolution import PathSolution
# from Time import get_path_matrix

def calculate_disconnected_timesteps(sol:PathSolution):

    # Finds the maximum disconnected timesteps for each drone

    info = sol.info

    time_steps = sol.real_time_path_matrix.shape[1]

    disconnected_timesteps_matrix = np.zeros((info.number_of_drones, sol.connectivity_matrix.shape[0]), dtype=int)

    drone_total_disconnected_timesteps = np.zeros(info.number_of_drones, dtype=int)

    for i in range(info.number_of_drones):
        # print(disconnected_timesteps_matrix[i].shape, connected_nodes(sol,i + 1))
        disconnected_timesteps_matrix[i] = connected_nodes(sol,i + 1)  # To account for skipping the base station # 0,1 , 1,2 ... 7,8
        drone_total_disconnected_timesteps[i] = len(np.where(disconnected_timesteps_matrix[i] == 0)[0])

    sol.disconnected_time_steps = drone_total_disconnected_timesteps

    sol.mean_disconnected_time = np.mean(sol.disconnected_time_steps)
    sol.max_disconnected_time = np.max(sol.disconnected_time_steps)

    # print(f"Disconnected Time Steps: {sol.disconnected_time_steps}")

    return sol.disconnected_time_steps


def enforce_hovering_connectivity(sol:PathSolution):

    info = sol.info

    # Step 1: Identify hovering drones

    hovering_drones = [i for i in range(info.number_of_drones) if len(sol.drone_dict[i]) < sol.time_slots]

    hovering_cells = [sol.drone_dict[i][-2] for i in hovering_drones]

    hovering_cells.insert(0,-1) # Add BS' position to "hovering cells"

    connectivity_matrix = np.zeros((len(hovering_cells), len(hovering_cells)), dtype=int)

    for i in range(len(hovering_cells)):
        for j in range(len(hovering_cells)):
            if i!=j and info.D[hovering_cells[i], hovering_cells[j]] <= info.comm_dist:
                connectivity_matrix[i,j] = 1

    connectivity_to_base = np.zeros(info.number_of_nodes)

    connectivity_to_base[BFS(connectivity_matrix)] = 1

    connectivity_to_base_percentage = sum(connectivity_to_base[1:])/(len(hovering_drones))

    return connectivity_to_base_percentage - 1







def bfs_connectivity(sol:PathSolution):

    percentage_connectivity_list = []

    for i in range(sol.time_slots):

        adj_matrix = sol.connectivity_matrix[i]

        # print(f"Node Locations:\n{sol.real_time_path_matrix[:,i]}")
        # print(f"adj matrix:\n{adj_matrix}")

        num_nodes = len(adj_matrix)
        total_paths = 0

        target_node = 0

        for start_node in range(num_nodes):
            if start_node == target_node:
                continue

            # Initialize the BFS queue
            queue = deque([(start_node, 0)])
            visited = set([start_node])

            while queue:
                current_node, hops = queue.popleft()

                # If we've reached the target node, count the path
                if current_node == target_node:
                    total_paths += 1
                    break  # Break if we've found a valid path to avoid overcounting

                # Add neighbors to the queue
                for neighbor in range(num_nodes):
                    if adj_matrix[current_node][neighbor] == 1 and neighbor not in visited:
                        visited.add(neighbor)
                        queue.append((neighbor, hops + 1))

        # Total possible connections, excluding self-connections
        total_possible_connections = num_nodes - 1

        # Percentage connectivity
        percentage_connectivity = (total_paths / total_possible_connections) * 100

        percentage_connectivity_list.append(percentage_connectivity)

    sol.percentage_connectivity = np.mean(percentage_connectivity_list)

    return -sol.percentage_connectivity


def calculate_connectivity_to_base_percentage_matrix(sol : PathSolution):

    info = sol.info

    connectivity_to_base= sol.connectivity_to_base_matrix

    time_slots = sol.time_slots

    connectivity_to_base_percentage = np.zeros(time_slots)

    # print(f"time slots check: {time_slots}, {sol.real_time_path_matrix.shape[1]}")

    for time in range(time_slots):
        connectivity_to_base_percentage[time] = sum(connectivity_to_base[time, 1:])/(info.number_of_drones)
        # print(f"conn to base array: {connectivity_to_base[time]}, conn to base perc: {connectivity_to_base_percentage[time]}")

    return connectivity_to_base_percentage


def calculate_percentage_connectivity(sol : PathSolution):

    info = sol.info

    connectivity_to_base_percentage = calculate_connectivity_to_base_percentage_matrix(sol)


    perc_conn = np.mean(connectivity_to_base_percentage)

    sol.percentage_connectivity = perc_conn

    # print(f"Path Matrix:\n{sol.real_time_path_matrix}")

    # print(f"Connectivity to Base Percentages:\n{connectivity_to_base_percentage}")

    # print(f"Percentage Connectivity: {perc_conn}")

    return -perc_conn # sol.info.connectivity_period (after second :)


def BFS(adj):

    # v = sol.info.Nd+1
    v = len(adj)

    ctb = []
    start = 0
    # Visited vector to so that a
    # vertex is not visited more than
    # once Initializing the vector to
    # false as no vertex is visited at
    # the beginning
    visited = [False] * v
    q = [start]

    # Set source as visited
    visited[start] = True

    while q:
        vis = q[0]

        # Print current node
        ctb.append(vis)

        q.pop(0)

        # For every adjacent vertex to
        # the current vertex
        for i in range(v):
            if (adj[vis][i] == 1 and
                  (not visited[i])):

                # Push the adjacent node
                # in the queue
                q.append(i)

                # set
                visited[i] = True

    return ctb



# def calculate_percentage_connectivity(sol:PathSolution):
#     if not sol.percentage_connectivity :
#         num_connected_drones_to_base = connected_nodes(sol, 0)
#         sol.percentage_connectivity = 100 * sum(num_connected_drones_to_base) / (len(num_connected_drones_to_base) * sol.info.number_of_drones)
#     return -sol.percentage_connectivity
#     # return (sum(num_connected_drones_to_base) / (len(num_connected_drones_to_base) * info.Nd)) * 100

# def calculate_total_maxDisconnectedTime(sol:PathSolution):
#     # if sol.disconnected_time_steps is None:
#     #     calculate_disconnected_timesteps(sol)
#     sol.total_maxDisconnectedTime = sum(sol.max_disconnected_timesteps)
#     return sol.total_maxDisconnectedTime

def calculate_total_disconnected_time(sol:PathSolution):
    # if sol.disconnected_time_steps is None:
    # # if not sol.disconnected_time_steps:
    #     calculate_disconnected_timesteps(sol)
    sol.total_disconnected_time = sum(sol.disconnected_time_steps)
    return sol.total_disconnected_time


def calculate_max_disconnected_time(sol:PathSolution):
    # if sol.disconnected_time_steps is None:
    # # if not sol.disconnected_time_steps:
    #     calculate_disconnected_timesteps(sol)
    sol.max_disconnected_time = max(sol.disconnected_time_steps)
    return sol.max_disconnected_time

def calculate_mean_disconnected_time(sol:PathSolution):
    # if sol.disconnected_time_steps is None:
    #     calculate_disconnected_timesteps(sol)
    sol.mean_disconnected_time = np.mean(sol.disconnected_time_steps)
    return sol.mean_disconnected_time

# --------------------------------------------------------------------------------------------------------------
# Functions below are used in calculating connectivity related objective functions and constraints written above
# --------------------------------------------------------------------------------------------------------------


def dfs(connectivity_matrix, node, visited, component):
    visited[node] = True
    component.append(node)
    for neighbor, connected in enumerate(connectivity_matrix[node]):
        if connected == 1 and not visited[neighbor]:
            dfs(connectivity_matrix, neighbor, visited, component)

def connected_components(connectivity_matrix):
    n = len(connectivity_matrix)
    visited = [False] * n
    components = []

    for node in range(n):
        if not visited[node]:
            component = []
            dfs(connectivity_matrix, node, visited, component)
            components.append(component)

    return components


def connected_nodes(sol:PathSolution, start_node):

    # start node: The node that we calculate connectivity to

    info = sol.info
    num_nodes = info.number_of_nodes

    num_connected_drones = np.zeros(sol.connectivity_matrix.shape[0], dtype=int)

    for i in range(sol.connectivity_matrix.shape[0]):

        connectivity_matrix = sol.connectivity_matrix[i]

        # num_nodes = len(connectivity_matrix)
        visited = [False] * num_nodes
        queue = deque([start_node])
        connected_count = 0

        # print(f"visited: {visited}")
        # print(f"start node: {start_node}")

        visited[start_node] = True  # Mark start node as visited

        while queue:
            node = queue.popleft()
            connected_count += 1

            for j in range(num_nodes):
                if connectivity_matrix[node][j] != 0 and not visited[j]:
                    queue.append(j)
                    visited[j] = True  # Mark the connected node as visited

        num_connected_drones[i] = connected_count - 1

        # print("---------------------------------------------------------")
        # print(f"step {i}")
        # print("---------------------------------------------------------")
        # print(f"Connectivity Matrix:\n{connectivity_matrix}")
        # print(f"Number of nodes connected to node {start_node}: {num_connected_drones[i]}")

    return num_connected_drones

    # return connected_count - 1  # Subtract 1 because start node is included in count


'''# At Least 1 Hovering Drone Connected to BS Constraint
def enforce_hovering_connectivity(sol:PathSolution):
    conn_weight = 0.6
    dist_weight = 1-conn_weight
    info = sol.info
    hovering_threshold = 5
    # relay_conn_score = 0
    # relay_dist_score = 0
    # Get path matrix
    drone_path_matrix = get_path_matrix(sol)[1:,:]
    print(f"Path Matrix:\n{drone_path_matrix}")
    # Get unique cell counts
    num_unique_cells = np.array([len(np.unique(row)) for row in drone_path_matrix])
    # print(f"Number of Unique Cells:\n{num_unique_cells}")
    # Get drone id with the least unique cells (Most Hovering Drone)
    main_hovering_drone = np.argmin(num_unique_cells)
    hovering_drones = np.array( [i for i in range(len(num_unique_cells)) if num_unique_cells[i] < hovering_threshold] )
    # print(f"All Hovering Drone IDs: {hovering_drones}") # Use Later
    # print(f"Main Hovering Drone ID: {main_hovering_drone}")
    # Calculate Hovering Drones' Connectivity and Distance Scores to BS
    conn_scores = []
    dist_scores = []
    for drone in hovering_drones:
        conn_score = 0
        dist_score = 0
        drone_path = drone_path_matrix[drone]
        visited_cells, visit_counts = np.unique(drone_path, return_counts=True)
        for i,cell in enumerate(visited_cells):
            dist_to_bs = info.D[-1,cell]
            conn_score += int(dist_to_bs <= info.comm_cell_range * info.cell_side_length) * visit_counts[i]
            dist_score += dist_to_bs * visit_counts[i]
        conn_score /= len(drone_path)
        conn_scores.append(conn_score)
        dist_scores.append(dist_score)
    avg_conn_violation = -np.mean(conn_scores)
    avg_dist_violation = np.mean(dist_scores)

    # print(avg_conn_violation, avg_dist_violation)


    return avg_conn_violation * conn_weight + avg_dist_violation * dist_weight

    # Calculate Main Hovering Drone's Connectivity to BS
    main_hovering_drone_path = drone_path_matrix[main_hovering_drone]
    # print(f"Path Matrix:\n{drone_path_matrix}")
    print(f"Main Hovering Drone Path: {main_hovering_drone_path}")

    # main_hovering_drone_unique_cells = np.unique(main_hovering_drone_path)
    # for cell in main_hovering_drone_unique_cells:
    #     cell_to_bs_dist = info.D[-1,cell]
    #     if cell_to_bs_dist <= info.comm_cell_range * info.cell_side_length:
    #         relay_conn_score += 1 # Add 1 to relay_conn_score if all cells visited by the main hovering drone are connected to BS
    #     relay_dist_score += cell_to_bs_dist
'''
