import numpy as np
from collections import deque
from PathSolution import PathSolution

def min_perc_conn_constraint(sol:PathSolution):
    # User-set threshold (e.g. from the interactive optimizer) overrides the
    # legacy hardcoded floor when present on the info object.
    thr = getattr(sol.info, "min_connectivity_constraint", None)
    if thr is not None:
        return thr - sol.percentage_connectivity
    if sol.info.number_of_drones > 2:
        return 0.5 - sol.percentage_connectivity
    else:
        return 0

def calculate_disconnected_timesteps(sol:PathSolution):

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

    return sol.disconnected_time_steps


def hovering_connectivity(sol:PathSolution):


    info = sol.info

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

    return connectivity_to_base_percentage


def enforce_hovering_connectivity(sol:PathSolution):


    info = sol.info

    hovering_drones = [i for i in range(info.number_of_drones) if sol.subtour_lengths[i] < max(sol.subtour_lengths)]
    
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

    return 1 - connectivity_to_base_percentage


def bfs_connectivity(sol:PathSolution):

    percentage_connectivity_list = []

    for i in range(sol.time_slots):

        adj_matrix = sol.connectivity_matrix[i]

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

    return sol.percentage_connectivity
    # return -sol.percentage_connectivity


def calculate_connectivity_to_base_percentage_matrix(sol : PathSolution):

    info = sol.info
    connectivity_to_base= sol.connectivity_to_base_matrix
    time_slots = sol.time_slots
    connectivity_to_base_percentage = np.zeros(time_slots)
    for time in range(time_slots):
        connectivity_to_base_percentage[time] = sum(connectivity_to_base[time, 1:])/(info.number_of_drones)
    return connectivity_to_base_percentage


def get_percentage_connectivity(sol : PathSolution):
    return sol.percentage_connectivity


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


def get_percentage_disconnectivity(sol:PathSolution):
    return sol.percentage_disconnectivity

def get_total_disconnected_time(sol:PathSolution):
    return sol.total_disconnected_time


def get_max_disconnected_time(sol:PathSolution):
    return sol.max_disconnected_time

def get_mean_disconnected_time(sol:PathSolution):
    return sol.mean_disconnected_time

# --------------------------------------------------------------------------------------------------------------
# Functions below are used in calculating connectivity related objective functions and constraints written above
# --------------------------------------------------------------------------------------------------------------


def dfs(connectivity_matrix, node, visited, component):
    visited[node] = True
    component.append(node)
    # print(connectivity_matrix[node])
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

def connected_nodes_at_step(adj, start_node):
    num_nodes = adj.shape[0]
    visited = [False] * num_nodes
    queue = deque([start_node])
    connected_count = 0

    visited[start_node] = True  # Mark start node as visited

    while queue:
        node = queue.popleft()
        connected_count += 1

        for j in range(num_nodes):
            if adj[node][j] != 0 and not visited[j]:
                queue.append(j)
                visited[j] = True  # Mark the connected node as visited

    return connected_count - 1  # Subtract 1 because start node is included in count

def get_connected_node_ids(adj_matrix, start_node):
    """
    Returns all nodes connected to the start_node (including multi-hop connectivity),
    excluding the start_node itself.

    Parameters:
    - adj_matrix: 2D NumPy array representing the adjacency matrix.
    - start_node: The node to find connections for.

    Returns:
    - A list of connected nodes excluding the start_node.
    """
    num_nodes = adj_matrix.shape[0]
    visited = [False] * num_nodes
    queue = deque([start_node])
    visited[start_node] = True
    connected_nodes = []

    while queue:
        node = queue.popleft()
        for neighbor in range(num_nodes):
            if adj_matrix[node, neighbor] == 1 and not visited[neighbor]:
                queue.append(neighbor)
                visited[neighbor] = True
                connected_nodes.append(neighbor)

    return connected_nodes
