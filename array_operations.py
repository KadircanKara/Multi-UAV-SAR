import numpy as np

def create_array_of_lists(rows, cols, fill_value):
    array_of_lists = np.empty((rows, cols), dtype=object)
    for i in range(rows):
        for j in range(cols):
            if fill_value == []:
                array_of_lists[i, j] = []
            else:
                array_of_lists[i, j] = [fill_value]  # Assign an empty list to each cell
    return array_of_lists

def initialize_search_map(rows, cols):
    search_map = np.empty((rows, cols), dtype=object)
    for i in range(rows):
        for j in range(cols):
            search_map[i, j] = {"timestep":-1, "belief":[0.5]}  # Assign an empty list to each cell
    return search_map