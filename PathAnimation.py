import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation
from Time import get_real_paths, get_real_connectivity_matrix, isCoordinateDiscrete
from matplotlib import patches

from PathSolution import *

class PathAnimation:
    def __init__(self, sol: PathSolution, fig, ax, target_locations=None, cell_occupancy_probabilities=None, B=0.9, p0=0.5):
        self.colors = ['b', 'g', 'r', 'c', 'm', 'y', 'k', 'mediumseagreen'] * 3  # Colors for 24 drones

        self.sol = sol
        self.fig = fig
        self.ax = ax
        self.paths = get_real_paths(self.sol)  # np.array([x_matrix, y_matrix])
        self.real_time_x_matrix, self.real_time_y_matrix = self.paths
        self.real_time_connectivity_matrix = get_real_connectivity_matrix(
            self.real_time_x_matrix, self.real_time_y_matrix, self.sol)
        self.target_locations = target_locations
        self.cell_occupancy_probabilities = np.array(cell_occupancy_probabilities)
        self.B = B
        self.p0 = p0
        
        print("Cell Occupancy Probabilities Steps:", self.cell_occupancy_probabilities.shape)
        print("Path Steps:", self.sol.real_time_path_matrix.shape[1])

        # Set ticks and labels
        x_ticks_values = [i for i in range(-self.sol.info.cell_side_length, 
                                           (self.sol.info.grid_size + 1) * self.sol.info.cell_side_length, 
                                           self.sol.info.cell_side_length)]
        y_tick_values = x_ticks_values.copy()
        x_ticks_labels = [i for i in range(-1, self.sol.info.grid_size + 1)]
        y_tick_labels = x_ticks_labels.copy()

        if ax:
            self.ax.set_xticks(x_ticks_values)
            self.ax.set_xticklabels(x_ticks_labels, fontsize=16)
            self.ax.set_yticks(y_tick_values)
            self.ax.set_yticklabels(y_tick_labels, fontsize=16)
            self.ax.grid(linestyle='--')
        else:
            plt.set_xticks(x_ticks_values)
            plt.set_xticklabels(x_ticks_labels)
            plt.set_yticks(y_tick_values)
            plt.set_yticklabels(y_tick_labels)
            plt.grid(linestyle='--')

        # Initialize target cells as rectangles
        if self.target_locations is not None and self.cell_occupancy_probabilities is not None:
            for cell in range(self.sol.info.number_of_cells):
                rect_color = "red" if cell in self.target_locations else "white"
                target_x, target_y = PathSolution.get_coords(self.sol, cell)
                bottom_left_x = target_x - self.sol.info.cell_side_length / 2
                bottom_left_y = target_y - self.sol.info.cell_side_length / 2
                rect = patches.Rectangle(
                    (bottom_left_x, bottom_left_y),
                    width=self.sol.info.cell_side_length,
                    height=self.sol.info.cell_side_length,
                    linewidth=1,
                    edgecolor='none',
                    facecolor=rect_color,
                    alpha=0.3
                )
                self.ax.add_patch(rect)

    def initialize_figure(self):
        """Initialize the plot elements for animation."""
        # Scatter plot for drones
        self.drone_animations = self.ax.scatter([], [], marker="o")

        # Path lines for each drone
        self.drone_path_lines = [
            self.ax.plot([], [], color=self.colors[_], marker="", linewidth=0.6)[0] 
            for _ in range(self.sol.info.number_of_nodes)
        ]

        # Connectivity lines between nodes
        self.connectivity_lines = [
            self.ax.plot([], [], color='k', marker="", linewidth=3)[0] 
            for _ in range(self.sol.info.number_of_nodes)
        ]

        # Annotations for cell occupancy probabilities
        self.anns = [
            self.ax.annotate(f"{self.p0:.2f}", xy=PathSolution.get_coords(self.sol, cell))
            for cell in range(self.sol.info.number_of_cells)
        ]

        # Initialize drone paths
        for drone_no, drone_path_line in enumerate(self.drone_path_lines):
            drone_x_path, drone_y_path = self.real_time_x_matrix[drone_no], self.real_time_y_matrix[drone_no]
            drone_path_line.set_data([drone_x_path], [drone_y_path])

        return self.drone_animations, *self.connectivity_lines, *self.anns

    def update(self, frame):
        """Update function for each frame of the animation."""
        # Update drone positions
        x_path = self.paths[0][:, frame]
        y_path = self.paths[1][:, frame]
        data = np.stack((x_path, y_path), axis=-1)
        self.drone_animations.set_offsets(data)

        # Update connectivity lines
        for node_no, node_connectivity_lines in enumerate(self.connectivity_lines):
            connectivity_lines_xdata = []
            connectivity_lines_ydata = []
            for node_no_2 in range(node_no + 1, self.sol.info.number_of_nodes):
                connectivity_array = self.real_time_connectivity_matrix[frame, node_no, :]
                if connectivity_array[node_no_2]:
                    connectivity_lines_xdata.extend([
                        self.paths[0][node_no, frame], self.paths[0][node_no_2, frame]
                    ])
                    connectivity_lines_ydata.extend([
                        self.paths[1][node_no, frame], self.paths[1][node_no_2, frame]
                    ])
            node_connectivity_lines.set_data(connectivity_lines_xdata, connectivity_lines_ydata)

        # Check if all drones are at discrete cells
        discrete_cell_counter = 0            
        drone_cells = []
        drone_coords = []
        for drone in range(self.sol.info.number_of_drones):
            drone_x, drone_y = x_path[drone+1], y_path[drone+1]
            drone_coords.append((drone_x, drone_y))
            drone_cells.append(self.sol.get_city((drone_x, drone_y)))
            if isCoordinateDiscrete(drone_x, drone_y, self.sol):
                discrete_cell_counter += 1

        drone_cells = np.array(drone_cells)

        # Reverted: update annotations & remove first column of cell_occupancy_probabilities
        annotation_delay = 0  # number of frames to wait before starting to update annotations

        # if frame >= annotation_delay and counter == self.sol.info.number_of_drones:
        # print(f"Drone Coords: {drone_coords}, Drone Cells: {drone_cells}, Current Discrete Cells: {self.sol.real_time_path_matrix[1:,0]}, EQUAL? {np.array_equal(drone_cells, self.sol.real_time_path_matrix[1:,0])} DISCRETE CELL COUNTER: {discrete_cell_counter} NUMBER OF DRONES: {self.sol.info.number_of_drones}")
        # if np.array_equal(drone_cells, self.sol.real_time_path_matrix[1:,0]) and discrete_cell_counter == self.sol.info.number_of_drones:
        if discrete_cell_counter == self.sol.info.number_of_drones:
            if self.cell_occupancy_probabilities.shape[1] > 0: # and self.sol.real_time_path_matrix.shape[1] > 0:
                cell_occupancy_probabilities_at_frame = self.cell_occupancy_probabilities[:, 0]
                for cell in range(self.sol.info.number_of_cells):
                    if cell in drone_cells:
                        max_recent_prob = cell_occupancy_probabilities_at_frame[cell]
                        self.anns[cell].set_text(f"{max_recent_prob:.2f}")                    
                        if cell in self.target_locations:
                            if max_recent_prob >= self.B:
                                target_x, target_y = PathSolution.get_coords(self.sol, cell)
                                bottom_left_x = target_x - self.sol.info.cell_side_length / 2
                                bottom_left_y = target_y - self.sol.info.cell_side_length / 2
                                rect = patches.Rectangle(
                                    (bottom_left_x, bottom_left_y),
                                    width=self.sol.info.cell_side_length,
                                    height=self.sol.info.cell_side_length,
                                    linewidth=1,
                                    edgecolor='none',
                                    facecolor="green",
                                    alpha=1.0
                                )
                                self.ax.add_patch(rect)
                                self.anns[cell].set_color("red")

                self.cell_occupancy_probabilities = self.cell_occupancy_probabilities[:, 1:]
                self.sol.real_time_path_matrix = self.sol.real_time_path_matrix[:, 1:]


        return self.drone_animations, *self.connectivity_lines, *self.anns

    def __call__(self):
        """Create and return the animation."""
        anim = FuncAnimation(
            self.fig, self.update, frames=self.paths[0].shape[1],
            init_func=self.initialize_figure, blit=False, interval=30,
            repeat=False
        )
        plt.show()
        return anim