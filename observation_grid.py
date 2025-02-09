import matplotlib.pyplot as plt
import networkx as nx
import numpy as np
from scipy.spatial import distance

from observation import ObservationGraph


class ObservationGrid:
    """
    A class to represent an observation grid for a drone.
    The grid is represented as a NetworkX graph with nodes and edges.

    Attributes:
    - graph (networkx.Graph): The NetworkX graph representing the observation grid.
    - grid_size (int): The size of the grid (number of nodes along one side).
    - node_spacing (float): The distance between adjacent nodes in meters.
    - origin (Tuple[float, float]): The origin of the grid in meters.
    - geo_origin (Tuple[float, float]): The origin of the grid in GPS coordinates.
    - proximity_threshold_merging (float): The threshold for merging nodes based on proximity.
    """

    def __init__(self, grid_size, node_spacing, origin=(0, 0), geo_origin=None, proximity_threshold_merging=.5):
        self.graph = self.__generate_chunk(grid_size, node_spacing)
        self.grid_size = grid_size
        self.node_spacing = node_spacing
        self.origin = origin  # Origin of the grid in meters
        self.geo_origin = geo_origin  # Origin of the grid in GPS coordinates
        self.proximity_threshold_merging = proximity_threshold_merging

    def add_observation(self, obs_graph: ObservationGraph):
        obs_graph.rotate_obs_graph(obs_graph.rotation[1])

        tmp_graph = obs_graph.graph.copy()

        # Get positions from both graphs
        square_positions = np.array(list(nx.get_node_attributes(self.graph, 'pos').values()))
        fov_positions = np.array(list(nx.get_node_attributes(tmp_graph, 'pos').values()))

        # Extract node occupancy from both graphs
        fov_occupancies = [tmp_graph.nodes[node].get('occupancy', 0) for node in tmp_graph.nodes]

        # Skip FOV nodes with occupancy == 0 (if they are not of interest)
        fov_positions_of_interest = fov_positions  # fov_positions[np.array(fov_occupancies) > 0]
        fov_occupancies_of_interest = fov_occupancies  # np.array(fov_occupancies)[np.array(fov_occupancies) > 0]

        # Compute pairwise distances between FOV and square node positions
        distances = distance.cdist(fov_positions_of_interest, square_positions, 'sqeuclidean')

        # Iterate over FOV nodes of interest and find the closest square node
        for fov_idx, fov_pos in enumerate(fov_positions_of_interest):
            fov_occupancy = fov_occupancies_of_interest[fov_idx]

            # Find the closest square node
            closest_square_idx = np.argmin(distances[fov_idx])
            closest_distance = distances[fov_idx, closest_square_idx]

            if closest_distance <= self.proximity_threshold_merging:
                # Get the corresponding square node
                square_node = list(self.graph.nodes)[closest_square_idx]
                square_occupancy = self.graph.nodes[square_node].get('occupancy', 0)

                if square_occupancy == 256 or fov_occupancy & 1:
                    self.graph.nodes[square_node]['occupancy'] = fov_occupancy

        return self.graph

    def index_pos(self, pos, origin_pos=None):  # TODO Address floating point division weirdness
        """
        Converts a position in meters to a grid index.
        :param pos:
        :param origin_pos:
        :return:
        """

        if not origin_pos:
            origin_pos = self.origin

        x_offset, y_offset = -(self.grid_size // 2) * self.node_spacing, 0

        x_delta = pos[1] - origin_pos[0]
        y_delta = pos[0] - origin_pos[1]

        x_idx = int((x_delta - x_offset) / self.node_spacing)
        y_idx = int((y_delta - y_offset) / self.node_spacing)

        return x_idx, y_idx

    def extents_relative(self):
        """
        Returns the relative extents of the grid in meters.

        Returns:
        - tuple: (width, height) of the grid in meters.
        """

        width = self.grid_size * self.node_spacing
        height = self.grid_size * self.node_spacing

        extents = {
            'width': width,
            'height': height,
            'left': self.origin[0] - width / 2,
            'right': self.origin[0] + width / 2,
            'bottom': self.origin[1],
            'top': self.origin[1] + height
        }

        return extents

    def extents_geo(self):
        """
        Returns the geographical extents of the grid in GPS coordinates.

        Returns:
        - tuple: (longitude, latitude) of the grid origin.
        """

        def add_to_geo_coordinates_meter(meter_x, meter_y, origin_lat):
            earth_radius = 6378137.0
            meter_x = meter_x / earth_radius
            meter_y = meter_y / earth_radius
            lat = origin_lat + (meter_y * (180 / np.pi))
            lon = meter_x / np.cos(origin_lat * np.pi / 180) + origin_lat
            return lon, lat

        if self.geo_origin is None:
            raise ValueError("Geographical origin not set for the grid.")

        relative_extents = self.extents_relative()

        geo_extents = {
            'width': relative_extents['width'],
            'height': relative_extents['height'],
            'left': add_to_geo_coordinates_meter(relative_extents['left'], 0, self.geo_origin[1]),
            'right': add_to_geo_coordinates_meter(relative_extents['right'], 0, self.geo_origin[1]),
            'bottom': add_to_geo_coordinates_meter(0, relative_extents['bottom'], self.geo_origin[1]),
            'top': add_to_geo_coordinates_meter(0, relative_extents['top'], self.geo_origin[1])
        }

        return geo_extents

    def is_relative_point_in_grid(self, point):
        """
        Checks if a relative point is within the grid.

        Args:
        - point (tuple): The relative point to check.

        Returns:
        - bool: True if the point is within the grid, False otherwise.
        """

        extents = self.extents_relative()
        return extents['left'] <= point[0] <= extents['right'] and extents['bottom'] <= point[1] <= extents['top']

    def is_geo_point_in_grid(self, point):
        """
        Checks if a geographical point is within the grid.

        Args:
        - point (tuple): The geographical point to check.

        Returns:
        - bool: True if the point is within the grid, False otherwise.
        """
        extents = self.extents_geo()
        return extents['left'][0] <= point[0] <= extents['right'][0] and extents['bottom'][1] <= point[1] <= \
            extents['top'][1]

    def visualize(self):
        positions = nx.get_node_attributes(self.graph, 'pos')

        # Ensure node_colors matches the number of nodes in the graph
        node_colors = []

        for node in self.graph.nodes:
            occupancy = self.graph.nodes[node].get('occupancy', 0)
            if occupancy == 0:
                node_colors.append('blue')  # Free space
            elif occupancy == 3:
                node_colors.append('yellow')
            elif occupancy == 5:
                node_colors.append('orange')
            elif occupancy == 128:
                node_colors.append('purple')  # Target
            elif occupancy == 256:
                node_colors.append('gray')  # Occlusion
            else:
                node_colors.append('red')  # Obstacle

        # Ensure that positions and node_colors have the same length
        if len(node_colors) != len(self.graph.nodes):
            raise ValueError(
                f"Inconsistent node colors: {len(node_colors)} colors for {len(self.graph.nodes)} nodes."
            )

        # Plot the graph
        plt.figure(figsize=(12, 10))
        nx.draw(
            self.graph,
            pos=positions,
            node_size=5,
            node_color=node_colors,
            with_labels=False,
            edge_color="gray"
        )
        plt.title("Obstacle Graph Visualization")
        plt.xlabel("Width (m)")
        plt.ylabel("Depth (m)")
        plt.show()

    @staticmethod
    def rotate_obs_graph(graph, angle_degrees, origin=(0, 0)):
        """
        Rotates a graph around a given origin.

        Parameters:
        - graph: The NetworkX graph to rotate.
        - angle_degrees: The rotation angle in degrees.
        - origin: The origin point for rotation (default is (0, 0)).
        """
        angle_radians = np.radians(angle_degrees)
        cos_angle = np.cos(angle_radians)
        sin_angle = np.sin(angle_radians)

        new_positions = {}
        for node, (x, y) in nx.get_node_attributes(graph, 'pos').items():
            x_shifted, y_shifted = x, y
            x_rotated = (x_shifted * cos_angle - y_shifted * sin_angle) + origin[0]
            y_rotated = (x_shifted * sin_angle + y_shifted * cos_angle) + origin[1]

            new_positions[node] = (x_rotated, y_rotated)

        nx.set_node_attributes(graph, new_positions, 'pos')

    @staticmethod
    def __generate_chunk(grid_size=10, node_spacing=0.5):
        """
        Generates a square graph with a given grid size and node spacing.
        The origin (0, 0) is positioned at the bottom middle of the grid.

        Parameters:
        - grid_size (int): Number of nodes along one side of the square grid.
        - node_spacing (float): Distance between adjacent nodes in meters.

        Returns:
        - self.graph (networkx.Graph): The generated square graph with node positions.
        """
        # Create a 2D grid graph
        chunk_graph = nx.grid_2d_graph(grid_size, grid_size)

        # Calculate offsets to shift the origin to the bottom middle
        x_offset = -(grid_size // 2) * node_spacing  # Center x-coordinates horizontally
        y_offset = 0  # Bottom row starts at y=0

        # Assign positions and other attributes to the nodes
        node_attributes = {
            (x, y): {
                'pos': (x * node_spacing + x_offset, y * node_spacing + y_offset),
                'occupancy': 256  # Default occupancy value
            }
            for x, y in chunk_graph.nodes
        }
        nx.set_node_attributes(chunk_graph, node_attributes)

        return chunk_graph


def copy_grid(grid1: ObservationGrid, grid2: ObservationGrid):
    """
    Copies node attributes from graph1 to graph2 based on positional similarity.

    Parameters:
    - graph1: The source NetworkX graph with attributes.
    - graph2: The target NetworkX graph to copy attributes to.

    Returns:
    - None (modifies graph2 in place)
    """
    graph1 = grid1.graph
    graph2 = grid2.graph

    pos1 = nx.get_node_attributes(graph1, 'pos')
    occupancy1 = nx.get_node_attributes(graph1, 'occupancy')
    pos2 = nx.get_node_attributes(graph2, 'pos')

    graph2_attributes = {}

    for node2, pos_value2 in pos2.items():
        if pos_value2 in pos1.values():
            pos1_index = list(pos1.values()).index(pos_value2)
            occupancy1_value = occupancy1[list(pos1.keys())[pos1_index]]
            graph2_attributes[node2] = {"pos": pos_value2, "occupancy": occupancy1_value}

    nx.set_node_attributes(graph2, graph2_attributes)
