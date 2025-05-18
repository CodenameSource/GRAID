import matplotlib.pyplot as plt
import networkx as nx
import numpy as np
from scipy.spatial import cKDTree

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
        self.graph = self.__generate_chunk(grid_size, node_spacing, origin)
        self.grid_size = grid_size
        self.node_spacing = node_spacing
        self.origin = origin  # Origin of the grid in meters
        self.geo_origin = geo_origin  # Origin of the grid in GPS coordinates
        self.proximity_threshold_merging = proximity_threshold_merging

    def add_observation(self, obs_graph: "ObservationGraph") -> nx.Graph:
        """
        Project every node from obs_graph onto this grid—both obstacles and free space.
        Each observed node is assigned to the nearest unused grid cell (with fallback).
        - If obs occupancy != 0, we OR its bits into the grid cell.
        - If obs occupancy == 0 and the grid cell was unknown, we mark it free (0).

        Returns the updated self.graph.
        """
        # Build KD‑tree over grid node positions
        grid_nodes = list(self.graph.nodes)
        positions = np.array([self.graph.nodes[n]["pos"] for n in grid_nodes])
        tree = cKDTree(positions)
        total = len(grid_nodes)

        # Determine the “unknown” default occupancy (initial grid state)
        default_unknown = next(iter(self.graph.nodes(data=True)))[1]["occupancy"]

        used = set()

        # Iterate every obs_graph node (including occupancy == 0)
        for _, data in obs_graph.graph.nodes(data=True):
            pos = data["pos"]
            focc = data.get("occupancy", 0)

            # Get all grid‐cell candidates sorted by distance
            dists, idxs = tree.query(pos, k=total, distance_upper_bound=np.inf)
            if total == 1:
                idxs = [idxs]

            # Find first unused grid index
            chosen = next((i for i in idxs if i < total and i not in used), None)
            # Fallback to absolute nearest if all used
            if chosen is None and idxs and idxs[0] < total:
                chosen = idxs[0]
            if chosen is None:
                continue

            used.add(chosen)
            gnode = grid_nodes[chosen]
            gocc = self.graph.nodes[gnode]["occupancy"]

            if focc:
                # merge obstacle bits
                self.graph.nodes[gnode]["occupancy"] = gocc | focc
            elif gocc == default_unknown:
                # mark free only if it was previously unknown
                self.graph.nodes[gnode]["occupancy"] = 0

        return self.graph

    def get_pos(self, node):
        try:
            return self.graph.nodes[node]['pos']
        except Exception:
            return None

    def index_by_pos(self, pos, origin_pos=None):
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
        - dict: {width, height, left, right, bottom, top}
        """
        if self.geo_origin is None:
            raise ValueError("Geographical origin not set for the grid.")

        # Relative extents in meters
        rel = self.extents_relative()

        # Helper: convert meter offsets to geo coords
        def add_to_geo_coordinates_meter(meter_x, meter_y, origin_lat, origin_lon):
            earth_radius = 6378137.0
            # convert meter offsets to radians
            rad_x = meter_x / earth_radius
            rad_y = meter_y / earth_radius
            # convert radians to degrees
            delta_lat = rad_y * (180.0 / np.pi)
            delta_lon = rad_x * (180.0 / np.pi) / np.cos(np.deg2rad(origin_lat))
            lat = origin_lat + delta_lat
            lon = origin_lon + delta_lon
            return {"lat": lat, "lon": lon}

        geo_extents = {
            'width': rel['width'],
            'height': rel['height'],
            'left': add_to_geo_coordinates_meter(rel['left'], 0, self.geo_origin['lat'], self.geo_origin['lon']),
            'right': add_to_geo_coordinates_meter(rel['right'], 0, self.geo_origin['lat'], self.geo_origin['lon']),
            'bottom': add_to_geo_coordinates_meter(0, rel['bottom'], self.geo_origin['lat'], self.geo_origin['lon']),
            'top': add_to_geo_coordinates_meter(0, rel['top'], self.geo_origin['lat'], self.geo_origin['lon'])
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
        return extents['bottom'] <= point[0] <= extents['top'] and extents['left'] <= point[1] <= extents['right']

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

    def visualize(self, name="observation_grid.png"):
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
        plt.savefig(name)

    def visualize_path(self, path, name="observation_grid_path.png", title="Path Visualization"):
        """
        Visualizes the obstacle grid with a given path overlaid.

        Parameters:
        - path (list): A list of nodes representing the path.
        - title (str): Title for the plot.
        """
        positions = nx.get_node_attributes(self.graph, 'pos')

        # Build node colors based on occupancy
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

        # Plot the base graph
        plt.figure(figsize=(12, 10))
        nx.draw(
            self.graph,
            pos=positions,
            node_size=5,
            node_color=node_colors,
            with_labels=False,
            edge_color="gray"
        )

        # Plot the path if one is provided
        if path:
            path_edges = list(zip(path[:-1], path[1:]))
            nx.draw_networkx_nodes(self.graph, pos=positions, nodelist=path, node_color='green', node_size=15)
            nx.draw_networkx_edges(self.graph, pos=positions, edgelist=path_edges, edge_color='green', width=2)

        plt.title(title)
        plt.xlabel("Width (m)")
        plt.ylabel("Depth (m)")
        plt.savefig(name)


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
    def __generate_chunk(grid_size=10, node_spacing=0.5, origin=(0, 0)):
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
        x_offset = -(grid_size // 2) * node_spacing + origin[0]
        y_offset = 0 + origin[1]

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

    def round_pos(pos) -> tuple:
        return round(pos[0], 2), round(pos[1], 2)


    graph1 = grid1.graph
    graph2 = grid2.graph

    pos_to_attrs = {round_pos(data.get('pos')): data for n, data in graph1.nodes(data=True)}

    for n2, data2 in graph2.nodes(data=True):
        pos2 = round_pos(data2.get('pos'))
        if pos2 in pos_to_attrs:
            graph2.nodes[n2].update(pos_to_attrs[pos2])

    grid2.graph = graph2
    return grid2
