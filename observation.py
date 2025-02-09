import matplotlib.pyplot as plt
import networkx as nx
import numpy as np


class ObservationGraph:
    def __init__(self, fov_degrees=90, max_depth_m=10.0, node_spacing=0.1, curve_scaling=0.5, obs_pixel_width=1280,
                 origin=(0, 0), geo_origin=None, rotation=(0, 0, 0)):
        """
        Creates a 2D depth graph with nodes expanding in a curved FOV-like fashion. Obstacles added separately.
        :param fov_degrees: The maximum field of view angle in degrees.
        :param max_depth_m: The maximum observable distance (in meters).
        :param node_spacing: Distance between nodes (in meters).
        :param curve_scaling: A scaling factor for curve shape. Smaller values create more gradual curves.
        :param obs_pixel_width: Width of the observation image in pixels
        :param origin: (x, y), where x is meters forward to the observation space origin and y is meters right
        :param geo_origin: (latitude, longitude)
        :param rotation: (pitch, yaw, roll)
        """

        self.fov_degrees = fov_degrees
        self.max_depth_m = max_depth_m
        self.node_spacing = node_spacing
        self.curve_scaling = curve_scaling
        self.graph = self.create_curved_fov_graph()
        self.max_depth = int(max_depth_m / node_spacing)
        self.max_width = int(self.projected_image_width(max_depth_m) / node_spacing)
        self.obs_pixel_width = obs_pixel_width

        self.origin = origin
        self.geo_origin = geo_origin
        self.rotation = rotation

        self.vertical_layers = []

    def projected_image_width(self, distance):
        """
        Calculates the projected image width at a given distance based on the horizontal FoV.

        Parameters:
        - distance: The distance the camera has traveled (in meters).

        Returns:
        - float: The projected image width at the given distance (in meters).
        """
        # Convert FOV from degrees to radians
        fov_radians = np.radians(self.fov_degrees)

        # Calculate the projected width using the formula
        projected_width = 2 * distance * np.tan(fov_radians / 2)

        return projected_width

    def create_curved_fov_graph(self):
        _width_coeff = self.projected_image_width(self.max_depth_m) / self.projected_image_width(
            self.max_depth_m ** self.curve_scaling)

        if self.fov_degrees >= 180:
            raise ValueError("Field of view must be less than 180 degrees.")

        depth_nodes = int(self.max_depth_m / self.node_spacing)  # Nodes along the depth axis (10 cm steps)
        graph = nx.Graph()

        # Iterate through each depth level
        for depth in range(1, depth_nodes + 1):
            # Current depth in meters
            current_depth_m = depth * self.node_spacing
            # Apply a non-linear scaling to the FOV expansion with depth
            scaled_depth = current_depth_m ** self.curve_scaling  # Non-linear scaling for curve effect
            projected_width = self.projected_image_width(scaled_depth) * _width_coeff  # Projected width at this depth

            # Number of nodes across at this depth level
            width_nodes = max(1, int(projected_width / self.node_spacing))

            # Adding nodes across the width, centered around x = 0
            for x in range(-width_nodes // 2, width_nodes // 2 + 1):
                node_id = (depth, x)
                graph.add_node(node_id, pos=(x * self.node_spacing, current_depth_m), occupancy=0, vertical_bitmap=0)

                # Connect to nodes in the same depth level (horizontal connections)
                if x > -width_nodes // 2:
                    left_node_id = (depth, x - 1)
                    if left_node_id in graph:
                        graph.add_edge(node_id, left_node_id)

                # Connect to nodes in the previous depth level (vertical connections)
                if depth > 1:
                    prev_depth = depth - 1
                    for dx in range(-1, 2):  # Connect to adjacent nodes
                        prev_node_id = (prev_depth, x + dx)
                        if prev_node_id in graph:
                            graph.add_edge(node_id, prev_node_id)

        return graph

    def rotate_obs_graph(self, angle_degrees):
        """
        Rotates a graph around a given origin.

        Parameters:
        - graph: The NetworkX graph to rotate.
        - angle_degrees: The rotation angle in degrees.
        """
        self.rotate_graph(self.graph, angle_degrees, origin=self.origin)

    def width_at_depth(self, depth):
        """
        Returns the number of nodes at a specific depth in the graph.

        Parameters:
        - graph: The graph created by create_curved_fov_graph.
        - depth: The depth level to check.

        Returns:
        - int: The number of nodes at the specified depth.
        """
        # Extract nodes at the given depth (nodes are stored as tuples (depth, x))
        nodes_at_depth = [node for node in self.graph.nodes if node[0] == depth]
        return len(nodes_at_depth)

    def kernel_to_graph_coord(self, depth_m, x_m):
        """
        Converts kernel coordinate to graph coordinate.

        Parameters:
        - x: Kernel coordinate from the left border of the image
        - depth: Depth level.

        Returns:
        - tuple: The graph node (depth, x) representing the kernel coordinate.
        """
        nearest_depth = int(np.floor(depth_m / self.node_spacing))
        nearest_depth_width = self.width_at_depth(nearest_depth)

        level_width_m = nearest_depth_width * self.node_spacing

        nearest_x = round((x_m - level_width_m / 2) / self.node_spacing)

        return nearest_depth, nearest_x

    def px_to_graph_coord(self, depth_m, px):
        """
        Converts pixel coordinate to graph coordinate.

        Parameters:
        - px: Pixel coordinate.
        - depth: Depth level.

        Returns:
        - float: Graph coordinate.
        """
        nearest_depth = int(np.floor(depth_m / self.node_spacing))
        nearest_depth_width = self.width_at_depth(nearest_depth)

        coord_range = (self.projected_image_width(depth_m) / self.node_spacing) / 2
        depth_coeff = coord_range / (self.obs_pixel_width / 2)

        nearest_x = round((px - self.obs_pixel_width / 2) * depth_coeff)

        return nearest_depth, nearest_x

    def place_target_point(self, forward_m, right_m):
        """
        Places a target point in the obstacle graph at a specific location.

        Parameters:
        - forward_m: Forward distance (meters) from the starting point (depth axis).
        - right_m: Rightward distance (meters) from the center (x-axis).

        Returns:
        - tuple: The graph node (depth, x) representing the target point.
        """
        # Convert distances to graph coordinates
        depth = round(forward_m / self.node_spacing)
        x_coord = round(right_m / self.node_spacing)

        # Define the target node
        target_node = (depth, x_coord)

        # Check if the node exists in the graph
        if target_node in self.graph:
            # Mark the node as a target (e.g., occupancy = 3)
            self.graph.nodes[target_node]['occupancy'] = 128
            print(f"Target point placed at {target_node} (Forward: {forward_m:.2f}m, Right: {right_m:.2f}m)")
        else:
            print(f"Target point {target_node} is outside the graph bounds.")

        return target_node

    def set_obstacle_raw(self, depth_start_m, depth_end_m, x_start_px, x_end_px):
        """
        Sets obstacle nodes within specified depth and x ranges, and calculates view occlusion behind them.

        Parameters:
        - depth_start_m, depth_end_m: The range of depths (in meters) to place obstacles.
        - x_start_px, x_end_px: The range of x coordinates (in pixels) to place obstacles.
        """
        depth_start, x_start = self.px_to_graph_coord(depth_start_m, x_start_px)
        depth_end, x_end = self.px_to_graph_coord(depth_end_m, x_end_px)
        self.set_obstacle(depth_start, depth_end, x_start, x_end)

    def set_obstacle(self, depth_start, depth_end, x_start, x_end, vertical_bitmap=0, depth_level=None):
        """
        Sets obstacle nodes within specified depth and x ranges, and calculates view occlusion behind them.

        Parameters:
        - depth_start, depth_end: The range of depths to place obstacles.
        - x_start, x_end: The range of x coordinates to place obstacles.
        - vertical_bitmap: The vertical bitmap of the obstacle. (Optional)
        - depth_level: The depth level of the obstacle. (Optional)
        """

        # Step 1: Set specified range of nodes to obstacle state
        for depth in range(depth_start, depth_end + 1):
            for x in range(x_start, x_end + 1):
                node_id = (depth, x)
                if node_id in self.graph:
                    if vertical_bitmap is not None:
                        if vertical_bitmap & depth_level[1]:
                            self.graph.nodes[node_id]['occupancy'] |= 2
                        if vertical_bitmap & depth_level[2]:
                            self.graph.nodes[node_id]['occupancy'] |= 4
                        self.graph.nodes[node_id]['vertical_bitmap'] |= vertical_bitmap
                    self.graph.nodes[node_id]['occupancy'] |= 1  # Set as obstacle

        # Step 2: Calculate occlusion for nodes behind the entire obstacle range
        self.apply_occlusion(depth_start, depth_end, x_start, x_end)

    def apply_occlusion(self, depth_start, depth_end, x_start, x_end):
        """
        Marks nodes behind a specified obstacle range as occluded based on FOV and obstacle position.

        Parameters:
        - depth_start, depth_end: Depth range of the obstacle nodes.
        - x_start, x_end: X-coordinate range of the obstacle nodes.
        """
        # Calculate the angles for the left and right bounds of the obstacle
        left_angle = np.arctan2((x_start * self.node_spacing), (depth_start * self.node_spacing))
        right_angle = np.arctan2((x_end * self.node_spacing), (depth_start * self.node_spacing))

        # Step through each depth level behind the obstacle range
        for depth in range(depth_end + 1, int(self.max_depth_m / self.node_spacing) + 1):
            # Calculate the depth in meters
            depth_m = depth * self.node_spacing

            # Calculate the occluded x-range at this depth
            x_occluded_min = int((np.tan(left_angle) * depth_m) / self.node_spacing)
            x_occluded_max = int((np.tan(right_angle) * depth_m) / self.node_spacing)

            # Iterate over the nodes in the occluded range and set them as occluded
            for x in range(x_occluded_min, x_occluded_max + 1):
                node_id = (depth, x)
                if node_id in self.graph and self.graph.nodes[node_id].get('occupancy', 0) == 0:
                    self.graph.nodes[node_id]['occupancy'] = 256  # Set as occluded

    def draw_graph(self):
        """
        Draws the 2D depth graph with curved field of view.
        Nodes are colored based on occupancy:
        - Blue for empty (value 0)
        - Red for obstacle (value 1)
        - Yellow for unknown (value 2)
        """
        # Get positions for plotting
        positions = nx.get_node_attributes(self.graph, 'pos')

        # Get the 'value' attribute for each node to determine colors
        node_values = nx.get_node_attributes(self.graph, 'occupancy')
        node_colors = [
            'blue' if node_values.get(node, 0) == 0 else
            'red' if node_values.get(node, 0) == 1 else
            'yellow' for node in self.graph.nodes
        ]

        # Plot the graph
        plt.figure(figsize=(12, 10))
        nx.draw(self.graph, pos=positions, node_size=5, node_color=node_colors, with_labels=False, edge_color="gray")
        plt.title("2D Depth Graph with Curved Field of View and Occupancy Colors")
        plt.xlabel("Width (m)")
        plt.ylabel("Depth (m)")
        # plt.gca().invert_yaxis()  # Invert y-axis for better visualization
        plt.show()

    @staticmethod
    def rotate_graph(graph, rotation_degrees, origin=(0, 0)):
        angle_radians = np.radians(rotation_degrees)
        cos_angle = np.cos(angle_radians)
        sin_angle = np.sin(angle_radians)

        new_positions = {}
        for node, (x, y) in nx.get_node_attributes(graph, 'pos').items():
            x_shifted, y_shifted = x, y
            x_rotated = (x_shifted * cos_angle - y_shifted * sin_angle) + origin[0]
            y_rotated = (x_shifted * sin_angle + y_shifted * cos_angle) + origin[1]

            new_positions[node] = (x_rotated, y_rotated)

        nx.set_node_attributes(graph, new_positions, 'pos')
