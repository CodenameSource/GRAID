import math
from math import radians, cos
from typing import List, Dict, Any

import networkx as nx

from imagery import DroneExtended
from observation import ObservationGraph
from observation_grid import ObservationGrid, copy_grid


class ObservationSpace:
    """
    A space for storing and managing observations.

    Attributes:
    - grid_size (int): The size of the observation grid.
    - grids (List[ObservationGrid]): The observation grids.
    - geo_origin (Dict[str, float]): The geographic origin of the observation space.
    - heading_origin (float): The heading origin of the observation space.
    - drone (DroneExtended): The drone used to collect observations.
    - mission_name (str): The name of the mission.
    - mission_metadata (Dict[str, Any]): The metadata of the mission.
    """

    def __init__(
            self,
            grid_size: int,
            node_spacing: float,
            grids: List[ObservationGrid] = None,
            geo_origin: Dict[str, float] = None,
            heading_origin: float = 0,
            drone: DroneExtended = None,
            mission_name: str = "",
            mission_metadata: Dict[str, Any] = None
    ):
        self.grid_size = grid_size
        self.node_spacing = node_spacing
        self.grids = grids or []
        self.geo_origin = geo_origin
        self.heading_origin = heading_origin
        self.drone = drone
        self.mission_name = mission_name
        self.mission_metadata = mission_metadata or {}

    def drone_position(self):
        """
        Get the drone position in the observation space.

        Returns:
        - Tuple[float, float]: The drone position.
        """
        drone_location = self.drone.get_location()
        drone_geo_pos = {"lat": drone_location["lat"], "lon": drone_location["lon"]}

        if self.geo_origin:
            forward, right = self.geo_offset(
                self.geo_origin["lat"],
                self.geo_origin["lon"],
                drone_geo_pos["lat"],
                drone_geo_pos["lon"]
            )
            drone_pos = (forward, right)

            return drone_pos
        else:
            return None

    def add(self, observation: ObservationGraph) -> None:
        """
        Adds an observation to the appropriate observation grid.

        Args:
        - observation (Any): The observation to add.
        """
        grids = self.map_observation(observation)
        if grids:
            for grid in grids:
                grid.add_observation(observation)
        else:
            origins = self.map_grid_origins(observation)

            for origin in origins:
                geo_origin = self.geo_compute(self.geo_origin["lat"], self.geo_origin["lon"], origin[0], origin[1])
                new_grid = ObservationGrid(self.grid_size, self.node_spacing, origin=origin, geo_origin=geo_origin)
                new_grid.add_observation(observation)
                self.grids.append(new_grid)

    def map_grid_origins(self, observation: ObservationGraph) -> [ObservationGrid]:
        """
        Maps an observation to the appropriate observation grid(s) and returns the grid(s) origin(s).

        Args:
        - observation (Any): The observation to map.

        Returns:
        - origin tuple(float, float): The origin of the grid containing the observation.
        """

        def map_origin(position):
            x_grids_index = int(position[0] / int(self.grid_size / 2))
            y_grids_index = position[1] / self.grid_size

            return x_grids_index * (self.grid_size / self.node_spacing), y_grids_index * (
                    self.grid_size / self.node_spacing)

        temp_graph = observation.rotate_graph(observation.graph.copy(), observation.rotation[1] - self.heading_origin,
                                              observation.origin)

        positions = nx.get_node_attributes(temp_graph, 'pos').values()

        origins = []
        for position in positions:
            origins.append(map_origin(position))

        unique_origins = list(set(origins))

        return unique_origins

    def map_observation(self, observation: ObservationGraph) -> [ObservationGrid]:
        """
        Maps an observation to the appropriate observation grid.

        Args:
        - observation (Any): The observation to map.

        Returns:
        - ObservationGrid: The grid containing the observation.
        """
        return [grid for grid in self.grids if grid.is_relative_point_in_grid(observation.origin)]

    @staticmethod
    def geo_offset(lat1, lon1, lat2, lon2):
        """
        Compute the forward (north-south) and right (east-west) offset in meters
        from point (lat1, lon1) to point (lat2, lon2).

        :param lat1: Latitude of the starting point (degrees)
        :param lon1: Longitude of the starting point (degrees)
        :param lat2: Latitude of the destination point (degrees)
        :param lon2: Longitude of the destination point (degrees)
        :return: (forward, right) distances in meters
        """
        # Earth radius in meters
        R = 6378137.0

        # Convert degrees to radians
        lat1, lon1, lat2, lon2 = map(radians, [lat1, lon1, lat2, lon2])

        # Forward offset (difference in latitude)
        dlat = lat2 - lat1
        forward = dlat * R

        # Right offset (difference in longitude, corrected by latitude)
        dlon = lon2 - lon1
        right = dlon * R * cos(lat1)

        return forward, right

    @staticmethod
    def geo_compute(lat1, lon1, forward, right):
        """
        Compute the destination point (lat2, lon2) in degrees given the starting point (lat1, lon1) and
        the forward and right offsets in meters.

        :param lat1: Latitude of the starting point (degrees)
        :param lon1: Longitude of the starting point (degrees)
        :param forward: Forward offset in meters
        :param right: Right offset in meters
        :return: (lat2, lon2) destination point in degrees
        """
        # Earth radius in meters
        R = 6378137.0

        # Convert degrees to radians
        lat1, lon1 = map(radians, [lat1, lon1])

        # Compute the destination point
        lat2 = lat1 + forward / R
        lon2 = lon1 + right / (R * cos(lat1))

        # Convert radians to degrees
        lat2, lon2 = map(radians, [lat2, lon2])

        return lat2, lon2

    def get_working_grid(self, source_point, target_point) -> (int, ObservationGrid):
        """
        Get the working grid for the source and target points.

        If the source and target points are in the same grid, return False and the grid.
        If the source and target points are in different grids, return True and the merged grid.

        :param source_point:
        :param target_point:
        :return: bool, working grid
        """
        grids = self.grids

        for grid in grids:
            if grid.is_relative_point_in_grid(source_point) and grid.is_relative_point_in_grid(target_point):
                return False, grid

        return True, ObservationSpace._get_merged_grid(source_point, target_point, grids, self.grid_size,
                                                       self.node_spacing)

    @staticmethod
    def _get_merged_grid(source_point, target_point, grids, grid_size, node_spacing, padding=5) -> (
    int, ObservationGrid):
        def get_chunk_borders(target_point, padding):
            """
            Get the chunk borders around the target point with a given padding.
            """

            chunk_borders = {
                "top": target_point[1] + padding,
                "bottom": target_point[1] - padding,
                "left": target_point[0] - padding,
                "right": target_point[0] + padding
            }

            return chunk_borders

        def point_distance(point1, point2):
            return math.sqrt((point1[0] - point2[0]) ** 2 + (point1[1] - point2[1]) ** 2)

        working_grid_borders = {
            "top": None,
            "bottom": None,
            "left": None,
            "right": None
        }

        source_point_grid = None
        target_point_grid = None

        for grid in grids:
            if grid.is_relative_point_in_grid(source_point):
                source_point_grid = grid
            if grid.is_relative_point_in_grid(target_point):
                target_point_grid = grid

            if source_point_grid and target_point_grid:
                break

        distance = point_distance(source_point, target_point)

        if distance + 2 * padding < source_point_grid.node_spacing * source_point_grid.grid_size:
            return None

        source_point_chunk = get_chunk_borders(source_point, padding)
        target_point_chunk = get_chunk_borders(target_point, padding)

        working_grid_borders["top"] = max(source_point_chunk["top"], target_point_chunk["top"])
        working_grid_borders["bottom"] = min(source_point_chunk["bottom"], target_point_chunk["bottom"])
        working_grid_borders["left"] = min(source_point_chunk["left"], target_point_chunk["left"])
        working_grid_borders["right"] = max(source_point_chunk["right"], target_point_chunk["right"])

        origin = (
            working_grid_borders["left"] + int((working_grid_borders["right"] - working_grid_borders["left"]) / 2),
            working_grid_borders["top"])

        working_grid = ObservationGrid(grid_size, node_spacing, origin=origin, proximity_threshold_merging=.5)

        for grid in grids:
            if working_grid_borders["left"] <= grid.origin[0] <= working_grid_borders["right"] and \
                    working_grid_borders["bottom"] <= grid.origin[1] <= working_grid_borders["top"]:
                copy_grid(grid, working_grid)

        return working_grid
