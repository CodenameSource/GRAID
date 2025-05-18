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
            proximity_threshold_merging: float = .1,
            grids: List[ObservationGrid] = None,
            geo_origin: Dict[str, float] = None,
            heading_origin: float = 0,
            drone: DroneExtended = None,
            mission_name: str = "",
            mission_metadata: Dict[str, Any] = None
    ):
        self.grid_size = grid_size
        self.node_spacing = node_spacing
        self.proximity_threshold_merging = proximity_threshold_merging
        self.grids = grids or []
        self.geo_origin = geo_origin
        self.heading_origin = heading_origin
        self.altitude_origin = drone.get_location()['alt']
        self.drone = drone
        self.mission_name = mission_name
        self.mission_metadata = mission_metadata or {}
        self.last_drone_location = None

    def drone_location(self):
        self.last_drone_location = self.drone.get_location()

        return self.last_drone_location

    def drone_position(self):
        """
        Get the drone position in the observation space.

        Returns:
        - Tuple[float, float]: The drone position.
        """
        drone_location = self.last_drone_location
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

        def round_pos(pos: tuple) -> tuple:
            return round(pos[0], 2), round(pos[1], 2)

        grids = self.map_observation(observation)
        observation.rotate_obs_graph(-observation.rotation[1])
        if grids:
            obs_pos = [round_pos(observation.graph.nodes[node]['pos']) for node in observation.graph.nodes]
            for grid in grids:
                grid_pos = [round_pos(grid.graph.nodes[node]['pos']) for node in grid.graph.nodes]
                cut_pos = [node for node in obs_pos if node not in grid_pos]
                grid.add_observation(observation)
        else:
            raise Exception("No grids mapped for this observation. This should not happen!")
            # origins = self.map_grid_origins(observation)

    #
    # for origin in origins:
    #    geo_origin = self.geo_compute(self.geo_origin["lat"], self.geo_origin["lon"], origin[0], origin[1])
    #    new_grid = ObservationGrid(self.grid_size, self.node_spacing, origin=origin, geo_origin=geo_origin)
    #    new_grid.add_observation(observation)
    #    self.grids.append(new_grid)

    def map_grid_origins(self, observation: ObservationGraph) -> [ObservationGrid]:
        """
        Maps an observation to the appropriate observation grid(s) and returns the grid(s) origin(s).

        Args:
        - observation (Any): The observation to map.

        Returns:
        - origin tuple(float, float): The origin of the grid containing the observation.
        """

        def map_origin(position):
            x_grids_index = math.floor(position[0] / int((self.grid_size * self.node_spacing) / 2))
            y_grids_index = math.floor(position[1] / int(self.grid_size * self.node_spacing))

            if position[0] < 0:
                x_grids_index += 1

            return x_grids_index * (self.grid_size * self.node_spacing), y_grids_index * (
                    self.grid_size * self.node_spacing)

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
        grid_origins = self.map_grid_origins(observation)
        mapped_grids = []

        for origin in grid_origins:
            origin_mapped = False
            for grid in self.grids:
                if grid.origin == origin:
                    mapped_grids.append(grid)
                    origin_mapped = True

            if not origin_mapped:
                geo_origin = self.geo_compute(self.geo_origin["lat"], self.geo_origin["lon"], origin[0], origin[1])
                new_grid = ObservationGrid(self.grid_size, self.node_spacing, origin=origin, geo_origin=geo_origin,
                                           proximity_threshold_merging=self.proximity_threshold_merging)
                self.grids.append(new_grid)
                mapped_grids.append(new_grid)

        return mapped_grids

    def snap_to_node_spacing(self, position):
        return self.__snap_to_node_spacing(position, self.node_spacing)

    @staticmethod
    def __snap_to_node_spacing(position, node_spacing):
        return round(position[0] / node_spacing) * node_spacing, round(position[1] / node_spacing) * node_spacing

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
        # TODO: Remove and implement more permanent fix
        lat1 = round(lat1, 8)
        lat2 = round(lat2, 8)

        lon1 = round(lon1, 8)
        lon2 = round(lon2, 8)

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
        # Earth radius in meters (WGS84)
        R = 6378137.0

        # Convert degrees to radians
        lat1_rad, lon1_rad = map(math.radians, [lat1, lon1])

        # Compute the destination point in radians
        lat2_rad = lat1_rad + forward / R
        lon2_rad = lon1_rad + right / (R * math.cos(lat1_rad))

        # Convert radians back to degrees
        lat2, lon2 = map(math.degrees, [lat2_rad, lon2_rad])

        return lat2, lon2

    def get_working_grid(self, source_point, target_point, working_grid_padding=5) -> (int, ObservationGrid):
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
                                                       self.node_spacing, working_grid_padding,
                                                       self.proximity_threshold_merging)

    @staticmethod
    def _get_merged_grid(source_point, target_point, grids, grid_size, node_spacing, padding=5,
                         proximity_threshold_merging=.1) -> (
    int, ObservationGrid):
        def get_chunk_borders(target_point, padding):
            """
            Get the chunk borders around the target point with a given padding.
            """

            chunk_borders = {
                "top": round(target_point[1], 1) + padding,
                "bottom": round(target_point[1], 1) - padding,
                "left": round(target_point[0], 1) - padding,
                "right": round(target_point[0], 1) + padding
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

        if distance + 2 * padding > source_point_grid.node_spacing * source_point_grid.grid_size:  #Return no merged grid if the distance between the points is too large. Done to keep the size of the grids standartised
            return None

        source_point_chunk = get_chunk_borders(source_point, padding)
        target_point_chunk = get_chunk_borders(target_point, padding)

        working_grid_borders["top"] = max(source_point_chunk["top"], target_point_chunk["top"])
        working_grid_borders["bottom"] = min(source_point_chunk["bottom"], target_point_chunk["bottom"])
        working_grid_borders["left"] = min(source_point_chunk["left"], target_point_chunk["left"])
        working_grid_borders["right"] = max(source_point_chunk["right"], target_point_chunk["right"])

        origin = (
            ObservationSpace.__snap_to_node_spacing((working_grid_borders["left"] + (
                        (working_grid_borders["right"] - working_grid_borders["left"]) / 2), 0), node_spacing)[1],
            working_grid_borders["bottom"])

        working_grid = ObservationGrid(grid_size, node_spacing, origin=origin,
                                       proximity_threshold_merging=proximity_threshold_merging)

        for grid in grids:
            working_grid = copy_grid(grid, working_grid)  #TODO: Check for overlapping positions here

        return working_grid
