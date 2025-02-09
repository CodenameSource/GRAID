import asyncio
import json
import time

import numpy as np

from Smav import Waypoint
from connection_config import ConnectionConfig
from depth_analysis import generate_projection_images, apply_kernels, bundle_kernels_to_larger_verticals, \
    map_obstacles_graph
from imagery import DroneExtended
from observation import ObservationGraph
from observation_grid import ObservationGrid
from observation_space import ObservationSpace
from pathfinding import find_path
from webapp import WebApp


class ObstacleAvoidance:
    """
    Class for handling obstacle avoidance operations.

    Attributes:
    - break_flag (bool): Flag to break the operation loop.
    - pause_execution (bool): Flag to pause the operation loop.
    - observation_space (ObservationSpace): The observation space.
    - drone (DroneExtended): The drone connection.
    - max_observation_depth (int): The maximum observation depth.
    - waypoints (List[Waypoint]): The mission waypoints.
    - waypoint_idx (int): The current waypoint index.
    """

    def __init__(self, max_observation_depth: int = 5):
        self.break_flag = False
        self.pause_execution = False
        self.observation_space = None
        self.max_observation_depth = max_observation_depth
        self.waypoints = []
        self.waypoint_idx = 0

        self.webapp = None

    def initialise_drone(self, connection_config: ConnectionConfig) -> None:
        """
        Initialises the drone connection.

        Args:
        - connection_config (ConnectionConfig): The connection configuration.
        """
        drone = DroneExtended(connection_config)
        drone_location = drone.get_location()
        heading_origin = drone_location["hdg"]
        geo_origin = {"lat": drone_location["lat"], "lon": drone_location["lon"]}
        initial_grid = ObservationGrid(200, .2, (0, 0), geo_origin, 0.1)
        self.observation_space = ObservationSpace(200, .2, [initial_grid], geo_origin, heading_origin, drone)

    def initialise_mission_waypoints(self, waypoints: [Waypoint]) -> None:
        """
        Initialises the mission waypoints.

        Args:
        - waypoints (List[Waypoint]): The mission waypoints.
        """
        self.waypoints = waypoints
        self.waypoint_idx = 0

    def observe(self):
        """
        Observes the environment.
        Steps:
        1. Get the drone location and heading.
        2. Generate depth images.
        3. Generate projection levels.
        4. Apply kernels.
        5. Bundle kernels to larger verticals.
        6. Map obstacles to the observation graph.

        Returns:
        - List[ObservationGraph]: The observation data.
        """
        observations = []

        drone_location = self.observation_space.drone.get_location()
        observation_hdg = drone_location["hdg"]
        observation_geo_origin = {"lat": drone_location["lat"], "lon": drone_location["lon"]}

        observation_origin = self.observation_space.geo_offset(self.observation_space.geo_origin["lat"],
                                                               self.observation_space.geo_origin["lon"],
                                                               drone_location["lat"], drone_location["lon"])

        for image in self.observation_space.drone.get_depth_images():
            horizontal_fov = image["config"].fov_horizontal
            vertical_fov = image["config"].fov_vertical
            heading_image = (observation_hdg + image["config"].rotation[1]) % 360

            projection_levels = generate_projection_images(image["images"][0], max_depth=self.max_observation_depth,
                                                           fov_horizontal=horizontal_fov, fov_vertical=vertical_fov)
            activations = apply_kernels(projection_levels, kernel_horizontal_size=self.observation_space.node_spacing,
                                        kernel_vertical_size=.1,
                                        fov_horizontal=horizontal_fov, fov_vertical=vertical_fov, percentile=10)
            activations = bundle_kernels_to_larger_verticals(activations, target_vertical_size=.5)

            observation_graph = ObservationGraph(horizontal_fov, self.max_observation_depth,
                                                 self.observation_space.node_spacing, .9,
                                                 activations[0]["projection_level"].shape[1], observation_origin,
                                                 observation_geo_origin, (0, heading_image, 0))

            observation_graph = map_obstacles_graph(observation_graph, activations)
            observations.append(observation_graph)
            # overlay_kernels_on_image(projection_levels) For debugging purposes

        return observations

    def check_activation(self, observation: ObservationGraph, padding: int = 7) -> (int, dict):
        """
        Determines if obstacle avoidance should be activated.

        Args:
        - observation (Any): The observation data.
        - padding (int): Padding distance for obstacle avoidance.

        Returns:
        - int: 0 for no activation required, 1 for activation required, 2 for blocked waypoint
        - dict: Path information if obstacle avoidance is activated, None otherwise.
        """

        source_pos = observation.origin
        target_waypoint = self.waypoints[self.waypoint_idx]
        target_pos = self.observation_space.geo_offset(self.observation_space.geo_origin["lat"],
                                                       self.observation_space.geo_origin["lon"],
                                                       target_waypoint.lat, target_waypoint.lon)

        merged, working_grid = self.observation_space.get_working_grid(source_pos, target_pos)
        if not working_grid:
            ValueError("No working grid found.")

        self.webapp.set_working_grid(working_grid)

        source_node = working_grid.index_pos(source_pos)
        target_node = working_grid.index_pos(target_pos)

        target_node_occupancy = working_grid.graph.nodes[target_node].get('occupancy', 0)
        if target_node_occupancy == 256:
            return 1, None
        elif target_node_occupancy & 1 == 1:
            return 2, None

        path = find_path(working_grid, source_node, target_node, avoid_occlusion=True, padding_distance=padding)
        if path["path"]:
            return 0, path
        else:
            return 1, None

    def path_next(self, last_observation: ObservationGraph, padding: int = 7) -> dict | None:
        """
        Handles obstacle avoidance logic.

        Args:
        - last_observation (ObservationGraph): The last observation data.
        - padding (int): Padding distance for obstacle avoidance.

        Returns:
        - dict: Path information to follow if obstacle avoidance was successful, None otherwise.
        """

        def calculate_angle(p1, p2):
            """Calculate the angle (in degrees) of the line segment from p1 to p2."""
            return np.degrees(np.arctan2(p2[0] - p1[0], p2[1] - p1[1]))

        def vectorise(points, angle_tolerance=5):
            """
            Bundle points into segments for straight lines, accounting for distance between points when measuring angles.

            Parameters:
            - points: List of (x, y) tuples representing rasterized points.
            - angle_tolerance: Tolerance for angle difference to consider points collinear (default: very small).

            Returns:
            - List of segments where each segment is a list of (x, y) tuples.
            """

            if len(points) < 2:
                return [points]

            segments = []
            segment_angles = []
            current_segment = [points[0]]
            current_angle = None
            last_angle = None

            for i in range(1, len(points)):
                angle_from_prvs_point = calculate_angle(points[i - 1], points[i])
                angle_from_segment = sum([calculate_angle(point, points[i]) for point in current_segment]) / len(
                    current_segment)

                if current_angle is None:
                    # Initialize the angle
                    current_angle = angle_from_prvs_point
                    last_angle = angle_from_prvs_point
                else:
                    # Check if the angle changes significantly, finalize the current segment if so
                    if angle_from_prvs_point > angle_tolerance and abs(
                            last_angle - angle_from_prvs_point) > angle_tolerance:
                        segments.append(current_segment)
                        segment_angles.append(last_angle)
                        last_angle = angle_from_segment
                        current_segment = []
                        current_angle = angle_from_prvs_point

                    # Add the current point to the segment
                    current_segment.append(points[i])

            segments.append(current_segment)
            segment_angles.append(last_angle)

            # Optimisation #1
            for i in range(1, len(segments)):
                if abs(segment_angles[i] - segment_angles[i - 1]) < angle_tolerance:
                    segments[i - 1] += segments[i]
                    segments[i] = []

            for i in range(len(segments) - 1):
                if len(segments[i]) == 0:
                    segments.pop(i)
                    segment_angles.pop(i)

            return segments

        def trace_segments(segments):
            """
            Trace the segments to get the vectorized points.

            Parameters:
            - segments: List of segments where each segment is a list of (x, y) tuples.

            Returns:
            - List of vectorized points.
            """
            vectorized_points = []
            for segment in segments:
                vectorized_points.append(segment[0])
                vectorized_points.append(segment[-1])
            return vectorized_points

        source_pos = last_observation.origin
        target_waypoint = self.waypoints[self.waypoint_idx]
        target_pos = self.observation_space.geo_offset(self.observation_space.geo_origin["lat"],
                                                       self.observation_space.geo_origin["lon"],
                                                       target_waypoint.lat, target_waypoint.lon)

        merged, working_grid = self.observation_space.get_working_grid(source_pos, target_pos)
        if not working_grid:
            return None

        self.webapp.set_working_grid(working_grid)

        source_node = working_grid.index_pos(source_pos)
        target_node = working_grid.index_pos(target_pos)

        path = find_path(working_grid, source_node, target_node, avoid_occlusion=False, padding_distance=padding)
        if not path["path"]:
            return None

        if path["occlusions"]:
            first_occluded_node = path["occlusions"][0]
            stop_pos_index = (max(0, path["path"].index(first_occluded_node) - padding))
        else:
            stop_pos_index = len(path["path"])

        path_segments = vectorise(path["path"][:stop_pos_index])
        path_points = trace_segments(path_segments)

        if target_node in path_points:
            self.waypoint_idx += 1
            full_path = True
        else:
            full_path = False

        path_geo = [
            working_grid.geo_compute(self.observation_space.geo_origin["lat"], self.observation_space.geo_origin["lat"],
                                     point[0], point[1])
            for point in path_points]
        path_geo = [{"lat": point[0], "lon": point[1]} for point in path_geo]
        path_heading = calculate_angle(path_geo[0], path_geo[-1])

        out_path = {
            "path_nodes": path_points,
            "path_pos": [working_grid.index_pos(point) for point in path_points],
            "path_geo": path_geo,
            "path_heading": path_heading,
            "full_path": full_path
        }

        return out_path

    def follow_path(self, path: dict) -> None:
        """
        Follows the path generated by the obstacle avoidance logic.

        Args:
        - path (dict): Path information.
        """
        for point in path["path_geo"]:
            self.observation_space.drone.goto(point["lat"], point["lon"])

    async def loop(self, downtime_sec: int, padding: int = 7) -> None:
        """
        Starts the obstacle avoidance operation loop.

        Args:
        - downtime_sec (int): Non-blocking sleep time between loops.
        - padding (int): Padding distance for obstacle avoidance.
        """

        self.webapp = WebApp(waypoints=self.waypoints, observation_space=self.observation_space)
        self.webapp.run()
        while not self.break_flag or self.waypoint_idx < len(self.waypoints):
            observations = self.observe()
            last_path = None
            for idx, observation in enumerate(observations):
                activation, path = self.check_activation(observation, padding)
                if activation == 0:
                    last_path = path
                    break
                elif activation == 1:
                    self.observation_space.add(observation)
                    path = self.path_next(observation, padding)
                    if path and path["full_path"]:
                        self.follow_path(path)
                        self.waypoint_idx += 1
                        break
                    elif path:
                        last_path = path
                    else:
                        self.waypoint_idx += 1  # Move to next waypoint if this one is inaccessible
                else:
                    self.waypoint_idx += 1
                    break
            if last_path:
                self.follow_path(last_path)
            else:
                self.follow_path({"path_geo": {"lat": self.waypoints[self.waypoint_idx].lat,
                                               "lon": self.waypoints[self.waypoint_idx].lon}})
                self.waypoint_idx += 1
            time.sleep(downtime_sec)
            while self.pause_execution:
                time.sleep(1)

    def start(self, downtime_sec: int) -> None:
        asyncio.run(self.loop(downtime_sec))

    def stop(self) -> None:
        """
        Stops the operation loop.
        """
        self.break_flag = True

    def pause(self) -> None:
        """
        Pauses the operation loop.
        """
        self.pause_execution = True

    def save_env(self, env_name: str) -> None:
        """
        Saves the environment configuration to a file.

        Args:
        - env_name (str): Name of the environment file.
        """
        with open(f"{env_name}.json", "w") as f:
            json.dump(self.observation_space.__dict__, f)

    def load_env(self, env_name: str) -> None:
        """
        Loads the environment configuration from a file.

        Args:
        - env_name (str): Name of the environment file.
        """
        with open(f"{env_name}.json", "r") as f:
            data = json.load(f)
            self.observation_space = ObservationSpace(**data)
