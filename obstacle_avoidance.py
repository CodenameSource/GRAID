import asyncio
import os
import nest_asyncio
import json
import time
import numpy as np
import cv2
import math

from typing import List, Tuple

from Smav import Waypoint
from imagery.connection_config import ConnectionConfig
from depth_analysis import generate_projection_images, apply_kernels, bundle_kernels_to_larger_verticals, \
    map_obstacles_graph, overlay_kernels_on_image
from imagery import DroneExtended
from observation import ObservationGraph
from observation_grid import ObservationGrid
from observation_space import ObservationSpace
from pathfinding import find_path
from webapp import WebApp

nest_asyncio.apply()

PATH_IDX = 0

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

        def create_initial_grid(grid_size, node_spacing, origin, geo_origin, proximity_threshold_mergind,
                                padding_meters=2):
            initial_grid = ObservationGrid(grid_size, node_spacing, origin, geo_origin, proximity_threshold_mergind)

            padding_nodes = int(math.ceil(padding_meters / node_spacing)) // 2
            for depth in range(padding_nodes * 2):
                for horizontal in range(-padding_nodes, padding_nodes):
                    node = (grid_size // 2 + horizontal, depth)
                    if node in initial_grid.graph:
                        initial_grid.graph.nodes[node]["occupancy"] = 0

            return initial_grid

        drone = DroneExtended(connection_config)
        if connection_config.type == "airsim" and connection_config.reset_simulation == True:
            drone.get_simulator().reset()

        drone_location = drone.get_location()
        heading_origin = drone_location["hdg"]
        geo_origin = {"lat": drone_location["lat"], "lon": drone_location["lon"]}
        initial_grid = create_initial_grid(200, .2, (0, 0), geo_origin, 0.1, 3)
        self.observation_space = ObservationSpace(200, .2, 0.1, [initial_grid], geo_origin, heading_origin, drone)

    def initialise_mission_waypoints(self, waypoints: [Waypoint]) -> None:
        """
        Initialises the mission waypoints.

        Args:
        - waypoints (List[Waypoint]): The mission waypoints.
        """
        self.waypoints = waypoints
        self.waypoint_idx = 0

    def observe(self) -> [ObservationGraph]:
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
        overlaid_images = []

        drone_location = self.observation_space.drone_location()
        observation_hdg = drone_location["hdg"]
        observation_geo_origin = {"lat": drone_location["lat"], "lon": drone_location["lon"]}

        observation_origin = self.observation_space.geo_offset(self.observation_space.geo_origin["lat"],
                                                               self.observation_space.geo_origin["lon"],
                                                               drone_location["lat"], drone_location["lon"])

        observation_origin = self.observation_space.snap_to_node_spacing(observation_origin)

        for image in self.observation_space.drone.get_depth_images():
            horizontal_fov = image["config"].horisontal_fov
            vertical_fov = image["config"].vertical_fov
            heading_image = (observation_hdg + image["config"].rotation[1]) % 360

            projection_levels = generate_projection_images(image["images"][0], max_depth=self.max_observation_depth,
                                                           fov_horizontal=horizontal_fov, fov_vertical=vertical_fov)
            activations = apply_kernels(projection_levels, kernel_horizontal_size=self.observation_space.node_spacing,
                                        kernel_vertical_size=.1,
                                        fov_horizontal=horizontal_fov, fov_vertical=vertical_fov, percentile=1)

            global PATH_IDX
            if not os.path.exists(f"activations/observation_{PATH_IDX}/"):
                os.makedirs(f"activations/observation_{PATH_IDX}/")

            depth_image = image['images'][0].copy()
            for idx, level in enumerate(activations):
                depth_image = overlay_kernels_on_image(level, ((idx + 1) / len(activations) * 1))
                cv2.imwrite(f"activations/observation_{PATH_IDX}/kernels_level_{idx}.png", depth_image)
            overlaid_images.append(depth_image)  # For debugging purposes

            activations = bundle_kernels_to_larger_verticals(activations, target_vertical_size_m=1.5)

            observation_graph = ObservationGraph(horizontal_fov, self.max_observation_depth,
                                                 self.observation_space.node_spacing, .9,
                                                 activations[0].projection_level.shape[1], observation_origin,
                                                 observation_geo_origin, (0, heading_image, 0))

            observation_graph = map_obstacles_graph(observation_graph, activations)
            observation_graph.apply_occlusion()
            observations.append(observation_graph)

        return observations, overlaid_images

    def check_activation(self, observation: ObservationGraph, padding_meters: int = .8) -> (int, dict):
        """
        Determines if obstacle avoidance should be activated.

        Args:
        - observation (Any): The observation data.
        - padding (int): Padding distance for obstacle avoidance.

        Returns:
        - int: 0 for no activation required, 1 for activation required, 2 for blocked waypoint
        - dict: Path information if obstacle avoidance is activated, None otherwise.
        """

        padding_nodes = int(padding_meters // self.observation_space.node_spacing)

        source_pos = observation.origin
        target_waypoint = self.waypoints[self.waypoint_idx]
        target_pos = self.observation_space.geo_offset(self.observation_space.geo_origin["lat"],
                                                       self.observation_space.geo_origin["lon"],
                                                       target_waypoint.lat, target_waypoint.lon)

        merged, working_grid = self.observation_space.get_working_grid(source_pos, target_pos)
        if not working_grid:
            ValueError("No working grid found.")

        self.webapp.set_working_grid(working_grid)

        source_node = working_grid.index_by_pos(source_pos)
        target_node = working_grid.index_by_pos(target_pos)

        target_node_occupancy = working_grid.graph.nodes[target_node].get('occupancy', 0)
        if target_node_occupancy == 256:
            return 1, None
        elif target_node_occupancy & 1 == 1:
            return 2, None

        path = find_path(working_grid, source_node, target_node, avoid_occlusion=True, padding_distance=padding_nodes)
        if path["path"]:
            path = self.path_next(observation, padding_meters, True)
            if path['full_path']:
                return 0, path

        return 1, None

    def path_next(self, last_observation: ObservationGraph, padding_meters: int = .6,
                  avoid_occlusion=False) -> dict | None:
        """
        Handles obstacle avoidance logic.

        Args:
        - last_observation (ObservationGraph): The last observation data.
        - padding (int): Padding distance for obstacle avoidance.

        Returns:
        - dict: Path information to follow if obstacle avoidance was successful, None otherwise.
        """
        Point = Tuple[float, float]
        Segment = Tuple[Point, Point]

        def calculate_angle(p1, p2):
            """Calculate the angle (in degrees) of the line segment from p1 to p2."""
            return np.degrees(np.arctan2(p2[0] - p1[0], p2[1] - p1[1]))

        def vectorise(
                points: List[Point],
                epsilon: float = 2.0
        ) -> List[Segment]:
            """
            Simplify a rasterized polyline into straight segments.

            Parameters
            ----------
            points
                Ordered list of (x,y) coordinates.
            epsilon
                Max perpendicular deviation (in pixels) for RDP.

            Returns
            -------
            List of (start, end) pairs for each vector segment.
            """

            def _point_line_distance(p: Point, a: Point, b: Point) -> float:
                """
                Perpendicular distance from p to line ab.
                If a == b, returns euclidean(p, a).
                """
                x0, y0 = p
                x1, y1 = a
                x2, y2 = b

                dx = x2 - x1
                dy = y2 - y1
                if dx == 0 and dy == 0:
                    return math.hypot(x0 - x1, y0 - y1)

                return abs(dy * x0 - dx * y0 + x2 * y1 - y2 * x1) / math.hypot(dx, dy)

            def _rdp(points: List[Point], epsilon: float) -> List[Point]:
                """
                Recursive Ramer–Douglas–Peucker.
                https://en.wikipedia.org/wiki/Ramer%E2%80%93Douglas%E2%80%93Peucker_algorithm
                """
                if len(points) < 3:
                    return points.copy()

                # find point with max distance to chord
                a, b = points[0], points[-1]
                idx_max, dist_max = 0, 0.0
                for i in range(1, len(points) - 1):
                    d = _point_line_distance(points[i], a, b)
                    if d > dist_max:
                        idx_max, dist_max = i, d

                if dist_max > epsilon:
                    # split and recurse
                    left = _rdp(points[: idx_max + 1], epsilon)
                    right = _rdp(points[idx_max:], epsilon)
                    # stitch, dropping duplicate
                    return left[:-1] + right
                else:
                    # all intermediate points can be dropped
                    return [a, b]



            if len(points) < 2:
                return []

            simplified = _rdp(points, epsilon)
            return [(simplified[i], simplified[i + 1]) for i in range(len(simplified) - 1)]

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

        padding_nodes = int(padding_meters // self.observation_space.node_spacing)

        source_pos = last_observation.origin
        target_waypoint = self.waypoints[self.waypoint_idx]
        target_pos = self.observation_space.geo_offset(self.observation_space.geo_origin["lat"],
                                                       self.observation_space.geo_origin["lon"],
                                                       target_waypoint.lat, target_waypoint.lon)

        merged, working_grid = self.observation_space.get_working_grid(source_pos, target_pos)
        if not working_grid:
            return None

        self.webapp.set_working_grid(working_grid)

        source_node = working_grid.index_by_pos(source_pos)
        target_node = working_grid.index_by_pos(target_pos)

        global PATH_IDX
        if not os.path.exists(f"activations/observation_{PATH_IDX}/"):
            os.makedirs(f"activations/observation_{PATH_IDX}/")

        path = find_path(working_grid, source_node, target_node, avoid_occlusion=avoid_occlusion,
                         padding_distance=padding_nodes,
                         savename=f"activations/observation_{PATH_IDX}/observation_graph.png")
        if not path["path"]:
            return None

        if path["occluded_nodes"]:
            stop_node = path["occluded_nodes"][0]
            stop_idx = max(0, path["path"].index(stop_node))
            full_path = False
        else:
            stop_idx = len(path["path"]) - 1
            path["occlusion_angles"] = [0]
            self.waypoint_idx += 1
            full_path = True

        if stop_idx > 1:
            path_segments = vectorise(path["path"][:stop_idx])
            path_points = trace_segments(path_segments)

            if len(path_points) <= 1:
                path_points = [path["path"][0], path["path"][0]]

            working_grid.visualize_path(path["path"][:stop_idx],
                                        f"activations/observation_{PATH_IDX}/working_grid_path_{PATH_IDX}")
            PATH_IDX += 1

            path_geo = [
                self.observation_space.geo_compute(self.observation_space.geo_origin["lat"],
                                                   self.observation_space.geo_origin["lon"],
                                                   working_grid.get_pos(point)[0], working_grid.get_pos(point)[1])
                for point in path_points]
            path_geo = [{"lat": point[0], "lon": point[1]} for point in path_geo]
            path_heading = path["occlusion_angles"][0]

        else:
            path_points = [path["path"][0], path["path"][0]]

            working_grid.visualize_path(path_points,
                                        f"activations/observation_{PATH_IDX}/working_grid_path_{PATH_IDX}")

            PATH_IDX += 1

            path_geo = [self.observation_space.geo_compute(self.observation_space.geo_origin["lat"],
                                                           self.observation_space.geo_origin["lon"],
                                                           working_grid.get_pos(path_points[0])[0],
                                                           working_grid.get_pos(path_points[0])[1])]
            path_geo = path_geo + path_geo

            path_geo = [{"lat": point[0], "lon": point[1]} for point in path_geo]
            path_heading = path["occlusion_angles"][0]


        out_path = {
            "path_nodes": path_points,
            "path_pos": [working_grid.get_pos(point) for point in path_points],
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
        for point in path["path_geo"][1:]:
            waypoint = Waypoint(point["lat"], point["lon"], self.observation_space.altitude_origin,
                                path['path_heading'])
            self.observation_space.drone.goto(waypoint, hold_time=0, velocity=.2)

    async def loop(self, downtime_sec: int, padding: int = .6) -> None:
        """
        Starts the obstacle avoidance operation loop.

        Args:
        - downtime_sec (int): Non-blocking sleep time between loops.
        - padding (int): Padding distance for obstacle avoidance.
        """

        self.webapp = WebApp(self.waypoints, self.observation_space, None)
        self.webapp.run()
        last_path = None
        while self.waypoint_idx < len(self.waypoints) and not self.break_flag:
            print("Observing\n")
            observations, _ = self.observe()
            for idx, observation in enumerate(observations):
                print("Mapping observation\n")
                self.observation_space.add(observation)
                print("Checking for activation\n")
                activation, path = self.check_activation(observation, padding)
                if activation == 0:
                    print("Direct path found\n")
                    last_path = path
                    break
                elif activation == 1:
                    print("Pathing\n")
                    path = self.path_next(observation, padding)
                    if path and path["full_path"]:
                        self.follow_path(path)
                        self.waypoint_idx += 1
                        break
                    elif path:
                        last_path = path
                    else:
                        if last_path:
                            last_path = {
                                "path_nodes": [last_path["path_nodes"][-1], last_path["path_nodes"][-2]],
                                "path_pos": [last_path["path_pos"][-1], last_path["path_pos"][-2]],
                                "path_geo": [last_path["path_geo"][-1], last_path["path_geo"][-2]],
                                "path_heading": path["path_heading"] if path and path["path_heading"] else last_path[
                                    "path_heading"],
                                "full_path": last_path["full_path"],
                                "repeated": True
                            }
                            break
                        else:
                            "Moving to next waypoint\n"
                            self.waypoint_idx += 1  # Move to next waypoint if this one is inaccessible
                else:
                    print("Blocked waypoint\n continuing...\n")
                    self.waypoint_idx += 1
                    break
            if last_path:
                print("Following path\n")
                self.follow_path(last_path)
                if "repeated" in last_path.keys() and last_path["repeated"] == True:
                    last_path = None
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

    def drone(self) -> DroneExtended:
        """
        Returns the drone object from the observation space
        Returns: DroneExtended
        """
        return self.observation_space.drone

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

