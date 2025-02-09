import asyncio
import math

import uvicorn
from fastapi import FastAPI, HTTPException, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware

from Smav import Waypoint
from observation_grid import ObservationGrid
from observation_space import ObservationSpace


class WebApp:
    """
    Web application for visualizing the drone's environment and state.
    """

    app = FastAPI()

    # Enable CORS (allow all origins for development)
    app.add_middleware(
        CORSMiddleware,
        allow_origins=["*"],
        allow_credentials=True,
        allow_methods=["*"],
        allow_headers=["*"],
    )

    def __init__(self, waypoints: [Waypoint], current_working_grid: ObservationGrid,
                 observation_space: ObservationSpace):
        self.current_working_grid = current_working_grid
        self.waypoints = waypoints
        self.observation_space = observation_space
        self.current_node = None

    def set_working_grid(self, grid: ObservationGrid):
        if self.current_working_grid != grid and grid is not None:
            self.current_working_grid = grid

    async def _start(self, host: str = "", port: int = 8000):
        uvicorn.run(self.app, host=host, port=port)

    def run(self, host: str = "", port: int = 8000):
        asyncio.run(self._start(host, port))

    @app.get("/graph")
    def get_graph(self):
        graph = self.current_working_grid.graph
        nodes = [
            {
                "data": {"id": str(node), "occupancy": graph.nodes[node]["occupancy"]},
                "position": {"x": graph[node]["pos"][0], "y": graph[node]["pos"][1]},
            }
            for node in graph
        ]

        edges = [
            {"data": {"source": str(u), "target": str(v)}} for u, v in graph.edges
        ]

        return {"nodes": nodes, "edges": edges}

    @app.get("/waypoints")
    def get_waypoints(self):
        processed_waypoints = [{
            "id": idx,
            "position": {"lat": wp.lat, "lng": wp.lon},
            "occupancy": 0,
        } for idx, wp in enumerate(self.waypoints)]
        return {"waypoints": processed_waypoints}

    @app.get("/get-waypoints-graph")
    def get_waypoints_graph(self):
        """
        Returns the waypoints transformed to graph coordinates.
        (This simple transformation is for demonstration; adjust as needed.)
        """

        graph_waypoints = []
        for wp in self.waypoints:
            pos = self.observation_space.geo_offset(self.observation_space.geo_origin["lat"],
                                                    self.observation_space.geo_origin["lon"],
                                                    wp.lat, wp.lon)
            if self.current_working_grid.is_relative_point_in_grid(pos):
                waypoint_node = self.current_working_grid.index_pos(pos)

                if waypoint_node in self.current_working_grid.graph:
                    graph_waypoints.append({
                        "id": f"wp_{wp['id']}",
                        "position": pos,
                        "occupancy": self.current_working_grid.graph.nodes[waypoint_node]["occupancy"]
                    })

        return {"waypoints": graph_waypoints}

    @app.get("/get-working-grid-borders")
    def get_working_grid_borders(self):
        """
        Returns the borders of the working grid (from the graph) in geo coordinates.
        :return:
        """
        working_grid_borders = self.current_working_grid.extents_geo()

        left = working_grid_borders["left"]
        right = working_grid_borders["right"]
        top = working_grid_borders["top"]
        bottom = working_grid_borders["bottom"]

        return {
            "min_lat": min(left["lat"], right["lat"], top["lat"], bottom["lat"]),
            "max_lat": max(left["lat"], right["lat"], top["lat"], bottom["lat"]),
            "min_lng": min(left["lng"], right["lng"], top["lng"], bottom["lng"]),
            "max_lng": max(left["lng"], right["lng"], top["lng"], bottom["lng"]),
        }

    @app.get("/change-current-node")
    def change_current_node(self, node_id: str):
        """
        Change the global current node based on a node_id provided as a string,
        :param node_id:
        :return:
        """

        try:
            parts = node_id.strip("()").split(",")
            node_tuple = tuple(int(x.strip()) for x in parts)
        except Exception:
            raise HTTPException(status_code=400, detail="Invalid node id format")

        if node_tuple in self.current_working_grid.is_relative_point_in_grid(node_tuple):
            node_index = self.current_working_grid.index_pos(node_tuple)
            if node_index in self.current_working_grid.graph:
                self.current_node = node_index
                return {"message": f"Current node changed to {node_index}"}
        raise HTTPException(status_code=404, detail="Node not found")

    @app.get("/get-current-node-info")
    def get_current_node_info(self):
        """
        Return the data associated with the current node.
        """
        if self.current_node is None:
            raise HTTPException(status_code=404, detail="No current node selected")

        node_data = self.current_working_grid.graph.nodes[self.current_node]
        node_data["geo_position"] = self.observation_space.geo_offset(self.observation_space.geo_origin["lat"],
                                                                      self.observation_space.geo_origin["lon"],
                                                                      node_data["pos"][0], node_data["pos"][1])
        return {"node": str(self.current_node), "data": node_data}

    def _get_drone_info(self):
        def calculate_speed(vx, vy, vz):
            return math.sqrt(vx ** 2 + vy ** 2 + vz ** 2)

        def speed_to_kmh(speed):
            return speed * 3.6

        drone_location = self.observation_space.drone.get_location()
        drone_position = self.observation_space.drone_position()

        speed_m = calculate_speed(drone_location["vx"], drone_location["vy"], drone_location["vz"])

        drone_node = None
        if self.current_working_grid.is_relative_point_in_grid(drone_position):
            drone_node = self.current_working_grid.index_pos(drone_position)

            if drone_node not in self.current_working_grid.graph:
                drone_node = None

        return {
            "altitude": drone_location["relative_alt"],  # in meters
            "speed": {"speedmeters": speed_m, "speedkm": speed_to_kmh(speed_m)},
            "heading": drone_location["hdg"],  # in degrees
            "position": {"x": drone_node[0], "y": drone_node[1]},
            "geo_position": {"lat": drone_location["lat"], "lng": drone_location["lon"]}
        }

    # Legacy endpoint
    @app.get("/get-drone-info")
    def get_drone_info(self):
        return self._get_drone_info()

    @app.websocket("/ws/drone-info")
    async def drone_info_ws(self, websocket: WebSocket):
        await websocket.accept()
        try:
            while True:
                drone_data = self._get_drone_info()
                await websocket.send_json(drone_data)
                await asyncio.sleep(10)
        except WebSocketDisconnect:
            print("Client disconnected from drone info websocket")
