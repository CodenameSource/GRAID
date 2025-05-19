import asyncio
import contextlib
import threading
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

    def __init__(self, waypoints: [Waypoint],
                 observation_space: ObservationSpace, current_working_grid: ObservationGrid):
        self.current_working_grid = current_working_grid
        self.waypoints = waypoints
        self.observation_space = observation_space
        self.current_node = None
        self.websocket_connections: dict[WebSocket, asyncio.Event] = {}
        self.app = None

    def set_working_grid(self, grid: ObservationGrid):
        if self.current_working_grid != grid and grid is not None:
            self.current_working_grid = grid

    def _start(self):
        async def lifespan(app: FastAPI):
            # START-UP  – kick off the broadcast task
            task = asyncio.create_task(self.broadcast_drone_info_loop())
            yield
            # SHUT-DOWN – cancel the task gracefully
            task.cancel()
            with contextlib.suppress(asyncio.CancelledError):
                await task

        self.app = FastAPI(lifespan=lifespan)

        # Enable CORS (allow all origins for development)
        self.app.add_middleware(
            CORSMiddleware,
            allow_origins=["*"],
            allow_credentials=True,
            allow_methods=["*"],
            allow_headers=["*"],
        )
        self.app.get("/graph")(self.get_graph)
        self.app.get("/waypoints")(self.get_waypoints)
        self.app.get("/get-waypoints-graph")(self.get_waypoints_graph)
        self.app.get("/get-working-grid-borders")(self.get_working_grid_borders)
        self.app.get("/change-current-node/{node_id}")(self.change_current_node)
        self.app.get("/get-current-node-info")(self.get_current_node_info)
        self.app.get("/get-drone-info")(self.get_drone_info)
        self.app.websocket("/ws/drone-info")(self.websocket_drone_info)

    def run(self, host: str = "", port: int = 8000):
        self._start()

        def _runner():
            uvicorn.run(self.app, host=host, port=port, log_level="info", lifespan="on")

        thread = threading.Thread(target=_runner, daemon=True)
        thread.start()
        print("Web app started")

    def get_graph(self):
        if self.current_working_grid is None:
            raise HTTPException(status_code=404, detail="No working grid set")

        graph = self.current_working_grid.graph
        nodes = [
            {
                "data": {"id": str(node), "occupancy": graph.nodes[node]["occupancy"]},
                "position": {"x": graph.nodes[node]["pos"][0], "y": graph.nodes[node]["pos"][1]},
            }
            for node in graph
        ]

        edges = [
            {"data": {"source": str(u), "target": str(v)}} for u, v in graph.edges
        ]

        return {"nodes": nodes, "edges": edges}

    def get_waypoints(self):
        processed_waypoints = [{
            "id": idx,
            "position": {"lat": wp.lat, "lon": wp.lon},
            "occupancy": 0,
        } for idx, wp in enumerate(self.waypoints)]

        return {"waypoints": processed_waypoints}

    def get_waypoints_graph(self):
        """
        Returns the waypoints transformed to graph coordinates.
        (This simple transformation is for demonstration; adjust as needed.)
        """
        if self.current_working_grid is None:
            raise HTTPException(status_code=404, detail="No working grid set")


        graph_waypoints = []
        for idx, wp in enumerate(self.waypoints):
            pos = self.observation_space.geo_offset(self.observation_space.geo_origin["lat"],
                                                    self.observation_space.geo_origin["lon"],
                                                    wp.lat, wp.lon)
            if self.current_working_grid.is_relative_point_in_grid(pos):
                waypoint_node = self.current_working_grid.index_by_pos(pos)

                if waypoint_node in self.current_working_grid.graph:
                    graph_waypoints.append({
                        "id": f"wp_{idx}",
                        "position": pos,
                        "occupancy": self.current_working_grid.graph.nodes[waypoint_node]["occupancy"]
                    })

        return {"waypoints": graph_waypoints}

    def get_working_grid_borders(self):
        """
        Returns the borders of the working grid (from the graph) in geo coordinates.
        :return:
        """
        if self.current_working_grid is None:
            raise HTTPException(status_code=404, detail="No working grid set")

        working_grid_borders = self.current_working_grid.extents_geo()

        left = working_grid_borders["left"]
        right = working_grid_borders["right"]
        top = working_grid_borders["top"]
        bottom = working_grid_borders["bottom"]

        return {
            "min_lat": min(left["lat"], right["lat"], top["lat"], bottom["lat"]),
            "max_lat": max(left["lat"], right["lat"], top["lat"], bottom["lat"]),
            "min_lon": min(left["lon"], right["lon"], top["lon"], bottom["lon"]),
            "max_lon": max(left["lon"], right["lon"], top["lon"], bottom["lon"]),
        }

    def change_current_node(self, node_id: str):
        """
        Change the global current node based on a node_id provided as a string,
        :param node_id:
        :return:
        """
        if self.current_working_grid is None:
            raise HTTPException(status_code=404, detail="No working grid set")


        try:
            parts = node_id.strip("()").split(",")
            node_tuple = tuple(int(x.strip()) for x in parts)
        except Exception:
            raise HTTPException(status_code=400, detail="Invalid node id format")

        if node_tuple in self.current_working_grid.is_relative_point_in_grid(node_tuple):
            node_index = self.current_working_grid.index_by_pos(node_tuple)
            if node_index in self.current_working_grid.graph:
                self.current_node = node_index
                return {"message": f"Current node changed to {node_index}"}
        raise HTTPException(status_code=404, detail="Node not found")

    def get_current_node_info(self):
        """
        Return the data associated with the current node.
        """
        if self.current_working_grid is None:
            raise HTTPException(status_code=404, detail="No working grid set")

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

        drone_location = self.observation_space.last_drone_location
        drone_position = self.observation_space.drone_position()

        speed_m = calculate_speed(drone_location["vx"], drone_location["vy"], drone_location["vz"])

        drone_node = None
        if self.current_working_grid.is_relative_point_in_grid(drone_position):
            drone_node = self.current_working_grid.index_by_pos(drone_position)

            if drone_node not in self.current_working_grid.graph:
                drone_node = None

        return {
            "altitude": drone_location["relative_alt"],  # in meters
            "speed": {"speedmeters": speed_m, "speedkm": speed_to_kmh(speed_m)},
            "heading": drone_location["hdg"],  # in degrees
            "graph_position": {"x": drone_node[0], "y": drone_node[1]},
            "position": {"x": drone_position[1], "y": drone_position[0]},
            "geo_position": {"lat": drone_location["lat"], "lon": drone_location["lon"]}
        }

    # Legacy endpoint
    def get_drone_info(self):
        return self._get_drone_info()

    async def drone_info_connect(self, ws: WebSocket):
        await ws.accept()
        print("WebSocket client connected")
        self.websocket_connections[ws] = asyncio.Event()

        await ws.receive()

    async def drone_info_disconnect(self, ws: WebSocket):
        ev = self.websocket_connections.pop(ws, None)
        if ev:
            ev.set()
        print("WebSocket client disconnected")

    async def websocket_drone_info(self, ws: WebSocket):
        await self.drone_info_connect(ws)

    async def broadcast_drone_info_loop(self):
        while True:
            if self.current_working_grid is not None and self.current_working_grid.graph is not None and len(
                    self.websocket_connections) > 0:
                data = self._get_drone_info()

                # Send to every connected client, cleaning up any that drop
                to_remove = []
                for ws, ev in list(self.websocket_connections.items()):
                    try:
                        await ws.send_json(data)
                    except WebSocketDisconnect:
                        to_remove.append(ws)
                    except Exception as e:
                        print("Error sending to client:", e)
                        to_remove.append(ws)

                for ws in to_remove:
                    await self.drone_info_disconnect(ws)

            await asyncio.sleep(5)
