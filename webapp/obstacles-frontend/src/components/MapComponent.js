// src/components/MapComponent.js
import React, { useEffect, useState } from "react";
import { MapContainer, TileLayer, Marker, Popup, Rectangle, useMap } from "react-leaflet";
import L from "leaflet";
import "leaflet/dist/leaflet.css";

const WAYPOINTS_URL = "http://localhost:8000/waypoints";
const GRID_BORDERS_URL = "http://localhost:8000/get-working-grid-borders";

// FitMapView: Adjusts the map view only once when the data is ready.
const FitMapView = ({ waypoints, droneData }) => {
  const map = useMap();
  const [initialFitDone, setInitialFitDone] = useState(false);

  useEffect(() => {
    if (!initialFitDone && map) {
      const bounds = [];
      waypoints.forEach((wp) => bounds.push([wp.position.lat, wp.position.lon]));
      if (droneData?.geo_position) {
        bounds.push([droneData.geo_position.lat, droneData.geo_position.lon]);
      }
      if (bounds.length > 0) {
        map.fitBounds(bounds, { padding: [50, 50] });
        setInitialFitDone(true);
      }
    }
  }, [map, waypoints, droneData, initialFitDone]);

  return null;
};

const MapComponent = ({ droneData, selectedNode }) => {
  const [waypoints, setWaypoints] = useState([]);
  const [gridBorders, setGridBorders] = useState(null);

  // Poll waypoints every 10 seconds
  useEffect(() => {
    const fetchWaypoints = () => {
      fetch(WAYPOINTS_URL)
        .then((res) => res.json())
        .then((data) => setWaypoints(data.waypoints))
        .catch((err) => console.error("Error fetching waypoints:", err));
    };
    fetchWaypoints();
    const wpInterval = setInterval(fetchWaypoints, 10000);
    return () => clearInterval(wpInterval);
  }, []);

  // Poll grid borders every 10 seconds
  useEffect(() => {
    const fetchGridBorders = () => {
      fetch(GRID_BORDERS_URL)
        .then((res) => res.json())
        .then((data) => {
          if (
            typeof data.min_lat === "number" &&
            typeof data.max_lat === "number" &&
            typeof data.min_lon === "number" &&
            typeof data.max_lon === "number"
          ) {
            setGridBorders(data);
          } else {
            console.error("Grid borders data is incomplete:", data);
          }
        })
        .catch((err) => console.error("Error fetching grid borders:", err));
    };
    fetchGridBorders();
    const gbInterval = setInterval(fetchGridBorders, 10000);
    return () => clearInterval(gbInterval);
  }, []);

  // Icon creators
  const createNumberedIcon = (number) =>
    L.divIcon({
      className: "custom-icon",
      html: `<div style="background: #007bff; color: #fff; width: 16px; height: 16px; border-radius: 50%; text-align: center; line-height: 16px; font-size: 10px; font-weight: bold;">${number}</div>`,
      iconSize: [16, 16],
    });

  const createDroneIcon = () =>
    L.divIcon({
      className: "custom-icon",
      html: `<img src="/drone.png" style="width:16px;height:16px;" alt="drone logo" />`,
      iconSize: [16, 16],
    });

  const createSelectedNodeIcon = () =>
    L.divIcon({
      className: "custom-icon",
      html: `<div style="background: rgba(255,0,47,0.78); color: #fff; width: 16px; height: 16px; border-radius: 50%; text-align: center; line-height: 16px; font-size: 10px; font-weight: bold;">*</div>`,
      iconSize: [16, 16],
    });

  // Determine center of map
  const center = droneData?.geo_position
    ? [droneData.geo_position.lat, droneData.geo_position.lon]
    : [37.7749, -122.4194];

  return (
    <MapContainer center={center} zoom={14} maxZoom={20} style={{ height: "500px", width: "100%" }}>
      <TileLayer
        url="https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png"
        attribution="&copy; OpenStreetMap contributors"
      />

      {/* Working grid borders as a Rectangle */}
      {gridBorders && (
        <Rectangle
          bounds={[
            [gridBorders.min_lat, gridBorders.min_lon],
            [gridBorders.max_lat, gridBorders.max_lon],
          ]}
          pathOptions={{ color: "purple", fillColor: "purple", fillOpacity: 0.2 }}
        />
      )}

      {/* Waypoints */}
      {waypoints.map((wp) => (
        <Marker
          key={wp.id}
          position={[wp.position.lat, wp.position.lon]}
          icon={createNumberedIcon(wp.id)}
        >
          <Popup>
            <strong>Waypoint {wp.id}</strong>
            <br />
            Lat: {wp.position.lat.toFixed(4)}
            <br />
            Lon: {wp.position.lon.toFixed(4)}
            <br />
            Occupancy: {wp.occupancy}
          </Popup>
        </Marker>
      ))}

      {/* Drone */}
      {droneData?.geo_position && (
        <Marker
          position={[droneData.geo_position.lat, droneData.geo_position.lon]}
          icon={createDroneIcon()}
        >
          <Popup>
            <strong>Drone</strong>
            <br />
            Altitude: {droneData.altitude} m
            <br />
            Speed: {droneData.speed.speedmeters} m/s
          </Popup>
        </Marker>
      )}

      {/* Selected Node */}
      {selectedNode?.data?.geo_position && (
        <Marker
          position={[
            selectedNode.data.geo_position.lat,
            selectedNode.data.geo_position.lon,
          ]}
          icon={createSelectedNodeIcon()}
        >
          <Popup>
            <strong>Selected Node</strong>
            <br />
            Node: {selectedNode.node}
            <br />
            Occupancy: {selectedNode.data.occupancy}
          </Popup>
        </Marker>
      )}

      <FitMapView waypoints={waypoints} droneData={droneData} />
    </MapContainer>
  );
};

export default MapComponent;
