import React, { useEffect, useState } from "react";
import { MapContainer, TileLayer, Marker, Popup, Polygon, useMap } from "react-leaflet";
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

      if (waypoints && waypoints.length > 0) {
        waypoints.forEach((wp) => {
          bounds.push([wp.position.lat, wp.position.lng]);
        });
      }

      if (droneData && droneData.geo_position) {
        bounds.push([droneData.geo_position.lat, droneData.geo_position.lng]);
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

  useEffect(() => {
    fetch(WAYPOINTS_URL)
      .then((res) => res.json())
      .then((data) => setWaypoints(data.waypoints))
      .catch((err) => console.error("Error fetching waypoints:", err));
  }, []);

  useEffect(() => {
    fetch(GRID_BORDERS_URL)
      .then((res) => res.json())
      .then((data) => {
        if (
          typeof data.min_lat === "number" &&
          typeof data.max_lat === "number" &&
          typeof data.min_lng === "number" &&
          typeof data.max_lng === "number"
        ) {
          setGridBorders(data);
        } else {
          console.error("Grid borders data is incomplete:", data);
        }
      })
      .catch((err) => console.error("Error fetching grid borders:", err));
  }, []);

  // Create a numbered icon for waypoints (smaller size).
  const createNumberedIcon = (number) => {
    return L.divIcon({
      className: "custom-icon",
      html: `<div style="
        background: #007bff;
        color: #fff;
        width: 16px;
        height: 16px;
        border-radius: 50%;
        text-align: center;
        line-height: 16px;
        font-size: 10px;
        font-weight: bold;
      ">${number}</div>`,
      iconSize: [16, 16],
    });
  };

  // Create an icon for the drone using a logo.
  const createDroneIcon = () => {
    return L.divIcon({
      className: "custom-icon",
      html: `<img src="/drone.png" style="width:16px;height:16px;" alt="drone logo" />`,
      iconSize: [16, 16],
    });
  };

  // Create an icon for the selected node using a logo.
  const createSelectedNodeIcon = () => {
    return L.divIcon({
      className: "custom-icon",
      html: `<div style="
        background: rgba(255,0,47,0.78);
        color: #fff;
        width: 16px;
        height: 16px;
        border-radius: 50%;
        text-align: center;
        line-height: 16px;
        font-size: 10px;
        font-weight: bold;
      ">*</div>`,
      iconSize: [16, 16],
    });
  };

  const center =
    droneData && droneData.geo_position
      ? [droneData.geo_position.lat, droneData.geo_position.lng]
      : [37.7749, -122.4194];

  let polygonCorners = [];
  if (
    gridBorders &&
    typeof gridBorders.min_lat === "number" &&
    typeof gridBorders.max_lat === "number" &&
    typeof gridBorders.min_lng === "number" &&
    typeof gridBorders.max_lng === "number"
  ) {
    polygonCorners = [
      [gridBorders.min_lat, gridBorders.min_lng],
      [gridBorders.min_lat, gridBorders.max_lng],
      [gridBorders.max_lat, gridBorders.max_lng],
      [gridBorders.max_lat, gridBorders.min_lng],
    ];
  }

  return (
    <MapContainer center={center} zoom={14} style={{ height: "500px", width: "100%" }}>
      <TileLayer
        url="https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png"
        attribution="&copy; OpenStreetMap contributors"
      />
      {polygonCorners.length > 0 && (
        <Polygon
          positions={polygonCorners}
          pathOptions={{ color: "purple", fillColor: "purple", fillOpacity: 0.2 }}
        />
      )}
      {waypoints.map((wp) => (
        <Marker
          key={wp.id}
          position={[wp.position.lat, wp.position.lng]}
          icon={createNumberedIcon(wp.id)}
        >
          <Popup>
            <strong>Waypoint {wp.id}</strong>
            <br />
            Lat: {wp.position.lat.toFixed(4)}
            <br />
            Lng: {wp.position.lng.toFixed(4)}
            <br />
            Occupancy: {wp.occupancy}
          </Popup>
        </Marker>
      ))}
      {droneData && droneData.geo_position && (
        <Marker
          position={[droneData.geo_position.lat, droneData.geo_position.lng]}
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
      {selectedNode &&
        selectedNode.data &&
        selectedNode.data.geo_position && (
          <Marker
            position={[
              selectedNode.data.geo_position.lat,
              selectedNode.data.geo_position.lng,
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
