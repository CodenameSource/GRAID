// src/components/DroneInfo.js
import React from "react";

const DroneInfo = ({ droneInfo }) => {
  if (!droneInfo) {
    return (
      <div className="drone-info">
        <h2>Drone Info</h2>
        <p>Loading...</p>
      </div>
    );
  }

  return (
    <div className="drone-info">
      <h2>Drone Info</h2>
      <p><strong>Altitude:</strong> {droneInfo.altitude} m</p>
      <p>
        <strong>Speed:</strong> {droneInfo.speed.speedmeters} m/s, {droneInfo.speed.speedkm} km/h
      </p>
      <p><strong>Heading:</strong> {droneInfo.heading}°</p>
      <p>
        <strong>Graph Position:</strong> ({droneInfo.position.x}, {droneInfo.position.y})
      </p>
      <p>
        <strong>Geo Position:</strong> ({droneInfo.geo_position.lat}, {droneInfo.geo_position.lng})
      </p>
    </div>
  );
};

export default DroneInfo;
