// src/components/MapLegend.js
import React from "react";

const MapLegend = () => {
  return (
    <div className="map-legend">
      <h4>Map Legend</h4>
      <ul>
        <li>
          <span className="legend-icon waypoint-icon"></span>
          Blue points – Waypoints
        </li>
        <li>
          <span className="legend-icon current-node-icon"></span>
          Red point – Current Node
        </li>
        <li>
          <span className="legend-icon area-icon"></span>
          Purple square – Graph Area
        </li>
      </ul>
    </div>
  );
};

export default MapLegend;
