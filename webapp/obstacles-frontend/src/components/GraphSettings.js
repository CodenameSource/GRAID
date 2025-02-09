// src/components/GraphSettings.js
import React, { useState } from "react";

const GraphSettings = ({ settings, onChange }) => {
  const [localSettings, setLocalSettings] = useState(settings);

  const handleCheckboxChange = (key) => (e) => {
    const newSettings = { ...localSettings, [key]: e.target.checked };
    setLocalSettings(newSettings);
    onChange(newSettings);
  };

  const handleSliderChange = (e) => {
    const value = Number(e.target.value);
    const newSettings = { ...localSettings, nodeSizeMultiplier: value };
    setLocalSettings(newSettings);
    onChange(newSettings);
  };

  return (
    <div className="graph-settings">
      <h4>Graph Settings</h4>
      <div className="setting-item">
        <label>
          <input
            type="checkbox"
            checked={localSettings.displayFreeNodes}
            onChange={handleCheckboxChange("displayFreeNodes")}
          />
          Display Free Nodes
        </label>
      </div>
      <div className="setting-item">
        <label>
          <input
            type="checkbox"
            checked={localSettings.displayWaypoints}
            onChange={handleCheckboxChange("displayWaypoints")}
          />
          Display Waypoints
        </label>
      </div>
      <div className="setting-item">
        <label>
          <input
            type="checkbox"
            checked={localSettings.displayOccludedNodes}
            onChange={handleCheckboxChange("displayOccludedNodes")}
          />
          Display Occluded Nodes
        </label>
      </div>
      <div className="setting-item">
        <label>
          Node Size Multiplier: {localSettings.nodeSizeMultiplier}
          <input
            type="range"
            min="0.5"
            max="3"
            step="0.1"
            value={localSettings.nodeSizeMultiplier}
            onChange={handleSliderChange}
          />
        </label>
      </div>
    </div>
  );
};

export default GraphSettings;
