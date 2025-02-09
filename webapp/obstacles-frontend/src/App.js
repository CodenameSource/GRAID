// src/App.js
import React, { useState, useEffect } from "react";
import axios from "axios";
import DroneInfo from "./components/DroneInfo";
import NodeDetails from "./components/NodeDetails";
import MapLegend from "./components/MapLegend";
import GraphSettings from "./components/GraphSettings";
import MapComponent from "./components/MapComponent";
import GraphComponent from "./components/GraphComponent";
import "./App.css";

const App = () => {
  const [nodeInfo, setNodeInfo] = useState(null);
  const [droneInfo, setDroneInfo] = useState(null);
  const [graphSettings, setGraphSettings] = useState({
    displayFreeNodes: false,
    displayWaypoints: true,
    displayOccludedNodes: false,
    nodeSizeMultiplier: .2,
  });

  const handleNodeClick = (nodeId) => {
    axios
      .get("http://127.0.0.1:8000/change-current-node", { params: { node_id: nodeId } })
      .then(() => axios.get("http://127.0.0.1:8000/get-current-node-info"))
      .then((response) => {
        setNodeInfo(response.data);
      })
      .catch((error) => {
        console.error("Error updating node info:", error);
      });
  };

  // Fetch drone info every 5 seconds.
  // Open a WebSocket connection for drone info.
  useEffect(() => {
    const ws = new WebSocket("ws://127.0.0.1:8000/ws/drone-info");

    ws.onopen = () => {
      console.log("Connected to drone info websocket");
    };

    ws.onmessage = (event) => {
      try {
        const data = JSON.parse(event.data);
        setDroneInfo(data);
      } catch (err) {
        console.error("Error parsing drone info:", err);
      }
    };

    ws.onerror = (error) => {
      console.error("WebSocket error:", error);
    };

    return () => {
      ws.close();
    };
  }, []);

  return (
    <div className="app-container">
      <div className="left-column">
        <DroneInfo droneInfo={droneInfo} />
        <MapLegend />
        <NodeDetails nodeData={nodeInfo} />
        <GraphSettings settings={graphSettings} onChange={setGraphSettings} />
      </div>
      <div className="right-column">
        <div className="map-container">
          <MapComponent droneData={droneInfo} selectedNode={nodeInfo} />
        </div>
        <div className="graph-container">
          <GraphComponent
            width="100%"
            height="100%"
            onNodeClick={handleNodeClick}
            droneInfo={droneInfo}
            graphSettings={graphSettings}
          />
        </div>
      </div>
    </div>
  );
};

export default App;
