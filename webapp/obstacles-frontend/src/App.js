import React, { useState } from "react";
import axios from "axios";
import { usePersistentWebSocket } from "./hooks/usePersistentWebSocket";
import DroneInfo from "./components/DroneInfo";
import NodeDetails from "./components/NodeDetails";
import MapLegend from "./components/MapLegend";
import GraphSettings from "./components/GraphSettings";
import MapComponent from "./components/MapComponent";
import GraphComponent from "./components/GraphComponent";
import "./App.css";

const WS_URL = "ws://127.0.0.1:8000/ws/drone-info";

const App = () => {
  const [nodeInfo, setNodeInfo] = useState(null);
  const [droneInfo, setDroneInfo] = useState(null);
  const [graphSettings, setGraphSettings] = useState({
    displayFreeNodes: false,
    displayWaypoints: true,
    displayOccludedNodes: false,
    nodeSizeMultiplier: 0.2,
  });

  // wire up the persistent WS
  usePersistentWebSocket(WS_URL, {
    onOpen: () => console.log("WebSocket connected"),
    onMessage: data => setDroneInfo(data),
    onError: e => console.error("WebSocket error:", e),
  });

  const handleNodeClick = nodeId => {
    axios
      .get("http://127.0.0.1:8000/change-current-node", {
        params: { node_id: nodeId },
      })
      .then(() => axios.get("http://127.0.0.1:8000/get-current-node-info"))
      .then(resp => setNodeInfo(resp.data))
      .catch(err => console.error("Error updating node info:", err));
  };

  return (
    <div className="app-container">
      <div className="left-column">
        <DroneInfo droneInfo={droneInfo} />
        <MapLegend />
        <NodeDetails nodeData={nodeInfo} />
        <GraphSettings
          settings={graphSettings}
          onChange={setGraphSettings}
        />
      </div>
      <div className="right-column">
        <div className="map-container">
          <MapComponent
            droneData={droneInfo}
            selectedNode={nodeInfo}
          />
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