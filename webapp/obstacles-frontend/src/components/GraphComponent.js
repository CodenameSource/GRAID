// src/components/GraphComponent.js
import React, { useEffect, useRef } from "react";
import CytoscapeComponent from "react-cytoscapejs";
import axios from "axios";

// Utility to flip a point vertically so the whole graph renders upside-down
const flipY = (pos) => ({ x: pos.x, y: -pos.y });

const GraphComponent = ({
  width = "800px",
  height = "800px",
  onNodeClick,
  droneInfo,
  graphSettings,
}) => {
  const cyRef = useRef(null);

  // Return a color based on node type or occupancy.
  const getNodeColor = (ele) => {
    const type = ele.data("type");
    if (type === "drone") return "blue";
    if (type === "waypoint") return "#007bff";
    const occupancy = ele.data("occupancy");
    if (((occupancy & 2) === 2) && ((occupancy & 3) === 3)) return "red";
    if ((occupancy & 2) === 2) return "yellow";
    if ((occupancy & 3) === 3) return "green";
    return "gray";
  };

  // Fetch graph data and optionally waypoints, then update Cytoscape
  const fetchGraphData = async () => {
    try {
      const graphRes = await axios.get("http://127.0.0.1:8000/graph");
      // Base nodes (exclude waypoint/drone) with flipped Y
      let nodes = graphRes.data.nodes.map((node) => ({
        group: "nodes",
        data: {
          id: String(node.data.id).replace(/[()]/g, "").trim(),
          type: node.data.type,
          occupancy: node.data.occupancy,
          occlusion: node.data.occlusion,
        },
        position: flipY(node.position),
      }));

      // Optionally add waypoint nodes
      if (graphSettings.displayWaypoints) {
        const wpRes = await axios.get(
          "http://127.0.0.1:8000/get-waypoints-graph"
        );
        wpRes.data.waypoints.forEach((wp) => {
          nodes.push({
            group: "nodes",
            data: { id: String(wp.id), type: "waypoint", occupancy: wp.occupancy },
            position: flipY(wp.position),
          });
        });
      }

      // Filter free/occluded only on base nodes
      if (!graphSettings.displayFreeNodes) {
        nodes = nodes.filter((n) => n.data.occupancy !== 0 || n.data.type === "waypoint");
      }
      if (!graphSettings.displayOccludedNodes) {
        nodes = nodes.filter((n) => n.data.occupancy !== 256 || n.data.type === "waypoint");
      }

      const allowedIds = new Set(nodes.map((n) => n.data.id));
      let edges = graphRes.data.edges.map((edge) => ({
        group: "edges",
        data: {
          id: edge.data.id
            ? String(edge.data.id)
            : `${String(edge.data.source).replace(/[()]/g, "").trim()}-${String(
                edge.data.target
              )
                .replace(/[()]/g, "")
                .trim()}`,
          source: String(edge.data.source).replace(/[()]/g, "").trim(),
          target: String(edge.data.target).replace(/[()]/g, "").trim(),
        },
      }));
      edges = edges.filter(
        (e) => allowedIds.has(e.data.source) && allowedIds.has(e.data.target)
      );

      const elements = [...nodes, ...edges];

      if (cyRef.current) {
        cyRef.current.batch(() => {
          cyRef.current.nodes().not('[type="waypoint"], [type="drone"]').remove();
          cyRef.current.edges().remove();
          cyRef.current.add(elements);
        });
        cyRef.current.layout({ name: "preset" }).run();
      }
    } catch (error) {
      console.error("Error fetching graph data:", error);
    }
  };

  // Initial fetch and polling every 10 seconds
  useEffect(() => {
    fetchGraphData();
    const interval = setInterval(fetchGraphData, 10000);
    return () => clearInterval(interval);
  }, [graphSettings]);

  // Update the drone node by removing and re-adding on each position change.
  useEffect(() => {
    if (cyRef.current && droneInfo) {
      cyRef.current.remove('node[type="drone"]');
      cyRef.current.add({
        group: "nodes",
        data: { id: "drone", type: "drone" },
        position: flipY(droneInfo.position),
      });
    }
  }, [droneInfo]);

  // Compute node sizes using the multiplier from settings.
  const multiplier = graphSettings.nodeSizeMultiplier || 0.75;
  const defaultSize = 1 * multiplier + "px";
  const droneSize = 2.5 * multiplier + "px";
  const waypointSize = 2.5 * multiplier + "px";

  return (
    <CytoscapeComponent
      elements={[]}
      style={{
        width,
        height,
        backgroundColor: "#1e1e1e",
        borderRadius: "10px",
      }}
      cy={(cy) => {
        cyRef.current = cy;
        cy.on("tap", "node", (event) => {
          const nodeId = event.target.data("id");
          if (onNodeClick) onNodeClick(nodeId);
        });
      }}
      layout={{ name: "preset" }}
      zoomingEnabled
      userZoomingEnabled
      panningEnabled
      userPanningEnabled
      autoungrabify
      stylesheet={[
        {
          selector: "node",
          style: {
            "background-color": (ele) => getNodeColor(ele),
            width: defaultSize,
            height: defaultSize,
            label: "",
            "text-valign": "center",
            "text-halign": "center",
          },
        },
        {
          selector: 'node[type="drone"]',
          style: {
            "background-image": "url('/drone.png')",
            "background-fit": "cover",
            width: droneSize,
            height: droneSize,
            label: "",
          },
        },
        {
          selector: 'node[type="waypoint"]',
          style: {
            "background-color": "#007bff",
            width: waypointSize,
            height: waypointSize,
            label: "",
          },
        },
        {
          selector: "edge",
          style: {
            width: 0.1,
            "line-color": "#ccc",
          },
        },
      ]}
    />
  );
};

export default GraphComponent;
