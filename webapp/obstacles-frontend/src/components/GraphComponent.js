// src/components/GraphComponent.js
import React, {useEffect, useRef} from "react";
import CytoscapeComponent from "react-cytoscapejs";
import axios from "axios";

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

    // Fetch base graph data (excluding waypoint nodes).
    const fetchGraphData = async () => {
        try {
            const response = await axios.get("http://127.0.0.1:8000/graph");
            let nodes = response.data.nodes.map((node) => ({
                group: "nodes",
                data: {
                    id: String(node.data.id).replace(/[()]/g, "").trim(),
                    occupancy: node.data.occupancy,
                    occlusion: node.data.occlusion, // may be undefined
                },
                position: node.position,
            }));
            // Filter free nodes if required.
            if (!graphSettings.displayFreeNodes) {
                nodes = nodes.filter((node) => node.data.occupancy !== 0);
            }
            // Filter occluded nodes if required.
            if (!graphSettings.displayOccludedNodes) {
                nodes = nodes.filter((node) => node.data.occupancy !== 256);
            }
            const allowedIds = new Set(nodes.map((n) => n.data.id));
            let edges = response.data.edges.map((edge) => ({
                group: "edges",
                data: {
                    id: edge.data.id
                        ? String(edge.data.id)
                        : `${String(edge.data.source)
                            .replace(/[()]/g, "").trim()}-${String(edge.data.target)
                            .replace(/[()]/g, "").trim()}`,
                    source: String(edge.data.source).replace(/[()]/g, "").trim(),
                    target: String(edge.data.target).replace(/[()]/g, "").trim(),
                },
            }));
            edges = edges.filter(
                (edge) =>
                    allowedIds.has(edge.data.source) && allowedIds.has(edge.data.target)
            );
            const elements = [...nodes, ...edges];
            if (cyRef.current) {
                // Remove any existing base graph nodes (except drone and waypoint nodes).
                cyRef.current.remove('node:not([data.type="drone"]):not([data.type="waypoint"])');
                cyRef.current.remove("edge");
                cyRef.current.batch(() => {
                    cyRef.current.add(elements);
                });
                const layout = cyRef.current.layout({name: "preset"});
                layout.run();
            }
        } catch (error) {
            console.error("Error fetching graph data:", error);
        }
    };

    useEffect(() => {
        fetchGraphData();
    }, [graphSettings]);

    // Fetch waypoint nodes from /get-waypoints-graph if enabled.
    useEffect(() => {
        const fetchWaypointsForGraph = async () => {
            try {
                const response = await axios.get("http://127.0.0.1:8000/get-waypoints-graph");
                const waypointsData = response.data.waypoints;
                waypointsData.forEach((wp) => {
                    if (cyRef.current) {
                        const wpElem = cyRef.current.getElementById(wp.id);
                        if (wpElem.nonempty()) {
                            wpElem.position(wp.position);
                        } else {
                            cyRef.current.add({
                                group: "nodes",
                                data: {id: wp.id, type: "waypoint", occupancy: wp.occupancy},
                                position: wp.position,
                            });
                        }
                    }
                });
                if (cyRef.current) {
                    const layout = cyRef.current.layout({name: "preset"});
                    layout.run();
                }
            } catch (error) {
                console.error("Error fetching waypoints for graph:", error);
            }
        };

        if (graphSettings.displayWaypoints) {
            fetchWaypointsForGraph();
        } else {
            if (cyRef.current) {
                cyRef.current.remove('node[data.type="waypoint"]');
            }
        }
    }, [graphSettings.displayWaypoints]);

    // Update or add the drone node when droneInfo changes.
    useEffect(() => {
        if (cyRef.current && droneInfo) {
            const droneElem = cyRef.current.getElementById("drone");
            if (droneElem.nonempty()) {
                droneElem.position({
                    x: droneInfo.position.x,
                    y: droneInfo.position.y,
                });
            } else {
                cyRef.current.add({
                    group: "nodes",
                    data: {id: "drone", type: "drone"},
                    position: {x: droneInfo.position.x, y: droneInfo.position.y},
                });
            }
        }
    }, [droneInfo]);

    // Compute node sizes using the multiplier from settings.
    const multiplier = graphSettings.nodeSizeMultiplier || .75;
    const defaultSize = 12 * multiplier + "px";
    const droneSize = 40 * multiplier + "px";
    const waypointSize = 24 * multiplier + "px";

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
            layout={{name: "preset"}}
            zoomingEnabled={true}
            userZoomingEnabled={false}
            panningEnabled={true}
            userPanningEnabled={false}
            autoungrabify={true}
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
                    // Drone node: use a custom icon.
                    selector: 'node[data.type="drone"]',
                    style: {
                        "background-image": "url('/drone.png')",
                        "background-fit": "cover",
                        width: droneSize,
                        height: droneSize,
                        label: "",
                    },
                },
                {
                    // Waypoint nodes.
                    selector: 'node[data.type="waypoint"]',
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
                        width: 1,
                        "line-color": "#ccc",
                    },
                },
            ]}
        />
    );
};

export default GraphComponent;
