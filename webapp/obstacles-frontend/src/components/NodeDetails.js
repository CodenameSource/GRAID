// src/components/NodeDetails.js
import React from "react";

const NodeDetails = ({nodeData}) => {
    return (
        <div className="node-details">
            <h3>Currently Selected Node:</h3>
            {nodeData ? (
                <>
                    <p>
                        <strong>Node:</strong> {nodeData.node}
                    </p>
                    <p>
                        <strong>Occupancy:</strong> {nodeData.data.occupancy}
                    </p>
                    <p>
                        <strong>Position:</strong>{" "}
                        {nodeData.data.pos
                            ? `(${nodeData.data.pos[0]}, ${nodeData.data.pos[1]})`
                            : "N/A"}
                    </p>
                    <p>
                        <strong>Geo Position:</strong>{" "}
                        {nodeData.data.geo_position
                            ? `(${nodeData.data.geo_position.lat}, ${nodeData.data.geo_position.lng})`
                            : "N/A"}
                    </p>
                </>
            ) : (
                <p>No node selected</p>
            )}
            {/* Legend */}
            <div className="legend"
                 style={{marginTop: "20px", padding: "10px", backgroundColor: "#f4f4f4", borderRadius: "5px"}}>
                <h4>Graph Legend</h4>
                <ul style={{listStyle: "none", padding: 0}}>
                    <li>
            <span style={{
                backgroundColor: "red",
                display: "inline-block",
                width: "15px",
                height: "15px",
                marginRight: "5px"
            }}></span>
                        Occupancy 1
                    </li>
                    <li>
            <span style={{
                backgroundColor: "yellow",
                display: "inline-block",
                width: "15px",
                height: "15px",
                marginRight: "5px"
            }}></span>
                        Occupancy 2
                    </li>
                    <li>
            <span style={{
                backgroundColor: "green",
                display: "inline-block",
                width: "15px",
                height: "15px",
                marginRight: "5px"
            }}></span>
                        Occupancy 3
                    </li>
                    <li>
            <span style={{
                backgroundColor: "gray",
                display: "inline-block",
                width: "15px",
                height: "15px",
                marginRight: "5px"
            }}></span>
                        Occluded
                    </li>
                    <li>
            <span style={{
                backgroundColor: "blue",
                display: "inline-block",
                width: "15px",
                height: "15px",
                marginRight: "5px"
            }}></span>
                        Drone (Graph)
                    </li>
                    <li>
            <span style={{
                backgroundColor: "purple",
                display: "inline-block",
                width: "15px",
                height: "15px",
                marginRight: "5px"
            }}></span>
                        Waypoint (Graph)
                    </li>
                </ul>
            </div>
        </div>
    );
};

export default NodeDetails;
