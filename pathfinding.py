import networkx as nx
import numpy as np


def find_path(graph, start_node, target_node, avoid_occlusion=False, padding_distance=1, mode="avoid_all"):
    """
    Finds two paths in the graph:
    1. Shortest path avoiding all obstacles.
    2. Shortest path avoiding only "red" obstacles.

    Parameters:
    - graph (ObsGraph): The obstacle graph.
    - start_node: The starting node (depth, x).
    - target_node: The target node (depth, x).
    - padding_distance: Minimum distance (in nodes) to maintain from obstacles.

    Returns:
    - dict: Contains:
        - "path_all": Path avoiding all obstacles.
        - "path_red": Path avoiding only "red" obstacles.
        - "expanded_obstacles_all": List of padded obstacle nodes for all obstacles.
        - "expanded_obstacles_red": List of padded obstacle nodes for red obstacles.
    """

    def heuristic(node1, node2):
        return np.sqrt((node1[0] - node2[0]) ** 2 + (node1[1] - node2[1]) ** 2)

    # Create a copy of the graph for all-obstacles pathfinding
    pathfinding_graph = graph.graph.copy()

    # Expand obstacles by padding_distance
    obstacle_nodes = [node for node, data in graph.graph.nodes(data=True) if data.get('occupancy') & 1 == 1]
    occlusion_nodes = [node for node, data in graph.graph.nodes(data=True) if data.get('occupancy') == 256]
    expanded_obstacles_all = set()
    expanded_obstacles_red = set()

    for node in obstacle_nodes:
        depth, x = node
        node_occupancy = graph.graph.nodes[node].get("occupancy", 0)
        for d in range(-padding_distance, padding_distance + 1):
            for dx in range(-padding_distance, padding_distance + 1):
                expanded_node = (depth + d, x + dx)
                if expanded_node in graph.graph:
                    expanded_obstacles_all.add(expanded_node)

                    if node_occupancy & 6 == 6:  # Red obstacle
                        expanded_obstacles_red.add(expanded_node)

    if avoid_occlusion:
        expanded_obstacles_all = list(expanded_obstacles_all) + list(occlusion_nodes)
        expanded_obstacles_red = list(expanded_obstacles_red) + list(occlusion_nodes)

    if mode == "avoid_all":
        pathfinding_graph.remove_nodes_from(expanded_obstacles_all)
    elif mode == "avoid_red":
        pathfinding_graph.remove_nodes_from(expanded_obstacles_red)
    else:
        raise ValueError(f"Invalid mode: {mode}")

    if start_node not in pathfinding_graph:
        pathfinding_graph.add_node(start_node, pos=(0, 0), occupancy=0)
    if target_node not in pathfinding_graph:
        pathfinding_graph.add_node(target_node,
                                   pos=(target_node[1] * graph.node_spacing, target_node[0] * graph.node_spacing),
                                   occupancy=0)

    if mode == "avoid_all":
        try:
            path = nx.astar_path(pathfinding_graph, start_node, target_node, heuristic=heuristic)
        except nx.NetworkXNoPath:
            path = None
    elif mode == "avoid_red":
        try:
            path = nx.astar_path(pathfinding_graph, start_node, target_node, heuristic=heuristic)
        except nx.NetworkXNoPath:
            path = None

    occlusions = None

    if not avoid_occlusion and path is not None:
        occlusions = [node for node in path if node in occlusion_nodes]

    return {
        "path": path,
        "occlusions": occlusions,
        "expanded_obstacles_all": list(expanded_obstacles_all),
        "expanded_obstacles_red": list(expanded_obstacles_red)
    }
