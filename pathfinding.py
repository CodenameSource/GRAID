import networkx as nx
import numpy as np
import matplotlib.pyplot as plt


def draw_graph(graph, special_nodes=[], savename="observation_graph.png"):
    """
    Draws the 2D depth graph with curved field of view.
    Nodes are colored based on occupancy:
    - Blue for empty (value 0)
    - Red for obstacle (value 1)
    - Yellow for unknown (value 2)
    """
    # Get positions for plotting
    positions = nx.get_node_attributes(graph, 'pos')

    # Get the 'value' attribute for each node to determine colors
    node_values = nx.get_node_attributes(graph, 'occupancy')
    node_colors = [
        'purple' if node in special_nodes else
        'blue' if node_values.get(node, 0) == 0 else
        'red' if node_values.get(node, 0) == 1 else
        'yellow' for node in graph.nodes
    ]

    # Plot the graph
    plt.figure(figsize=(12, 10))
    nx.draw(graph, pos=positions, node_size=5, node_color=node_colors, with_labels=False, edge_color="gray")
    plt.title("2D Depth Graph with Curved Field of View and Occupancy Colors")
    plt.xlabel("Width (m)")
    plt.ylabel("Depth (m)")
    # plt.gca().invert_yaxis()  # Invert y-axis for better visualization
    plt.savefig(savename)


def find_path(graph, start_node, target_node,
              avoid_occlusion=False,
              padding_distance=1,
              mode="avoid_all", savename="observation_graph.png"):
    """
    Finds a path in the graph, snapping start/target to nearest free
    space if they fall under padding.  Flags any path‐nodes that lie
    within padding_distance of an occlusion node, computes their angles
    from the (original) start point, and returns which start/target were used.

    Returns a dict with:
      - "path": the computed path (or None)
      - "used_start": the node actually used as start
      - "used_target": the node actually used as target
      - "occlusions": list of occlusion nodes near the path
      - "occluded_nodes": list of path‐nodes flagged
      - "occlusion_angles": angles (degrees) from the original start_node
      - "expanded_obstacles_all": padded obstacles (all)
      - "expanded_obstacles_red": padded red obstacles
    """

    def heuristic(n1, n2):
        return np.hypot(n1[0] - n2[0], n1[1] - n2[1])

    # copy graph for pathfinding
    G = graph.graph.copy()

    # identify obstacles
    obstacle_nodes = [n for n, d in G.nodes(data=True) if d.get('occupancy', 0) & 1]
    occlusion_nodes = [n for n, d in G.nodes(data=True) if d.get('occupancy', 0) == 256]

    expanded_all = set()
    expanded_red = set()
    r2 = padding_distance ** 2

    # expand obstacles by padding_distance
    for node in obstacle_nodes:
        occ = G.nodes[node].get('occupancy', 0)
        z0, x0 = node
        for dz in range(-padding_distance, padding_distance + 1):
            for dx in range(-padding_distance, padding_distance + 1):
                if dz * dz + dx * dx <= r2:
                    nbr = (z0 + dz, x0 + dx)
                    if nbr in G:
                        expanded_all.add(nbr)
                        if (occ & 6) == 6:  # red flag
                            expanded_red.add(nbr)

    if avoid_occlusion:
        expanded_all |= set(occlusion_nodes)
        expanded_red |= set(occlusion_nodes)

    # prune graph
    if mode == "avoid_all":
        G.remove_nodes_from(expanded_all)
    elif mode == "avoid_red":
        G.remove_nodes_from(expanded_red)
    else:
        raise ValueError(f"Invalid mode: {mode}")

    # --- snap start/target to nearest free node if needed ---
    if start_node in G:
        used_start = start_node
    else:
        # find nearest remaining node in G by Euclidean distance
        used_start = min(G.nodes, key=lambda n: heuristic(n, start_node))

    if target_node in G:
        used_target = target_node
    else:
        used_target = min(G.nodes, key=lambda n: heuristic(n, target_node))

    draw_graph(G, special_nodes=[used_start, used_target], savename=savename)
    # compute A* path
    try:
        path = nx.astar_path(G, used_start, used_target, heuristic=heuristic)
    except nx.NetworkXNoPath:
        path = None

    # flag occlusions: any path‐node within padding_distance of an occlusion node
    occlusion_nodes = [n for n, d in G.nodes(data=True) if d.get('occupancy', 0) == 256]

    occlusions = []
    occluded_nodes = []
    if path:
        for p in path:
            for occ in occlusion_nodes:
                if heuristic(p, occ) <= padding_distance * 2:
                    occlusions.append(occ)
                    occluded_nodes.append(p)
                    break

    # compute angles (in degrees) from the ORIGINAL start_node to each occlusion
    occlusion_angles = []
    sx, sy = start_node
    for (oz, ox) in occlusions:
        dy = oz - sx
        dx = ox - sy
        occlusion_angles.append(np.degrees(np.arctan2(dy, dx)))

    return {
        "path": path,
        "used_start": used_start,
        "used_target": used_target,
        "occlusions": occlusions,
        "occluded_nodes": occluded_nodes,
        "occlusion_angles": occlusion_angles,
        "expanded_obstacles_all": list(expanded_all),
        "expanded_obstacles_red": list(expanded_red)
    }
