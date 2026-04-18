"""Topology post-processing helpers."""

from collections import defaultdict, deque

from bricksim.topology.schema import JsonConnection, JsonPoseHint, JsonTopology


def bfs_sort_connections(topology: JsonTopology) -> JsonTopology:
    r"""Return a copy of ``topology`` with its connections sorted in a BFS order.

    The intent is to approximate an assembly sequence:

    - We treat the topology as an undirected graph whose vertices are part ids
      and whose edges are individual JsonConnection entries.
    - For each connected component, we pick a root (anchor) and run BFS:
      * If a pose hint exists whose ``part`` lies in the component, we use it
        as the root (if multiple, the smallest id is chosen).
      * Otherwise, we fall back to the smallest part id in the component.
    - While performing BFS we:
      * Emit one "tree" edge the first time we discover a new vertex.
      * Append any remaining edges in the component afterwards, in a
        deterministic order.

    This produces a stable ordering where, within each component, connections
    that introduce new parts appear earlier than connections between already
    visited parts. The ``parts`` and ``pose_hints`` arrays are preserved
    unchanged.

    Args:
        topology: A dict matching the bricksim/lego_topology@2 schema.

    Returns:
        A shallow copy of ``topology`` whose ``\"connections\"`` list has been
        reordered. If there are no connections, the original topology is
        returned unchanged.
    """
    connections: list[JsonConnection] = list(topology["connections"])
    if not connections:
        return topology

    # Build the set of involved part ids.
    nodes: set[int] = set()
    for conn in connections:
        nodes.add(conn["stud_id"])
        nodes.add(conn["hole_id"])
    if not nodes:
        return topology

    # Adjacency: node -> neighbor set.
    adjacency: defaultdict[int, set[int]] = defaultdict(set)
    # For each unordered pair of nodes, track indices of connections that join them.
    conn_indices_by_pair: defaultdict[tuple[int, int], list[int]] = defaultdict(list)

    for idx, conn in enumerate(connections):
        stud_id = conn["stud_id"]
        hole_id = conn["hole_id"]
        if stud_id == hole_id:
            # Self-loop; still track so it does not get lost, but it does not
            # participate in BFS tree edges.
            key = (stud_id, hole_id)
            conn_indices_by_pair[key].append(idx)
            continue

        adjacency[stud_id].add(hole_id)
        adjacency[hole_id].add(stud_id)
        key = (stud_id, hole_id) if stud_id <= hole_id else (hole_id, stud_id)
        conn_indices_by_pair[key].append(idx)

    # Derive connected components over the nodes that appear in connections.
    unvisited = set(nodes)
    components: list[set[int]] = []
    while unvisited:
        root = min(unvisited)  # deterministic choice
        stack = [root]
        comp = {root}
        unvisited.remove(root)
        while stack:
            node = stack.pop()
            for neighbor in adjacency[node]:
                if neighbor in unvisited:
                    unvisited.remove(neighbor)
                    comp.add(neighbor)
                    stack.append(neighbor)
        components.append(comp)

    # Collect anchor candidates from pose hints, if available.
    pose_hints: list[JsonPoseHint] = list(topology["pose_hints"])
    anchor_ids: set[int] = {hint["part"] for hint in pose_hints}

    sorted_conn_indices: list[int] = []
    seen_conn: set[int] = set()

    for comp in components:
        # Choose root for this component.
        roots_in_comp = sorted(comp & anchor_ids)
        if roots_in_comp:
            root = roots_in_comp[0]
        else:
            root = min(comp)

        # BFS to determine a tree over this component.
        visited_nodes: set[int] = {root}
        q: deque[int] = deque([root])

        # For deterministic behaviour, we iterate neighbors in sorted order and
        # always pick the lowest-index unused connection between two nodes.
        while q:
            node = q.popleft()
            for neighbor in sorted(adjacency[node]):
                if neighbor not in comp:
                    continue
                key = (
                    (node, neighbor)
                    if node <= neighbor
                    else (neighbor, node)
                )
                indices = conn_indices_by_pair.get(key, [])
                if not indices:
                    continue

                if neighbor not in visited_nodes:
                    visited_nodes.add(neighbor)
                    q.append(neighbor)
                    # Emit one "tree" edge: the first unused connection.
                    for idx in indices:
                        if idx not in seen_conn:
                            seen_conn.add(idx)
                            sorted_conn_indices.append(idx)
                            break

        # Emit remaining edges inside this component not used as tree edges.
        for a, b in sorted(conn_indices_by_pair.keys()):
            if a not in comp or b not in comp:
                continue
            for idx in conn_indices_by_pair[(a, b)]:
                if idx not in seen_conn:
                    seen_conn.add(idx)
                    sorted_conn_indices.append(idx)

    # Finally, append any connections that were not covered by components above
    # (this should not normally happen, but keeps the function robust).
    for idx in range(len(connections)):
        if idx not in seen_conn:
            sorted_conn_indices.append(idx)

    # Rebuild the connections list in the new order.
    new_connections = [connections[i] for i in sorted_conn_indices]
    new_topology = topology.copy()
    new_topology["connections"] = new_connections
    return new_topology
