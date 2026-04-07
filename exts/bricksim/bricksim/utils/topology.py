from collections import defaultdict, deque
from typing import Any, Dict, List, Tuple


def bfs_sort_connections(topology: Dict[str, Any]) -> Dict[str, Any]:
    """Return a copy of ``topology`` with its connections sorted in a BFS order.

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
    connections: List[Dict[str, Any]] = list(topology.get("connections", []))
    if not connections:
        return topology

    # Build the set of involved part ids.
    nodes: set[int] = set()
    for c in connections:
        nodes.add(int(c["stud_id"]))
        nodes.add(int(c["hole_id"]))
    if not nodes:
        return topology

    # Adjacency: node -> neighbor set.
    adjacency: Dict[int, set[int]] = defaultdict(set)
    # For each unordered pair of nodes, track indices of connections that join them.
    conn_indices_by_pair: Dict[Tuple[int, int], List[int]] = defaultdict(list)

    for idx, c in enumerate(connections):
        a = int(c["stud_id"])
        b = int(c["hole_id"])
        if a == b:
            # Self-loop; still track so it does not get lost, but it does not
            # participate in BFS tree edges.
            key = (a, b)
            conn_indices_by_pair[key].append(idx)
            continue

        adjacency[a].add(b)
        adjacency[b].add(a)
        key = (a, b) if a <= b else (b, a)
        conn_indices_by_pair[key].append(idx)

    # Derive connected components over the nodes that appear in connections.
    unvisited = set(nodes)
    components: List[set[int]] = []
    while unvisited:
        root = min(unvisited)  # deterministic choice
        stack = [root]
        comp = {root}
        unvisited.remove(root)
        while stack:
            u = stack.pop()
            for v in adjacency.get(u, ()):
                if v in unvisited:
                    unvisited.remove(v)
                    comp.add(v)
                    stack.append(v)
        components.append(comp)

    # Collect anchor candidates from pose hints, if available.
    pose_hints: List[Dict[str, Any]] = list(topology.get("pose_hints", []))
    anchor_ids: set[int] = {int(h["part"]) for h in pose_hints if "part" in h}

    sorted_conn_indices: List[int] = []
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
            u = q.popleft()
            for v in sorted(adjacency.get(u, ())):
                if v not in comp:
                    continue
                key = (u, v) if u <= v else (v, u)
                indices = conn_indices_by_pair.get(key, [])
                if not indices:
                    continue

                if v not in visited_nodes:
                    visited_nodes.add(v)
                    q.append(v)
                    # Emit one "tree" edge: the first unused connection between u and v.
                    for idx in indices:
                        if idx not in seen_conn:
                            seen_conn.add(idx)
                            sorted_conn_indices.append(idx)
                            break

        # Emit any remaining edges inside this component that were not used as tree edges.
        for (a, b) in sorted(conn_indices_by_pair.keys()):
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
    new_topology = dict(topology)
    new_topology["connections"] = new_connections
    return new_topology
