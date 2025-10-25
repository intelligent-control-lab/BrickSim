import random
from collections import defaultdict, deque
from typing import Any, Set, Tuple, Dict, List, Optional, DefaultDict

Edge = Tuple[Any, Any]
def canon(u, v) -> Edge: return (u, v) if u <= v else (v, u)

class SkipGraph:
    def __init__(self, K: int = 8, stable: bool = True):
        assert K >= 1
        self.K = K
        self.stable = stable
        self._adj_base = defaultdict(set)
        self._aux_edges = set()
        self._adj_aux = defaultdict(set)

    @property
    def base_edges(self) -> Set[Edge]:
        edges = set()
        for u, nbrs in self._adj_base.items():
            for v in nbrs:
                if u < v:
                    edges.add((u, v))
        return edges

    @property
    def aux_edges(self) -> Set[Edge]:
        return set(self._aux_edges)

    def connect(self, a, b):
        if a == b:
            return
        if b not in self._adj_base[a]:
            self._adj_base[a].add(b)
            self._adj_base[b].add(a)
            self._recompute_aux_all()

    def disconnect(self, a, b):
        if a == b:
            return
        if b in self._adj_base[a]:
            self._adj_base[a].remove(b)
            self._adj_base[b].remove(a)
            if not self._adj_base[a]:
                self._adj_base.pop(a, None)
            if not self._adj_base[b]:
                self._adj_base.pop(b, None)
            self._recompute_aux_all()

    def _vertex_set(self):
        V = set(self._adj_base.keys())
        for u, nbrs in self._adj_base.items():
            V.update(nbrs)
        return V

    def _bfs_targets(self, s):
        needed = [1 << i for i in range(1, self.K + 1)]
        if not needed:
            return {}
        maxd = needed[-1]
        dist = {s: 0}
        q = deque([s])
        candidates = {d: [] for d in needed}
        while q:
            u = q.popleft()
            d = dist[u]
            if (0 < d <= maxd) and (d in candidates) and (u != s):
                candidates[d].append(u)
            if d == maxd:
                continue
            for w in self._adj_base.get(u, ()):
                if w not in dist:
                    dist[w] = d + 1
                    q.append(w)
        return {
            d: min(lst) if self.stable
            else (
                ins.pop() if (ins := self._adj_aux[s].intersection(lst) and ins)
                else min(lst)
            )
            for d, lst in candidates.items() if lst
        }

    def _recompute_aux_all(self):
        self._aux_edges.clear()
        self._adj_aux = defaultdict(set)
        V = self._vertex_set()
        if not V:
            return
        for v in V:
            for t in self._bfs_targets(v).values():
                if v == t:
                    continue
                e = canon(v, t)
                if e not in self._aux_edges:
                    self._aux_edges.add(e)
                    self._adj_aux[v].add(t)
                    self._adj_aux[t].add(v)

class ForestSkipGraph:
    """
    Spanning-forest skip links:
      For each vertex v and each i in {1..K}, if the 2^i-th ancestor of v exists
      in the current forest, add the undirected aux edge {v, ancestor_{2^i}(v)}.
      No aux edge may duplicate a base edge.

    Key correctness points:
      * Aux edges use reference counts so they survive 'ownership' flips.
      * Adding a base edge removes any coincident aux (and clears aux_target owner).
      * Removing a NON-tree base edge can restore a previously blocked aux at endpoints.
      * Promotion on a cut orients the crossing edge before rerooting.
    """

    def __init__(self, K: int = 8):
        assert K >= 1
        self.K = K

        # Base graph
        self.adj_base: Dict[Any, Set[Any]] = defaultdict(set)
        self.base_tree_edges: Set[Edge] = set()
        self.base_nontree_edges: Set[Edge] = set()

        # Forest
        self.parent: Dict[Any, Optional[Any]] = {}
        self.children: Dict[Any, Set[Any]] = defaultdict(set)
        self.depth: Dict[Any, int] = {}
        self.up: Dict[Any, List[Optional[Any]]] = {}

        # Aux (refcounted)
        self.aux_edges: Set[Edge] = set()
        self.adj_aux: Dict[Any, Set[Any]] = defaultdict(set)
        self._aux_ref: Dict[Edge, int] = defaultdict(int)
        self.aux_target: Dict[Any, List[Optional[Any]]] = {}

        # Components
        self.comp_id: Dict[Any, Any] = {}
        self.comp_root: Dict[Any, Any] = {}
        self.comp_size: Dict[Any, int] = {}

        self.V: Set[Any] = set()

    # ---------- public API ----------

    def connect(self, a: Any, b: Any):
        if a == b:
            return
        self._ensure_vertex(a); self._ensure_vertex(b)
        e = canon(a, b)
        if e in self.base_tree_edges or e in self.base_nontree_edges:
            return

        # If an aux edge coincides with the new base edge, drop it and clear ownership
        if e in self.aux_edges:
            for i in range(1, self.K + 1):
                if self.aux_target[a][i] == b:
                    self._dec_aux(a, b)
                    self.aux_target[a][i] = None
                    break
                if self.aux_target[b][i] == a:
                    self._dec_aux(b, a)
                    self.aux_target[b][i] = None
                    break
            # Defensive: ensure we don't leave a stale aux edge
            if e in self.aux_edges:
                # If ownership wasn’t recorded (shouldn’t happen), force-remove cleanly
                self._force_remove_aux_pair(a, b)

        # Add base edge
        self._base_add(a, b)

        ca, cb = self.comp_id[a], self.comp_id[b]
        if ca != cb:
            # Link smaller component under larger
            sa, sb = self.comp_size[ca], self.comp_size[cb]
            if sa <= sb:
                small_comp, big_comp = ca, cb
                small_ep, big_ep = a, b
            else:
                small_comp, big_comp = cb, ca
                small_ep, big_ep = b, a

            small_nodes = self._nodes_of_component(small_comp)
            self._reroot_component_at(small_ep, small_nodes)
            self.parent[small_ep] = big_ep
            self.children[big_ep].add(small_ep)
            self.base_tree_edges.add(e)

            big_root = self.comp_root[big_comp]
            merged_nodes = small_nodes | self._nodes_of_component(big_comp)
            self._set_component(merged_nodes, big_comp, big_root)

            self._rebuild_up_subtree(small_ep)
            self._sync_aux_for_vertices(small_nodes)
        else:
            # Non-tree base edge
            self.base_nontree_edges.add(e)
            # No aux change needed; creation is blocked by _inc_aux.
        self.assert_invariants()

    def disconnect(self, a: Any, b: Any):
        if a == b:
            return
        e = canon(a, b)
        if e not in self.base_tree_edges and e not in self.base_nontree_edges:
            return

        self._base_remove(a, b)

        if e in self.base_nontree_edges:
            # Simple non-tree delete; try to restore endpoint aux if it becomes allowed
            self.base_nontree_edges.remove(e)
            touched: Set[Any] = set()
            for i in range(1, self.K + 1):
                if self.up.get(a) and self.up[a][i] == b:
                    touched.add(a)
                if self.up.get(b) and self.up[b][i] == a:
                    touched.add(b)
            if touched:
                self._sync_aux_for_vertices(touched)
            return

        # Tree edge cut
        self.base_tree_edges.remove(e)

        if self.parent.get(a) == b:
            child, parent = a, b
        elif self.parent.get(b) == a:
            child, parent = b, a
        else:
            # Inconsistent forest; nothing to do
            return

        # Detach
        self.parent[child] = None
        self.children[parent].discard(child)

        child_side = self._collect_subtree(child)
        parent_root = self._find_root(parent)
        parent_side = self._collect_subtree(parent_root)

        # Promote a crossing non-tree edge if any
        crossing = self._find_crossing_nontree(child_side, parent_side)
        if crossing is not None:
            x, y = crossing
            # Ensure x ∈ child_side, y ∈ parent_side (the finder may return swapped endpoints)
            if x in parent_side and y in child_side:
                x, y = y, x
            assert x in child_side and y in parent_side

            self.base_nontree_edges.remove(canon(x, y))
            # Re-root the child side at x and attach beneath y
            self._reroot_component_at(x, child_side)
            self.parent[x] = y
            self.children[y].add(x)
            self.base_tree_edges.add(canon(x, y))

            self._rebuild_up_subtree(x)
            self._sync_aux_for_vertices(child_side)
        else:
            # True split
            self._set_component(child_side, child, child)
            self._set_component(parent_side, parent_root, parent_root)
            self._rebuild_up_subtree(child)
            self._sync_aux_for_vertices(child_side)
        self.assert_invariants()

    @property
    def base_edges(self) -> Set[Edge]:
        return set(self.base_tree_edges | self.base_nontree_edges)

    @property
    def aux_edges_set(self) -> Set[Edge]:
        return set(self.aux_edges)

    def assert_invariants(self):
        # Aux ⊆ (pairs at 2^i on the forest) and Aux ∩ Base = ∅
        for (u,v) in self.aux_edges:
            assert canon(u,v) not in self.base_tree_edges
            assert canon(u,v) not in self.base_nontree_edges
        for v in self.V:
            for i in range(1, self.K+1):
                t = self.up[v][i]
                has = self.aux_target[v][i]
                if t is not None:
                    e = canon(v,t)
                    # If base edge, aux must be absent and target cleared
                    if e in self.base_tree_edges or e in self.base_nontree_edges:
                        assert e not in self.aux_edges
                        assert has is None
                    else:
                        # otherwise, presence should match
                        assert ((has == t) == (e in self.aux_edges))

    # ---------- internals ----------

    def _force_remove_aux_pair(self, a: Any, b: Any):
        """Forcefully remove the undirected aux edge {a,b} (refcount, adjacency, set)."""
        e = canon(a, b)
        # Clear refcount & adj regardless of current count to avoid desync
        self._aux_ref.pop(e, None)
        if e in self.aux_edges:
            self.aux_edges.remove(e)
            self.adj_aux[a].discard(b)
            self.adj_aux[b].discard(a)
        # Clear any stale aux_target entries
        for v, t in ((a, b), (b, a)):
            if v in self.aux_target:
                for i in range(1, self.K + 1):
                    if self.aux_target[v][i] == t:
                        self.aux_target[v][i] = None

    def _ensure_vertex(self, v: Any):
        if v in self.V:
            return
        self.V.add(v)
        self.adj_base[v]  # touch
        self.parent[v] = None
        self.children[v] = set()
        self.depth[v] = 0
        self.up[v] = [None] * (self.K + 1)
        self.aux_target[v] = [None] * (self.K + 1)
        self.comp_id[v] = v
        self.comp_root[v] = v
        self.comp_size[v] = 1

    def _base_add(self, a: Any, b: Any):
        self.adj_base[a].add(b)
        self.adj_base[b].add(a)

    def _base_remove(self, a: Any, b: Any):
        self.adj_base[a].discard(b)
        self.adj_base[b].discard(a)
        if not self.adj_base[a]:
            self.adj_base.pop(a, None)
        if not self.adj_base[b]:
            self.adj_base.pop(b, None)

    def _collect_subtree(self, root: Any) -> Set[Any]:
        out: Set[Any] = set()
        q = deque([root])
        while q:
            u = q.popleft()
            if u in out:
                continue
            out.add(u)
            for c in self.children[u]:
                q.append(c)
        return out

    def _find_root(self, v: Any) -> Any:
        while self.parent[v] is not None:
            v = self.parent[v]
        return v

    def _set_component(self, nodes: Set[Any], comp_id: Any, comp_root: Any):
        for v in nodes:
            self.comp_id[v] = comp_id
        self.comp_root[comp_id] = comp_root
        self.comp_size[comp_id] = len(nodes)

    def _nodes_of_component(self, comp: Any) -> Set[Any]:
        root = self.comp_root[comp]
        return self._collect_subtree(root)

    def _reroot_component_at(self, new_root: Any, nodes: Set[Any]):
        undirected = defaultdict(list)
        for u in nodes:
            p = self.parent[u]
            if p is not None and p in nodes:
                undirected[u].append(p)
                undirected[p].append(u)
            for c in self.children[u]:
                if c in nodes:
                    undirected[u].append(c)
                    undirected[c].append(u)

        new_parent: Dict[Any, Optional[Any]] = {u: None for u in nodes}
        new_children: Dict[Any, Set[Any]] = {u: set() for u in nodes}

        seen = set([new_root])
        q = deque([new_root])
        while q:
            u = q.popleft()
            for w in undirected[u]:
                if w in nodes and w not in seen:
                    seen.add(w)
                    new_parent[w] = u
                    new_children[u].add(w)
                    q.append(w)

        for u in nodes:
            self.parent[u] = new_parent[u]
            self.children[u] = new_children[u]

    def _rebuild_up_subtree(self, root: Any):
        if self.parent[root] is None:
            self.depth[root] = 0
            self.up[root][0] = None
        else:
            p = self.parent[root]
            self.depth[root] = self.depth[p] + 1
            self.up[root][0] = p
        for j in range(1, self.K + 1):
            pj = self.up[root][j - 1]
            self.up[root][j] = self.up[pj][j - 1] if pj is not None else None

        q = deque([root])
        while q:
            u = q.popleft()
            for c in self.children[u]:
                self.depth[c] = self.depth[u] + 1
                self.up[c][0] = u
                for j in range(1, self.K + 1):
                    p = self.up[c][j - 1]
                    self.up[c][j] = self.up[p][j - 1] if p is not None else None
                q.append(c)

    def _ancestor(self, v: Any, dist: int) -> Optional[Any]:
        if dist <= 0:
            return v
        u = v
        bit = 0
        while dist > 0 and u is not None and bit <= self.K:
            if dist & 1:
                u = self.up[u][bit]
            dist >>= 1
            bit += 1
        return u

    # --- Replace your _sync_aux_for_vertices with this O(K) version ---
    def _sync_aux_for_vertices(self, verts: Set[Any]):
        """
        For each v in 'verts', sync aux targets to exactly the 2^i-th ancestor.
        This is O(K) per vertex because we directly read up[v][i].
        """
        for v in verts:
            old = self.aux_target[v]  # list of length K+1, index 0 unused
            # Remove changed/obsolete edges first
            for i in range(1, self.K + 1):
                t_old = old[i]
                t_new = self.up[v][i]          # <-- O(1): 2^i-th ancestor
                if t_old is not None and t_old != t_new:
                    self._dec_aux(v, t_old)
                    old[i] = None
            # Add missing edges (never duplicate a base edge)
            for i in range(1, self.K + 1):
                t_new = self.up[v][i]
                if t_new is None:
                    old[i] = None
                    continue
                if old[i] == t_new:
                    continue
                e = canon(v, t_new)
                if e in self.base_tree_edges or e in self.base_nontree_edges:
                    old[i] = None
                    continue
                self._inc_aux(v, t_new)
                if self._aux_ref.get(e, 0) > 0:
                    old[i] = t_new
                else:
                    old[i] = None

    def _inc_aux(self, u: Any, v: Any):
        if u == v:
            return
        e = canon(u, v)
        if e in self.base_tree_edges or e in self.base_nontree_edges:
            return
        self._aux_ref[e] += 1
        if self._aux_ref[e] == 1:
            self.aux_edges.add(e)
            self.adj_aux[u].add(v)
            self.adj_aux[v].add(u)

    def _dec_aux(self, u: Any, v: Any):
        if u == v:
            return
        e = canon(u, v)
        c = self._aux_ref.get(e, 0)
        if c <= 0:
            return
        c -= 1
        if c == 0:
            self._aux_ref.pop(e, None)
            self.aux_edges.discard(e)
            self.adj_aux[u].discard(v)
            self.adj_aux[v].discard(u)
        else:
            self._aux_ref[e] = c

    def _find_crossing_nontree(self, A: Set[Any], B: Set[Any]) -> Optional[Edge]:
        # Scan smaller side; may return (x,y) with x in A or B depending on swap.
        if len(A) > len(B):
            A, B = B, A
        Bset = B
        for x in A:
            for y in self.adj_base.get(x, ()):
                if y in Bset and canon(x, y) in self.base_nontree_edges:
                    return (x, y)
        return None

class RandomizedForestSkipGraph:
    """
    Spanning-forest skip links using Randomized Hierarchical Decomposition (RHD) with Fallback.
    Achieves O(log N) expected diameter and O(log N) expected maximum degree.

    Auxiliary Edge Rule (Revised with Fallback):
      For each vertex v, assign a random level L(v).
      An aux edge {v, w} exists if w is the *closest* ancestor of v such that
      L(w) > L(v) AND the edge {v, w} is NOT in the base graph (tree or non-tree).
      (If the closest higher-level ancestor is blocked, we continue searching upwards).
    """

    def __init__(self, randomization_p: float = 0.5, max_level_cap: int = 64):
        # Probability that a node advances to the next level.
        self.P = randomization_p
        self.MAX_LEVEL = max_level_cap

        # Base graph
        self.adj_base: DefaultDict[Any, Set[Any]] = defaultdict(set)
        self.base_tree_edges: Set[Edge] = set()
        self.base_nontree_edges: Set[Edge] = set()

        # Forest Structure
        self.parent: Dict[Any, Optional[Any]] = {}
        self.children: DefaultDict[Any, Set[Any]] = defaultdict(set)

        # Randomized Levels L(v)
        self.level: Dict[Any, int] = {}

        # Aux Edges (Refcounted and Tracked)
        self.aux_edges: Set[Edge] = set()
        self.adj_aux: DefaultDict[Any, Set[Any]] = defaultdict(set)
        self._aux_ref: DefaultDict[Edge, int] = defaultdict(int)
        # aux_target[v] stores the ancestor w that v connects to (if any)
        self.aux_target: Dict[Any, Optional[Any]] = {}

        # Components (for connectivity tracking and small-to-large merging)
        self.comp_id: Dict[Any, Any] = {}
        self.comp_root: Dict[Any, Any] = {}
        self.comp_size: Dict[Any, int] = {}

        self.V: Set[Any] = set()

    # ---------- public API ----------

    def connect(self, a: Any, b: Any):
        # Amortized Expected Time Complexity: O(log^2 N)
        if a == b:
            return
        self._ensure_vertex(a); self._ensure_vertex(b)
        e = canon(a, b)
        if e in self.base_tree_edges or e in self.base_nontree_edges:
            return

        # 1. Handle collision: If an aux edge exists, remove it before adding the base edge.
        if e in self.aux_edges:
            self._force_remove_aux_pair(a, b)

        # 2. Add base edge
        self._base_add(a, b)

        ca, cb = self.comp_id[a], self.comp_id[b]
        if ca != cb:
            # 3. Link components (Tree edge). Use small-to-large heuristic.
            sa, sb = self.comp_size[ca], self.comp_size[cb]
            if sa <= sb:
                small_comp, big_comp = ca, cb
                small_ep, big_ep = a, b
            else:
                small_comp, big_comp = cb, ca
                small_ep, big_ep = b, a

            # Reroot the smaller component. O(S).
            small_nodes = self._nodes_of_component(small_comp)
            self._reroot_component_at(small_ep, small_nodes)

            # Attach
            self.parent[small_ep] = big_ep
            self.children[big_ep].add(small_ep)
            self.base_tree_edges.add(e)

            # Update component metadata (Merge small into big)
            self.comp_size[big_comp] += self.comp_size[small_comp]
            # Clean up old component info
            if small_comp in self.comp_size: del self.comp_size[small_comp]
            if small_comp in self.comp_root: del self.comp_root[small_comp]
            
            for v in small_nodes:
                self.comp_id[v] = big_comp

            # 4. Sync Aux Edges. O(S * log N) expected time.
            self._sync_aux_for_vertices(small_nodes)
        else:
            # 3b. Intra-component (Non-tree edge)
            self.base_nontree_edges.add(e)

            # 4b. Sync Aux Edges. O(log N) expected time.
            # The new base edge blocks {a, b}, potentially triggering a fallback search.
            self._sync_aux_for_vertices({a, b})


    def disconnect(self, a: Any, b: Any):
        # Worst-Case Expected Time Complexity: O(N * (log N + Delta_base))
        if a == b:
            return
        e = canon(a, b)
        if e not in self.base_tree_edges and e not in self.base_nontree_edges:
            return

        # Remove the edge from the base graph structures early
        self._base_remove(a, b)

        if e in self.base_nontree_edges:
            # 1. Non-tree delete. O(log N) expected time.
            self.base_nontree_edges.remove(e)

            # Removing the base edge might unblock a preferred aux path.
            if a in self.V and b in self.V and self.comp_id.get(a) == self.comp_id.get(b):
               # Sync the endpoints.
               self._sync_aux_for_vertices({a, b})
            return

        # 2. Tree edge cut
        self.base_tree_edges.remove(e)

        if self.parent.get(a) == b:
            child, parent = a, b
        elif self.parent.get(b) == a:
            child, parent = b, a
        else:
            return

        # Detach
        self.parent[child] = None
        self.children[parent].discard(child)

        # Identify the child side of the cut. O(N_child).
        child_side = self._collect_subtree(child)

        # 3. Search for a replacement (Promotion) - Optimized version.
        # O(N_child * Delta_base). Avoids O(N) collection of parent_side.
        crossing = self._find_crossing_nontree_from_side(child_side)

        if crossing is not None:
            # 4a. Reconnection
            x, y = crossing
            # We know x is in child_side and y is not (y is in parent_side).

            self.base_nontree_edges.remove(canon(x, y))

            # Re-root the child side at the new connection point x. O(N_child).
            self._reroot_component_at(x, child_side)

            # Attach
            self.parent[x] = y
            self.children[y].add(x)
            self.base_tree_edges.add(canon(x, y))

            # 5. Sync Aux Edges. O(N_child * log N) expected time.
            self._sync_aux_for_vertices(child_side)
        else:
            # 4b. True split - Optimized metadata update.

            # The component ID of the parent side (which retains the original ID).
            parent_comp_id = self.comp_id.get(parent)

            # Child side forms a new component. O(N_child).
            # We use 'child' as the new component ID and root.
            self._set_component(child_side, child, child)

            # Parent side remains in the original component ID, but its size decreases. O(1).
            if parent_comp_id is not None and parent_comp_id in self.comp_size:
                self.comp_size[parent_comp_id] -= len(child_side)

            # 5. Sync Aux Edges. O(N_child * log N) expected time.
            self._sync_aux_for_vertices(child_side)

    @property
    def base_edges(self) -> Set[Edge]:
        return set(self.base_tree_edges | self.base_nontree_edges)

    @property
    def aux_edges_set(self) -> Set[Edge]:
        return set(self.aux_edges)

    # ---------- internals - Initialization ----------

    def _generate_level(self) -> int:
        """Generate a random level using geometric distribution, with a cap."""
        level = 0
        # FIX: Use the cap for robustness against rare, very high levels.
        while random.random() < self.P and level < self.MAX_LEVEL:
            level += 1
        return level

    def _ensure_vertex(self, v: Any):
        if v in self.V:
            return
        self.V.add(v)
        self.parent[v] = None
        # Initialize randomized level
        self.level[v] = self._generate_level()
        self.aux_target[v] = None
        # Initialize component (singleton)
        self.comp_id[v] = v
        self.comp_root[v] = v
        self.comp_size[v] = 1

    # ---------- internals - Base Graph Helpers ----------

    def _base_add(self, a: Any, b: Any):
        self.adj_base[a].add(b)
        self.adj_base[b].add(a)

    def _base_remove(self, a: Any, b: Any):
        self.adj_base[a].discard(b)
        self.adj_base[b].discard(a)

    # ---------- internals - Forest and Component Helpers ----------

    def _collect_subtree(self, root: Any) -> Set[Any]:
        """BFS to collect all nodes in the subtree rooted at 'root'."""
        out: Set[Any] = set()
        q = deque([root])
        while q:
            u = q.popleft()
            if u in out:
                continue
            out.add(u)
            for c in self.children.get(u, ()):
                q.append(c)
        return out

    def _set_component(self, nodes: Set[Any], comp_id: Any, comp_root: Any):
        """Update component metadata for a set of nodes."""
        for v in nodes:
            if v in self.V:
               self.comp_id[v] = comp_id
        self.comp_root[comp_id] = comp_root
        self.comp_size[comp_id] = len(nodes)

    def _nodes_of_component(self, comp: Any) -> Set[Any]:
        """Get nodes belonging to a component ID."""
        root = self.comp_root.get(comp)
        if root is None:
            return set()
        return self._collect_subtree(root)

    def _reroot_component_at(self, new_root: Any, nodes: Set[Any]):
        """
        Reorients the spanning tree of a component such that new_root becomes the root. O(|nodes|).
        """
        # 1. Collect undirected tree edges within the component
        undirected = defaultdict(list)
        for u in nodes:
            p = self.parent.get(u)
            if p is not None and p in nodes:
                undirected[u].append(p)
                undirected[p].append(u)

        # 2. BFS from new_root to establish new parent/child relationships
        new_parent: Dict[Any, Optional[Any]] = {u: None for u in nodes}
        new_children: Dict[Any, Set[Any]] = defaultdict(set)

        seen = {new_root}
        q = deque([new_root])
        while q:
            u = q.popleft()
            for w in undirected[u]:
                if w not in seen:
                    seen.add(w)
                    new_parent[w] = u
                    new_children[u].add(w)
                    q.append(w)

        # 3. Apply the changes
        for u in nodes:
            if u in self.V:
               self.parent[u] = new_parent[u]
               # Use .get() to handle nodes that became leaves
               self.children[u] = new_children.get(u, set())

    def _find_crossing_nontree_from_side(self, A: Set[Any]) -> Optional[Tuple[Any, Any]]:
        """
        (FIX: Optimized) Finds a non-tree base edge connecting set A to its complement.
        O(|A| * Delta_base). Returns (x, y) such that x in A and y not in A.
        """
        for x in A:
            for y in self.adj_base.get(x, ()):
                # If y is not in A, it's a crossing edge.
                if y not in A:
                    # Check if it's a non-tree edge (for replacement).
                    if canon(x, y) in self.base_nontree_edges:
                        return (x, y)
        return None

    # ---------- internals - Aux Edge Management (RHD Specific) ----------

    def _find_aux_target(self, v: Any) -> Optional[Any]:
        """
        (FIX: Implements Fallback) Find the closest ancestor w of v such that
        Level(w) > Level(v) AND {v, w} is not a base edge.
        If the ideal target is blocked, continue searching (fallback).
        
        Expected time complexity: O(log N). Worst case: O(Height).
        """
        if v not in self.level: return None
        v_level = self.level[v]
        w = self.parent.get(v)

        while w is not None:
            if self.level.get(w, -1) > v_level:
                # Found a potential target based on level. Check if blocked by a base edge.
                e = canon(v, w)
                if e in self.base_tree_edges or e in self.base_nontree_edges:
                    # Blocked (e.g., w is the parent, or a non-tree edge exists).
                    # Fallback: Continue searching upwards.
                    w = self.parent.get(w)
                    continue
                # Not blocked, this is the target.
                return w

            w = self.parent.get(w)
        return None

    def _sync_aux_for_vertices(self, verts: Set[Any]):
        """
        Recalculate and update the auxiliary edges for the given vertices.
        Expected time complexity: O(|verts| * log N).
        """
        for v in verts:
            if v not in self.V: continue

            # 1. Find the desired target using the fallback logic.
            desired_target = self._find_aux_target(v)
            current_target = self.aux_target.get(v)

            # 2. Update if the target has changed.
            if current_target != desired_target:
                # Remove the old edge reference
                if current_target is not None:
                    self._dec_aux(v, current_target)
                    self.aux_target[v] = None # Ensure clear

                # Add the new edge reference
                if desired_target is not None:
                    # The blocking check is already handled by _find_aux_target.
                    self._inc_aux(v, desired_target)
                    self.aux_target[v] = desired_target

    def _inc_aux(self, u: Any, v: Any):
        """Increment reference count for aux edge {u, v}, adding it if new."""
        if u == v:
            return
        e = canon(u, v)
        # Defensive safety check (should be handled by _find_aux_target)
        if e in self.base_tree_edges or e in self.base_nontree_edges:
            return

        self._aux_ref[e] += 1
        if self._aux_ref[e] == 1:
            # New edge created
            self.aux_edges.add(e)
            self.adj_aux[u].add(v)
            self.adj_aux[v].add(u)

    def _dec_aux(self, u: Any, v: Any):
        """Decrement reference count for aux edge {u, v}, removing it if count reaches 0."""
        if u == v:
            return
        e = canon(u, v)
        c = self._aux_ref.get(e, 0)
        if c <= 0:
            return

        c -= 1
        if c == 0:
            # Edge removed
            self._aux_ref.pop(e, None)
            self.aux_edges.discard(e)
            self.adj_aux[u].discard(v)
            self.adj_aux[v].discard(u)
        else:
            self._aux_ref[e] = c

    def _force_remove_aux_pair(self, a: Any, b: Any):
        """Forcefully remove the undirected aux edge {a,b}, clearing ownership."""
        e = canon(a, b)

        # Determine ownership and release references correctly.
        if self.aux_target.get(a) == b:
             self._dec_aux(a, b)
             self.aux_target[a] = None
        if self.aux_target.get(b) == a:
             self._dec_aux(b, a)
             self.aux_target[b] = None

        # Defensive fallback: ensure the edge is gone if refcount tracking was somehow inconsistent
        if e in self.aux_edges:
            self._aux_ref.pop(e, None)
            self.aux_edges.discard(e)
            self.adj_aux[a].discard(b)
            self.adj_aux[b].discard(a)

import math
import matplotlib as mpl
import matplotlib.pyplot as plt
import matplotlib.animation as animation

for key in [
    'keymap.fullscreen','keymap.grid','keymap.home','keymap.back','keymap.forward','keymap.pan',
    'keymap.zoom','keymap.save','keymap.quit','keymap.xscale','keymap.yscale','keymap.all_axes','keymap.help'
]:
    if key in mpl.rcParams:
        mpl.rcParams[key] = []

from typing import Literal


class GraphVisualizer:
    def __init__(self, kind: Literal['skip', 'forest', 'rforest'] = 'skip'):
        # Config/state
        self.kind = kind
        self._K = 3  # Adjustable via keyboard for 'skip' and 'forest'

        # Graph instance
        self.G = self._make_graph()
        self.pos = {}; self.next_id = 0
        self.pick_radius = 0.05
        self.sel_first = None
        self.path_mode = False; self.path_pair = None
        self.show_aux = True; self.show_help = True
        self.fig, self.ax = plt.subplots()
        # Keep data limits fixed when resizing window: use adjustable='box'.
        # This prevents Matplotlib from changing xlim/ylim to preserve aspect on resize.
        self.ax.set_aspect('equal', adjustable='box')
        self.ax.set_anchor('C')  # keep centered while preserving data limits
        self.ax.set_xlim(-1, 1); self.ax.set_ylim(-1, 1)
        self.ax.set_xticks([]); self.ax.set_yticks([])
        self.node_scatter = None; self.node_labels = {}; self.hud_text = None
        self.base_line_artists = []; self.aux_line_artists = []; self.path_line_artists = []
        self.diameter_line_artists = []
        self.anim = None; self.visited_layers = []; self.shortest_path_nodes = []
        self.diameter_path_nodes = []; self.diameter_length = 0
        self.maxdeg_artists = []; self.max_degree_vertex = None; self.max_degree_value = 0
        self.fig.canvas.mpl_connect('button_press_event', self.on_click)
        self.fig.canvas.mpl_connect('key_press_event', self.on_key)
        self._draw_all()

    # ---- graph factory / helpers ----
    def _make_graph(self):
        if self.kind == 'skip':
            return SkipGraph(K=self._K)
        if self.kind == 'forest':
            return ForestSkipGraph(K=self._K)
        if self.kind == 'rforest':
            return RandomizedForestSkipGraph()
        # Fail-fast on invalid kind
        raise ValueError(f"Unknown kind: {self.kind}")

    def _get_base_edges(self) -> Set[Edge]:
        return getattr(self.G, 'base_edges')

    def _get_aux_edges(self) -> Set[Edge]:
        if hasattr(self.G, 'aux_edges'):
            return getattr(self.G, 'aux_edges')
        if hasattr(self.G, 'aux_edges_set'):
            return getattr(self.G, 'aux_edges_set')
        return set()

    def _algo_name(self) -> str:
        return {'skip': 'SkipGraph', 'forest': 'ForestSkipGraph', 'rforest': 'RandomizedForestSkipGraph'}[self.kind]
    def add_vertex_at(self, xy):
        v = self.next_id; self.next_id += 1; self.pos[v] = xy; self._draw_all()
    def remove_vertex(self, v):
        if v not in self.pos: return
        incident = [e for e in self._get_base_edges() if v in e]
        for u, w in incident: self.G.disconnect(u, w)
        self.pos.pop(v, None)
        if self.sel_first == v: self.sel_first = None
        if self.path_pair and (self.path_pair[0] == v or self.path_pair[1] == v): self.path_pair = None; self.shortest_path_nodes = []; self.visited_layers = []
        self._draw_all()
    def _distance(self, a, b): return math.hypot(a[0]-b[0], a[1]-b[1])
    def _nearest_node(self, x, y):
        best, bestd = None, float('inf')
        for v,p in self.pos.items():
            d = self._distance((x,y),p)
            if d < bestd: best,bestd = v,d
        return best if (best is not None and bestd <= self.pick_radius) else None
    def on_click(self, event):
        if event.inaxes != self.ax: return
        x,y = event.xdata, event.ydata
        if x is None or y is None: return
        hit = self._nearest_node(x,y)
        if getattr(event, 'dblclick', False):
            if hit is None: self.add_vertex_at((x,y))
            else: self.remove_vertex(hit)
            return
        if self.path_mode:
            if hit is None: return
            if self.path_pair is None: self.path_pair = (hit, None)
            else: self.path_pair = (self.path_pair[0], hit)
            self._draw_all(); return
        if hit is None: self.sel_first = None; self._draw_all(); return
        if self.sel_first is None: self.sel_first = hit; self._draw_all()
        else:
            u,v = self.sel_first, hit; self.sel_first = None
            if u != v:
                e = canon(u,v)
                if e in self._get_base_edges(): self.G.disconnect(u,v)
                else: self.G.connect(u,v)
                self._draw_all()
    def on_key(self, event):
        if event.key == 'q': plt.close(self.fig); return
        if event.key == 'p': self.path_mode = not self.path_mode; self.sel_first = None; self._draw_all()
        elif event.key == 'a': self.animate_shortest_path()
        elif event.key == 'x': self.clear_path()
        elif event.key == 'f': self.show_aux = not self.show_aux; self._draw_all()
        elif event.key == 'h': self.show_help = not self.show_help; self._draw_all()
        elif event.key == 'k':
            if self.kind in ('skip', 'forest'):
                self._rebuild_graph_with_new_k(self._K + 1)
        elif event.key == 'j':
            if self.kind in ('skip', 'forest'):
                self._rebuild_graph_with_new_k(max(1, self._K - 1))

    def _rebuild_graph_with_new_k(self, newK: int):
        self._K = int(newK)
        old = list(self._get_base_edges())
        self.G = self._make_graph()
        for u, v in old:
            self.G.connect(u, v)
        self._draw_all()
    def _clear_artists(self):
        if self.node_scatter is not None: self.node_scatter.remove(); self.node_scatter=None
        for t in list(self.node_labels.values()): t.remove()
        self.node_labels.clear()
        for L in self.base_line_artists: L.remove()
        self.base_line_artists.clear()
        for L in self.aux_line_artists: L.remove()
        self.aux_line_artists.clear()
        for L in self.path_line_artists: L.remove()
        self.path_line_artists.clear()
        for L in self.diameter_line_artists: L.remove()
        self.diameter_line_artists.clear()
        for A in self.maxdeg_artists: A.remove()
        self.maxdeg_artists.clear()
        if self.hud_text is not None:
            try: self.hud_text.remove()
            finally: self.hud_text = None
    def _draw_all(self, draw_static_path: bool = True):
        self._clear_artists()
        for (u,v) in sorted(self._get_base_edges()):
            if u not in self.pos or v not in self.pos: continue
            x1,y1 = self.pos[u]; x2,y2 = self.pos[v]
            self.base_line_artists.append(self.ax.plot([x1,x2],[y1,y2],linestyle='-',linewidth=1)[0])
        if self.show_aux:
            for (u,v) in sorted(self._get_aux_edges()):
                if u not in self.pos or v not in self.pos: continue
                x1,y1 = self.pos[u]; x2,y2 = self.pos[v]
                self.aux_line_artists.append(self.ax.plot([x1,x2],[y1,y2],linestyle='--',linewidth=1)[0])
        # Compute and draw graph diameter (base+aux) on each redraw
        self._compute_and_store_diameter()
        if len(self.diameter_path_nodes) >= 2:
            p = self.diameter_path_nodes
            for i in range(len(p)-1):
                u,v = p[i], p[i+1]
                if u not in self.pos or v not in self.pos: continue
                x1,y1 = self.pos[u]; x2,y2 = self.pos[v]
                # Distinct style for diameter: dash-dot, thicker, red
                self.diameter_line_artists.append(self.ax.plot([x1,x2],[y1,y2],linestyle='-.',linewidth=2,color='tab:red',alpha=0.9)[0])
        # Compute max base-degree and highlight the vertex with a circle
        self._compute_and_store_max_degree_base()
        if self.max_degree_vertex is not None and self.max_degree_vertex in self.pos:
            x,y = self.pos[self.max_degree_vertex]
            r = 0.06
            circ = mpl.patches.Circle((x,y), r, fill=False, ec='tab:green', lw=2.0, zorder=6)
            self.ax.add_patch(circ)
            self.maxdeg_artists.append(circ)
        if draw_static_path and len(self.shortest_path_nodes) >= 2:
            p = self.shortest_path_nodes
            for i in range(len(p)-1):
                u,v = p[i], p[i+1]
                if u not in self.pos or v not in self.pos: continue
                x1,y1 = self.pos[u]; x2,y2 = self.pos[v]
                self.path_line_artists.append(self.ax.plot([x1,x2],[y1,y2],linestyle=':',linewidth=3)[0])
        nodes = sorted(self.pos.keys())
        xs = [self.pos[v][0] for v in nodes]; ys = [self.pos[v][1] for v in nodes]
        sizes = []
        for v in nodes:
            s = 50.0
            if self.sel_first == v: s *= 1.8
            if self.path_pair and (v == self.path_pair[0] or self.path_pair[1] == v): s *= 1.6
            sizes.append(s)
        self.node_scatter = self.ax.scatter(xs, ys, s=sizes, marker='o')
        self.node_labels = {v: self.ax.text(self.pos[v][0], self.pos[v][1], f"{v}", ha='center', va='center', fontsize=8) for v in nodes}
        maxdeg_suffix = (
            f" | maxdeg={self.max_degree_value}"
            + (f"(v={self.max_degree_vertex})" if self.max_degree_vertex is not None else "")
        )
        k_text = str(self._K) if self.kind in ('skip', 'forest') else '—'
        text = (
            f"Mode: {'path-pick' if self.path_mode else 'toggle-edge'} | algo={self._algo_name()} | "
            f"K={k_text} | V={len(self.pos)} | E_base={len(self._get_base_edges())} | E_aux={len(self._get_aux_edges())} | "
            f"diam={self.diameter_length}" + maxdeg_suffix
        )
        self.hud_text = self.ax.text(0.02,0.98,text,transform=self.ax.transAxes,fontsize=8,ha='left',va='top',bbox=dict(boxstyle='round',alpha=0.1),zorder=10)
        self.fig.canvas.draw_idle()
    def _compute_and_store_max_degree_base(self):
        # Degree computed over base edges only and restricted to currently placed nodes
        deg: Dict[Any, int] = {v: 0 for v in self.pos.keys()}
        for (u, v) in self._get_base_edges().union(self._get_aux_edges()):
            if u in deg and v in deg:
                deg[u] += 1
                deg[v] += 1
        if not deg:
            self.max_degree_vertex = None
            self.max_degree_value = 0
            return
        # Stable tie-breaker: smallest vertex id
        max_val = max(deg.values())
        # Collect candidates to guard against empty dict (already handled)
        candidates = [k for k, d in deg.items() if d == max_val]
        try:
            best_v = min(candidates)
        except TypeError:
            # Fallback if vertices are non-comparable types
            best_v = sorted(candidates, key=lambda x: str(x))[0]
        self.max_degree_vertex = best_v
        self.max_degree_value = int(max_val)
    def _compute_and_store_diameter(self):
        # Diameter = max shortest-path distance (in edges) over all vertex pairs
        adj = self._combined_adjacency()
        verts = list(adj.keys())
        if not verts:
            self.diameter_path_nodes = []
            self.diameter_length = 0
            return
        best_len = -1
        best_path = []
        for s in verts:
            dist = {s: 0}
            parent = {s: None}
            q = deque([s])
            while q:
                u = q.popleft()
                for v in adj.get(u, ()): 
                    if v not in dist:
                        dist[v] = dist[u] + 1
                        parent[v] = u
                        q.append(v)
            t = max(dist, key=lambda k: dist[k]) if dist else s
            d = dist.get(t, 0)
            if d > best_len:
                best_len = d
                path = []
                cur = t
                while cur is not None:
                    path.append(cur)
                    cur = parent[cur]
                path.reverse()
                best_path = path
        self.diameter_length = int(max(best_len, 0))
        self.diameter_path_nodes = best_path
    def _combined_adjacency(self):
        vertices = set(self.pos.keys())
        adj = {v:set() for v in vertices}
        for (u,v) in self._get_base_edges():
            if u in adj and v in adj: adj[u].add(v); adj[v].add(u)
        for (u,v) in self._get_aux_edges():
            if u in adj and v in adj: adj[u].add(v); adj[v].add(u)
        return adj
    def _bfs_layers_and_parents(self, s, t):
        adj = self._combined_adjacency()
        visited = {s}; parent = {s: None}; q = deque([s]); layers = [set([s])]; found = False
        while q and not found:
            L = len(q); nxt = set()
            for _ in range(L):
                u = q.popleft()
                for v in adj.get(u, ()):
                    if v not in visited:
                        visited.add(v); parent[v] = u; q.append(v); nxt.add(v)
                        if v == t: found = True
            if nxt: layers.append(nxt)
        path = []
        if t in parent:
            cur = t
            while cur is not None: path.append(cur); cur = parent[cur]
            path.reverse()
        return layers, path
    def animate_shortest_path(self):
        if not self.path_pair or self.path_pair[1] is None: return
        s,t = self.path_pair
        if s not in self.pos or t not in self.pos: return
        layers, path = self._bfs_layers_and_parents(s, t)
        self.visited_layers = layers; self.shortest_path_nodes = path
        frames = [('layer', i) for i in range(len(layers))] + [('pathedge', i) for i in range(max(0,len(path)-1))]
        # Clear any existing artists and redraw without static path highlight,
        # so the new animation starts from a clean slate.
        self._draw_all(draw_static_path=False)
        self.path_line_artists = []
        if len(path) >= 2:
            for i in range(len(path)-1):
                u,v = path[i], path[i+1]
                x1,y1 = self.pos[u]; x2,y2 = self.pos[v]
                self.path_line_artists.append(self.ax.plot([x1,x2],[y1,y2],linestyle=':',linewidth=3,alpha=0.0)[0])
        def init(): return []
        def update(frame):
            kind, idx = frame
            visited_up_to = set().union(*layers[: idx + 1]) if kind == 'layer' else set().union(*layers)
            nodes = sorted(self.pos.keys()); sizes = []
            for v in nodes:
                s = 50.0
                if v in visited_up_to: s *= 1.7
                if self.path_pair and (v == self.path_pair[0] or self.path_pair[1] == v): s *= 1.5
                sizes.append(s)
            if self.node_scatter is not None: self.node_scatter.set_sizes(sizes)
            if kind == 'pathedge' and idx < len(self.path_line_artists): self.path_line_artists[idx].set_alpha(1.0)
            return self.base_line_artists + self.aux_line_artists + self.path_line_artists + ([self.node_scatter] if self.node_scatter else [])
        old_anim = getattr(self, 'anim', None)
        if old_anim is not None:
            es = getattr(old_anim, 'event_source', None)
            if es is not None and hasattr(es, 'stop'):
                es.stop()
        self.anim = animation.FuncAnimation(
            self.fig,
            update,
            frames=frames,
            init_func=init,
            interval=300,
            blit=False,
            repeat=False,
        )
        self.fig.canvas.draw_idle()
    def clear_path(self):
        self.shortest_path_nodes = []; self.visited_layers = []; self.path_pair = None; self._draw_all()

def main():
    # Choose one of: 'skip', 'forest', 'rforest'
    vis = GraphVisualizer(kind='skip')
    # Starter layout
    vis.add_vertex_at((-0.6, 0.0)); vis.add_vertex_at((-0.2, 0.3)); vis.add_vertex_at((-0.2,-0.3))
    vis.add_vertex_at((0.2, 0.3)); vis.add_vertex_at((0.2,-0.3)); vis.add_vertex_at((0.6, 0.0))
    # Example chain
    vis.G.connect(0,1); vis.G.connect(1,3); vis.G.connect(3,5)
    vis._draw_all(); plt.show()

if __name__ == '__main__':
    main()
