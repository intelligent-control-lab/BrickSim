export module bricksim.physx.breakage_selector;

import std;
import bricksim.core.graph;
import bricksim.physx.breakage;
import bricksim.utils.unordered_pair;
import bricksim.vendor;

namespace bricksim {

struct DSU {
	std::vector<int> parent;
	std::vector<int> rank;
	explicit DSU(int n) : parent(n), rank(n, 0) {
		std::iota(parent.begin(), parent.end(), 0);
	}
	int find(int i) {
		int root = i;
		while (root != parent[root]) {
			root = parent[root];
		}
		int curr = i;
		while (curr != root) {
			int nxt = parent[curr];
			parent[curr] = root;
			curr = nxt;
		}
		return root;
	}
	void unite(int i, int j) {
		int root_i = find(i);
		int root_j = find(j);
		if (root_i == root_j)
			return;
		if (rank[root_i] < rank[root_j]) {
			parent[root_i] = root_j;
		} else if (rank[root_i] > rank[root_j]) {
			parent[root_j] = root_i;
		} else {
			parent[root_j] = root_i;
			rank[root_i]++;
		}
	}
};

constexpr bool contractible(double u) {
	return u < 1.0;
}

struct SuperEdge {
	int u{-1};
	int v{-1};
	int cap{0};
	double max_w{-1.0};
	std::vector<ConnSegId> conns{};
};

struct ContractedGraph {
	int n{};
	std::vector<SuperEdge> edges{};
	std::vector<std::vector<int>> adj{};
	double max_w{-1.0};
};

ContractedGraph contract_graph(const auto &g, const BreakageSystem &sys,
                               const BreakageSolution &sol) {
	auto conn_ends = [&](ConnSegId csid) -> std::tuple<int, int> {
		ConnSegRef csref =
		    g.connection_segments().template key_of<ConnSegRef>(csid);
		const auto &[stud_if_ref, hole_if_ref] = csref;
		const auto &[stud_pid, stud_ifid] = stud_if_ref;
		const auto &[hole_pid, hole_ifid] = hole_if_ref;
		int u = sys.part_id_to_index().at(stud_pid);
		int v = sys.part_id_to_index().at(hole_pid);
		return {u, v};
	};
	DSU dsu(sys.num_parts());
	for (int i = 0; i < sys.num_clutches(); ++i) {
		double utilization = sol.utilization(i);
		if (!contractible(utilization)) {
			continue;
		}
		ConnSegId csid = sys.clutch_ids()[i];
		auto [u, v] = conn_ends(csid);
		dsu.unite(u, v);
	}
	std::vector<int> supernode_ids(sys.num_parts(), -1);
	int num_supernodes = 0;
	auto get_supernode_id = [&](int root) {
		int &id = supernode_ids[root];
		if (id == -1) {
			id = num_supernodes++;
		}
		return id;
	};
	using SuperEdgeKey = UnorderedPair<int>;
	std::unordered_map<SuperEdgeKey, SuperEdge> edges;
	double global_max_w = -1.0;
	for (int i = 0; i < sys.num_clutches(); ++i) {
		double utilization = sol.utilization(i);
		if (contractible(utilization)) {
			continue;
		}
		ConnSegId csid = sys.clutch_ids()[i];
		auto [u, v] = conn_ends(csid);
		int root_u = dsu.find(u);
		int root_v = dsu.find(v);
		if (root_u == root_v) {
			continue;
		}
		SuperEdgeKey key{get_supernode_id(root_u), get_supernode_id(root_v)};
		auto [it, _] =
		    edges.try_emplace(key, SuperEdge{.u = key.first, .v = key.second});
		SuperEdge &edge = it->second;
		edge.cap++;
		edge.max_w = std::max(edge.max_w, utilization);
		global_max_w = std::max(global_max_w, utilization);
		edge.conns.push_back(csid);
	}
	ContractedGraph cg;
	cg.n = num_supernodes;
	cg.max_w = global_max_w;
	cg.adj.resize(cg.n);
	cg.edges.reserve(edges.size());
	for (auto &[key, edge] : edges) {
		int edge_idx = cg.edges.size();
		cg.adj[edge.u].push_back(edge_idx);
		cg.adj[edge.v].push_back(edge_idx);
		cg.edges.push_back(std::move(edge));
	}
	return cg;
}

struct Dinic {
	const ContractedGraph &cg;
	std::vector<int> level;
	std::vector<int> it;
	std::vector<int> uv_cap;
	std::vector<int> vu_cap;

	explicit Dinic(const ContractedGraph &cg)
	    : cg(cg), level(cg.n), it(cg.n), uv_cap(cg.edges.size()),
	      vu_cap(cg.edges.size()) {
		for (std::size_t i = 0; i < cg.edges.size(); ++i) {
			uv_cap[i] = cg.edges[i].cap;
			vu_cap[i] = cg.edges[i].cap;
		}
	}

	int other(int eid, int x) const {
		const SuperEdge &e = cg.edges[eid];
		return x == e.u ? e.v : e.u;
	}

	int residual(int eid, int from) const {
		const SuperEdge &e = cg.edges[eid];
		return from == e.u ? uv_cap[eid] : vu_cap[eid];
	}

	void push(int eid, int from, int f) {
		const SuperEdge &e = cg.edges[eid];
		if (from == e.u) {
			uv_cap[eid] -= f;
			vu_cap[eid] += f;
		} else {
			vu_cap[eid] -= f;
			uv_cap[eid] += f;
		}
	}

	bool bfs(int s, int t) {
		std::fill(level.begin(), level.end(), -1);
		std::queue<int> q;
		level[s] = 0;
		q.push(s);
		while (!q.empty()) {
			int u = q.front();
			q.pop();
			for (int eid : cg.adj[u]) {
				int v = other(eid, u);
				if (residual(eid, u) <= 0 || level[v] != -1) {
					continue;
				}
				level[v] = level[u] + 1;
				if (v == t) {
					return true;
				}
				q.push(v);
			}
		}
		return level[t] != -1;
	}

	int dfs(int u, int t, int f) {
		if (u == t || f == 0) {
			return f;
		}
		for (int &i = it[u]; i < static_cast<int>(cg.adj[u].size()); ++i) {
			int eid = cg.adj[u][i];
			int v = other(eid, u);
			int r = residual(eid, u);
			if (r <= 0 || level[v] != level[u] + 1) {
				continue;
			}
			int pushed = dfs(v, t, std::min(f, r));
			if (pushed == 0) {
				continue;
			}
			push(eid, u, pushed);
			return pushed;
		}
		return 0;
	}

	int max_flow(int s, int t) {
		int flow = 0;
		while (bfs(s, t)) {
			std::fill(it.begin(), it.end(), 0);
			while (true) {
				int pushed = dfs(s, t, std::numeric_limits<int>::max());
				if (pushed == 0) {
					break;
				}
				flow += pushed;
			}
		}
		return flow;
	}

	std::vector<char> reachable_from(int s) const {
		std::vector<char> seen(cg.n, false);
		std::queue<int> q;
		seen[s] = true;
		q.push(s);
		while (!q.empty()) {
			int u = q.front();
			q.pop();
			for (int eid : cg.adj[u]) {
				int v = other(eid, u);
				if (seen[v] || residual(eid, u) <= 0) {
					continue;
				}
				seen[v] = true;
				q.push(v);
			}
		}
		return seen;
	}
};

struct CutResult {
	int value;
	std::vector<char> side;
};

CutResult min_cut(const ContractedGraph &cg, int s, int t) {
	Dinic dinic(cg);
	int value = dinic.max_flow(s, t);
	return {.value = value, .side = dinic.reachable_from(s)};
}

export std::vector<ConnSegId>
select_conns_to_break(const auto &g, const BreakageSystem &sys,
                      const BreakageSolution &sol) {
	ContractedGraph cg = contract_graph(g, sys, sol);
	if (cg.n <= 1 || cg.max_w < 1.0) {
		return {};
	}
	int best_cut = std::numeric_limits<int>::max();
	std::vector<char> best_side;
	for (const SuperEdge &e : cg.edges) {
		if (e.max_w != cg.max_w) {
			continue;
		}
		CutResult cut = min_cut(cg, e.u, e.v);
		if (cut.value < best_cut) {
			best_cut = cut.value;
			best_side = std::move(cut.side);
		}
	}
	std::vector<ConnSegId> result;
	result.reserve(best_cut);
	for (const SuperEdge &e : cg.edges) {
		if (best_side[e.u] == best_side[e.v]) {
			continue;
		}
		result.insert(result.end(), e.conns.begin(), e.conns.end());
	}
	return result;
}

} // namespace bricksim
