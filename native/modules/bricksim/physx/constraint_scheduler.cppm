export module bricksim.physx.constraint_scheduler;

import std;
import bricksim.core.specs;
import bricksim.core.graph;
import bricksim.utils.transforms;
import bricksim.utils.unordered_pair;

namespace bricksim {

export template <class Create, class V, class Handle, class Transform>
concept EdgeCreateWithTransformFn =
    requires(Create c, const V &a, const V &b, const Transform &T_a_b) {
	    { c(a, b, T_a_b) } -> std::same_as<Handle>;
    };

export template <class Destroy, class Handle>
concept EdgeDestroyFn = requires(Destroy d, Handle &&h) {
	{ d(h) } -> std::same_as<void>;
};

export template <class Strategy, class Graph>
concept ConstraintSchedulingStrategy =
    requires(Strategy s, const typename Graph::ComponentView cc) {
	    {
		    s.compute(cc)
	    } -> std::same_as<std::generator<UnorderedPair<PartId>>>;
    };

export template <class Graph, class Strategy, class Handle, class CreateEdge,
                 class DestroyEdge>
    requires ConstraintSchedulingStrategy<Strategy, Graph> &&
             EdgeCreateWithTransformFn<CreateEdge, PartId, Handle,
                                       Transformd> &&
             EdgeDestroyFn<DestroyEdge, Handle>
class ConstraintScheduler {
  public:
	using EdgeKey = UnorderedPair<PartId>;

	ConstraintScheduler(const Graph *graph, Strategy strategy,
	                    CreateEdge create_edge, DestroyEdge destroy_edge)
	    : graph_(graph), strategy_(std::move(strategy)),
	      create_(std::move(create_edge)), destroy_(std::move(destroy_edge)) {}

	~ConstraintScheduler() {
		clear();
		graph_ = nullptr;
	}

	ConstraintScheduler(const ConstraintScheduler &) = delete;
	ConstraintScheduler &operator=(const ConstraintScheduler &) = delete;
	ConstraintScheduler(ConstraintScheduler &&) = delete;
	ConstraintScheduler &operator=(ConstraintScheduler &&) = delete;

	void clear() {
		for (auto &[_, val] : handles_) {
			auto &[h, _] = val;
			destroy_(std::move(h));
		}
		handles_.clear();
		skeleton_adj_.clear();
		dirty_vertices_.clear();
	}

	void notify_part_added(PartId id) {
		dirty_vertices_.insert(id);
	}

	void notify_part_removed(PartId id) {
		auto it_adj = skeleton_adj_.find(id);
		if (it_adj != skeleton_adj_.end()) {
			auto &[_, adj] = *it_adj;
			// Destroy all connected edges
			for (PartId neighbor_id : adj) {
				EdgeKey ek{id, neighbor_id};
				// Can't call remove_constraint_edge_ here because it would
				// try to erase adj and skeleton_adj_ while we are iterating
				// over it
				// 1. Remove from neighbor's adjacency
				auto it_neighbor_adj = skeleton_adj_.find(neighbor_id);
				if (it_neighbor_adj != skeleton_adj_.end()) {
					auto &[_, neighbor_adj] = *it_neighbor_adj;
					neighbor_adj.erase(id);
					if (neighbor_adj.empty()) {
						skeleton_adj_.erase(it_neighbor_adj);
					}
				}
				// 2. Remove the handle
				auto it_handle = handles_.find(ek);
				if (it_handle != handles_.end()) {
					auto &[_, val] = *it_handle;
					auto &[h, _] = val;
					destroy_(std::move(h));
					handles_.erase(it_handle);
				}
				// 3. Mark neighbor as dirty
				dirty_vertices_.insert(neighbor_id);
			}
			// Finally erase the adjacency entry for this vertex
			skeleton_adj_.erase(it_adj);
		}

		// Erase dirty vertex record
		dirty_vertices_.erase(id);
	}

	void notify_connected(PartId a, PartId b) {
		dirty_vertices_.insert(a);
		dirty_vertices_.insert(b);
	}

	void notify_disconnected(PartId a, PartId b) {
		dirty_vertices_.insert(a);
		dirty_vertices_.insert(b);
	}

	void commit() {
		for (auto cc : consume_dirty_ccs_()) {
			// 1. Collect all existing constraint edges, and transforms
			std::unordered_map<PartId, Transformd> transforms;
			std::unordered_set<EdgeKey> existing_edges;
			for (auto [u, T_root_u] : cc.transforms()) {
				transforms.emplace(u, T_root_u);
				auto it_adj = skeleton_adj_.find(u);
				if (it_adj != skeleton_adj_.end()) {
					auto &[_, adj] = *it_adj;
					for (PartId v : adj) {
						if (u < v) {
							existing_edges.emplace(u, v);
						}
					}
				}
			}
			// 2. Compute desired constraint edges
			for (EdgeKey desired_ek : strategy_.compute(cc)) {
				const auto &[a, b] = desired_ek;
				const Transformd &T_root_a = transforms.at(a);
				const Transformd &T_root_b = transforms.at(b);
				Transformd T_a_b = inverse(T_root_a) * T_root_b;
				auto it = handles_.find(desired_ek);
				if (it == handles_.end()) {
					// New edge, create it
					add_constraint_edge_(desired_ek, T_a_b);
				} else {
					existing_edges.erase(desired_ek);
					auto &[_, existing_T] = it->second;
					if (!SE3d{}.almost_equal(existing_T, T_a_b)) {
						// Transform changed, recreate it
						remove_constraint_edge_(desired_ek);
						add_constraint_edge_(desired_ek, T_a_b);
					}
				}
			}
			// 3. Remove obsolete constraint edges
			for (const EdgeKey &obsolete_ek : existing_edges) {
				remove_constraint_edge_(obsolete_ek);
			}
		}
	}

	const std::unordered_map<EdgeKey, std::tuple<Handle, Transformd>> &
	constraints() const {
		return handles_;
	}

  private:
	const Graph *graph_;
	Strategy strategy_;
	CreateEdge create_;
	DestroyEdge destroy_;
	std::unordered_map<EdgeKey, std::tuple<Handle, Transformd>> handles_;
	std::unordered_map<PartId, std::unordered_set<PartId>> skeleton_adj_;
	std::unordered_set<PartId> dirty_vertices_;

	void add_constraint_edge_(const EdgeKey &ek, const Transformd &T) {
		const auto &[a, b] = ek;
		handles_.emplace(ek, std::make_tuple(create_(a, b, T), T));

		auto [it_a, _] = skeleton_adj_.try_emplace(a);
		auto &[_, adj_a] = *it_a;
		adj_a.insert(b);

		auto [it_b, _] = skeleton_adj_.try_emplace(b);
		auto &[_, adj_b] = *it_b;
		adj_b.insert(a);
	}

	void remove_constraint_edge_(const EdgeKey &ek) {
		const auto &[a, b] = ek;

		auto it_a = skeleton_adj_.find(a);
		if (it_a != skeleton_adj_.end()) {
			auto &[_, adj_a] = *it_a;
			adj_a.erase(b);
			if (adj_a.empty()) {
				skeleton_adj_.erase(it_a);
			}
		}

		auto it_b = skeleton_adj_.find(b);
		if (it_b != skeleton_adj_.end()) {
			auto &[_, adj_b] = *it_b;
			adj_b.erase(a);
			if (adj_b.empty()) {
				skeleton_adj_.erase(it_b);
			}
		}

		auto it = handles_.find(ek);
		if (it != handles_.end()) {
			auto &[_, val] = *it;
			auto &[h, _] = val;
			destroy_(std::move(h));
			handles_.erase(it);
		}
	}

	std::generator<typename Graph::ComponentView> consume_dirty_ccs_() {
		if (graph_ == nullptr || dirty_vertices_.empty()) {
			co_return;
		}
		std::unordered_set<PartId> remaining_vertices;
		remaining_vertices.swap(dirty_vertices_);
		while (!remaining_vertices.empty()) {
			PartId seed = *remaining_vertices.begin();
			remaining_vertices.erase(seed);
			if (!graph_->parts().contains(seed)) {
				continue;
			}
			auto cc_view = graph_->component_view(seed);
			for (PartId pid : cc_view.vertices()) {
				remaining_vertices.erase(pid);
			}
			co_yield {cc_view};
		}
	}
};

export template <class Graph> struct TreeOnlySchedulingPolicy {
	using ComponentView = typename Graph::ComponentView;
	using EdgeKey = UnorderedPair<PartId>;
	std::generator<EdgeKey> compute(const ComponentView &cc) const {
		// Use only tree edges from the underlying DynamicGraph.
		for (auto [u, v] : cc.edges(/*tree_only=*/true)) {
			co_yield {u, v};
		}
	}
};

export template <class Graph> struct FullGraphSchedulingPolicy {
	using ComponentView = typename Graph::ComponentView;
	using EdgeKey = UnorderedPair<PartId>;
	std::generator<EdgeKey> compute(const ComponentView &cc) const {
		// All LEGO edges in this CC.
		for (auto [u, v] : cc.edges(/*tree_only=*/false)) {
			co_yield {u, v};
		}
	}
};

export template <class Graph> struct ExponentialSkipSchedulingPolicy {
	using ComponentView = typename Graph::ComponentView;
	using EdgeKey = UnorderedPair<PartId>;

	const std::size_t k;

	ExponentialSkipSchedulingPolicy(std::size_t k_ = 8) : k(k_) {}

	std::generator<EdgeKey> compute(const ComponentView &cc) const {
		// Add edges at distance 1, 2, 4, ..., 2^k
		std::vector<PartId> part_ids;
		for (PartId pid : cc.vertices()) {
			part_ids.push_back(pid);
		}
		std::sort(part_ids.begin(), part_ids.end());
		std::size_t n = part_ids.size();
		for (std::size_t i = 0; i < n; ++i) {
			for (std::size_t p = 0; p < k; ++p) {
				std::size_t skip = std::size_t{1} << p;
				std::size_t j = i + skip;
				if (j >= n) {
					break;
				}
				co_yield {part_ids[i], part_ids[j]};
			}
		}
	}
};

export template <class Graph, template <class> class StrategyA,
                 template <class> class StrategyB>
struct CombinedSchedulingPolicy {
	using ComponentView = typename Graph::ComponentView;
	using EdgeKey = UnorderedPair<PartId>;

	CombinedSchedulingPolicy(StrategyA<Graph> strategy_a = {},
	                         StrategyB<Graph> strategy_b = {})
	    : strategy_a_(std::move(strategy_a)),
	      strategy_b_(std::move(strategy_b)) {}

	std::generator<EdgeKey> compute(const ComponentView &cc) const {
		std::unordered_set<EdgeKey> yielded_edges;
		// First yield all edges from strategy A
		for (auto ek : strategy_a_.compute(cc)) {
			yielded_edges.insert(ek);
			co_yield ek;
		}
		// Then yield edges from strategy B that haven't been yielded yet
		for (auto ek : strategy_b_.compute(cc)) {
			if (yielded_edges.find(ek) == yielded_edges.end()) {
				yielded_edges.insert(ek);
				co_yield ek;
			}
		}
	}

  private:
	StrategyA<Graph> strategy_a_;
	StrategyB<Graph> strategy_b_;
};

export template <class Graph> struct RamanujanLikeSchedulingPolicy {
	using ComponentView = typename Graph::ComponentView;
	using EdgeKey = UnorderedPair<PartId>;

	const std::size_t degree; // target degree per vertex (even, recommended)
	const std::uint64_t seed; // deterministic seed per-policy

	RamanujanLikeSchedulingPolicy(std::size_t degree_ = 4,
	                              std::uint64_t seed_ = 0x9e3779b97f4a7c15ull)
	    : degree(degree_), seed(seed_) {}

	std::generator<EdgeKey> compute(const ComponentView &cc) const {
		// Collect and sort vertex ids to get a stable index ordering [0..n-1].
		std::vector<PartId> verts;
		verts.reserve(cc.size());
		for (PartId pid : cc.vertices()) {
			verts.push_back(pid);
		}
		std::sort(verts.begin(), verts.end());
		const std::size_t n = verts.size();
		if (n < 2 || degree == 0) {
			co_return;
		}

		// Adjacency in index space: i -> neighbors j (on sorted verts).
		std::vector<std::unordered_set<std::size_t>> adj_index(n);

		// Simple xorshift* RNG in index space, deterministic from (seed, i, round).
		auto hash3 = [](std::uint64_t x, std::uint64_t y, std::uint64_t z) {
			std::uint64_t h = 0x9e3779b97f4a7c15ull;
			auto mix = [](std::uint64_t v) {
				v ^= v >> 33;
				v *= 0xff51afd7ed558ccdull;
				v ^= v >> 33;
				v *= 0xc4ceb9fe1a85ec53ull;
				v ^= v >> 33;
				return v;
			};
			h ^= mix(x);
			h ^= mix(y) + 0x9e3779b97f4a7c15ull + (h << 6) + (h >> 2);
			h ^= mix(z) + 0x9e3779b97f4a7c15ull + (h << 6) + (h >> 2);
			return h;
		};

		auto next_neighbor = [&](std::size_t i,
		                         std::size_t attempt) -> std::size_t {
			// Use a hash of (seed, i, attempt) to pick a partner index in [0,n).
			std::uint64_t h = hash3(seed, i, attempt);
			return static_cast<std::size_t>(h % n);
		};

		// Greedy degree-filling: do multiple global rounds to avoid getting stuck.
		const std::size_t max_rounds = degree * 4; // safety factor
		for (std::size_t round = 0; round < max_rounds; ++round) {
			bool progress = false;
			for (std::size_t i = 0; i < n; ++i) {
				if (adj_index[i].size() >= degree) {
					continue;
				}
				// Try a few candidates for this (i,round).
				constexpr std::size_t max_attempts_per_round = 8;
				for (std::size_t a = 0; a < max_attempts_per_round; ++a) {
					std::size_t j =
					    next_neighbor(i, round * max_attempts_per_round + a);
					if (j == i) {
						continue; // no self-loop
					}
					if (adj_index[i].count(j) != 0) {
						continue; // no multi-edge
					}
					if (adj_index[j].size() >= degree) {
						continue; // keep degree bounded
					}
					// Accept edge i<->j
					adj_index[i].insert(j);
					adj_index[j].insert(i);
					progress = true;
					break;
				}
			}
			// Optional early exit if all degrees reached or no edges added.
			bool all_full = true;
			for (std::size_t i = 0; i < n; ++i) {
				if (adj_index[i].size() < degree) {
					all_full = false;
					break;
				}
			}
			if (all_full || !progress) {
				break;
			}
		}

		// Emit all edges exactly once, mapped back to PartId space.
		for (std::size_t i = 0; i < n; ++i) {
			PartId a = verts[i];
			for (std::size_t j : adj_index[i]) {
				if (j <= i) {
					continue; // ensure one direction
				}
				PartId b = verts[j];
				co_yield EdgeKey{a, b};
			}
		}
	}
};

} // namespace bricksim
