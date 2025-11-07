export module lego_assemble.utils.skip_graph;

import std;
import lego_assemble.utils.pair;

namespace lego_assemble {

// ---------- Concepts ----------
export template <class Create, class V, class Handle>
concept EdgeCreateFn = requires(Create c, const V &a, const V &b) {
	{ c(a, b) } -> std::same_as<Handle>;
};

export template <class Destroy, class Handle>
concept EdgeDestroyFn = requires(Destroy d, Handle h) {
	{ d(h) } -> std::same_as<void>;
};

// A no-op scheduler that only tracks base adjacency.
export template <class V, class Handle, class CreateEdge, class DestroyEdge,
                 class Hash = std::hash<V>, class Eq = std::equal_to<V>,
                 class Cmp = std::less<V>>
    requires EdgeCreateFn<CreateEdge, V, Handle> &&
             EdgeDestroyFn<DestroyEdge, Handle> &&
             std::strict_weak_order<Cmp, V, V>
class NoOpSkipGraphScheduler {
  public:
	using Vertex = V;
	using EdgeKey = std::pair<V, V>;
	using AdjSet = std::pmr::unordered_set<V, Hash, Eq>;
	using AdjMap = std::pmr::unordered_map<V, AdjSet, Hash, Eq>;
	using HandleMap = std::pmr::unordered_map<EdgeKey, Handle,
	                                          PairHash<V, Hash>, PairEq<V, Eq>>;

	NoOpSkipGraphScheduler(
	    CreateEdge create_edge, DestroyEdge destroy_edge,
	    std::pmr::memory_resource *mr = std::pmr::get_default_resource())
	    : base_adj_{mr} {}

	~NoOpSkipGraphScheduler() {}

	bool connect(const V &a, const V &b) {
		if (Eq{}(a, b))
			return false;
		bool inserted = add_base_adj_(a, b);
		if (!inserted)
			return false;
		return true;
	}

	bool disconnect(const V &a, const V &b) {
		if (Eq{}(a, b))
			return false;
		bool removed = remove_base_adj_(a, b);
		if (!removed)
			return false;
		return true;
	}

	auto aux_edges() const {
		return std::ranges::empty_view<const EdgeKey>{};
	}

	void clear() {
		base_adj_.clear();
	}

  private:
	bool add_base_adj_(const V &a, const V &b) {
		auto &A = base_adj_[a];
		auto [itA, insA] = A.insert(b);
		auto &B = base_adj_[b];
		auto [itB, insB] = B.insert(a);
		if (!(insA && insB)) {
			if (insA)
				A.erase(itA);
			if (insB)
				B.erase(itB);
			return false;
		}
		return true;
	}

	bool remove_base_adj_(const V &a, const V &b) {
		auto ita = base_adj_.find(a);
		if (ita == base_adj_.end())
			return false;
		auto itb = base_adj_.find(b);
		if (itb == base_adj_.end())
			return false;

		std::size_t ea = ita->second.erase(b);
		std::size_t eb = itb->second.erase(a);
		if (ea == 0 || eb == 0) {
			if (ea)
				ita->second.insert(b);
			if (eb)
				itb->second.insert(a);
			return false;
		}
		if (ita->second.empty())
			base_adj_.erase(ita);
		if (itb->second.empty())
			base_adj_.erase(itb);
		return true;
	}

  private:
	AdjMap base_adj_;
};

// Scheduler that builds/tears down auxiliary edges at skip distances.
export template <class V, class Handle, class CreateEdge, class DestroyEdge,
                 bool DestroyEdgesOnDestruct = true, int K = 8,
                 bool Stable = true, class Hash = std::hash<V>,
                 class Eq = std::equal_to<V>, class Cmp = std::less<V>>
    requires EdgeCreateFn<CreateEdge, V, Handle> &&
             EdgeDestroyFn<DestroyEdge, Handle> &&
             std::strict_weak_order<Cmp, V, V>
class SimpleSkipGraphScheduler {
  public:
	using Vertex = V;
	using EdgeKey = std::pair<V, V>;
	using AdjSet = std::pmr::unordered_set<V, Hash, Eq>;
	using AdjMap = std::pmr::unordered_map<V, AdjSet, Hash, Eq>;
	using HandleMap = std::pmr::unordered_map<EdgeKey, Handle,
	                                          PairHash<V, Hash>, PairEq<V, Eq>>;

	SimpleSkipGraphScheduler(
	    CreateEdge create_edge, DestroyEdge destroy_edge,
	    std::pmr::memory_resource *mr = std::pmr::get_default_resource())
	    : create_(std::move(create_edge)), destroy_(std::move(destroy_edge)),
	      mr_(mr), base_adj_(mr), aux_adj_(mr), aux_handles_(mr) {}

	~SimpleSkipGraphScheduler() {
		if (DestroyEdgesOnDestruct)
			clear();
	}

	bool connect(const V &a, const V &b) {
		if (Eq{}(a, b))
			return false;
		bool inserted = add_base_adj_(a, b);
		if (!inserted)
			return false;
		recompute_aux_();
		return true;
	}

	bool disconnect(const V &a, const V &b) {
		if (Eq{}(a, b))
			return false;
		bool removed = remove_base_adj_(a, b);
		if (!removed)
			return false;
		recompute_aux_();
		return true;
	}

	auto aux_edges() const {
		return std::views::keys(aux_handles_);
	}

	void clear() {
		for (auto &[_, h] : aux_handles_)
			destroy_(h);
		aux_handles_.clear();
		aux_adj_.clear();
		base_adj_.clear();
	}

  private:
	EdgeKey canon_(const V &u, const V &v) const {
		if (Cmp{}(v, u))
			return {v, u};
		return {u, v};
	}

	bool add_base_adj_(const V &a, const V &b) {
		auto &A = base_adj_[a];
		auto [itA, insA] = A.insert(b);
		auto &B = base_adj_[b];
		auto [itB, insB] = B.insert(a);
		if (!(insA && insB)) {
			if (insA)
				A.erase(itA);
			if (insB)
				B.erase(itB);
			return false;
		}
		return true;
	}

	bool remove_base_adj_(const V &a, const V &b) {
		auto ita = base_adj_.find(a);
		if (ita == base_adj_.end())
			return false;
		auto itb = base_adj_.find(b);
		if (itb == base_adj_.end())
			return false;

		std::size_t ea = ita->second.erase(b);
		std::size_t eb = itb->second.erase(a);
		if (ea == 0 || eb == 0) {
			if (ea)
				ita->second.insert(b);
			if (eb)
				itb->second.insert(a);
			return false;
		}
		if (ita->second.empty())
			base_adj_.erase(ita);
		if (itb->second.empty())
			base_adj_.erase(itb);
		return true;
	}

	std::pmr::vector<V> vertex_set_() const {
		std::pmr::vector<V> verts(mr_);
		verts.reserve(base_adj_.size() * 2);
		for (auto const &[u, nbrs] : base_adj_) {
			verts.push_back(u);
			for (auto const &v : nbrs)
				verts.push_back(v);
		}
		std::sort(verts.begin(), verts.end(), Cmp{});
		verts.erase(std::unique(verts.begin(), verts.end(), Eq{}), verts.end());
		return verts;
	}

	void recompute_aux_() {
		const AdjMap old_aux = aux_adj_;
		std::pmr::unordered_set<EdgeKey, PairHash<V, Hash>, PairEq<V, Eq>>
		    desired_aux(mr_);
		std::pmr::vector<int> targets(mr_);
		targets.reserve(static_cast<std::size_t>(K));
		for (int i = 1; i <= K; ++i)
			targets.push_back(1 << i);
		const int maxd = targets.back();
		auto verts = vertex_set_();
		for (const V &s : verts) {
			std::pmr::unordered_map<V, int, Hash, Eq> dist(mr_);
			dist.emplace(s, 0);
			std::pmr::deque<V> q(mr_);
			q.push_back(s);
			std::pmr::unordered_map<int, std::pmr::vector<V>> cand(mr_);
			for (int d : targets)
				cand.emplace(d, std::pmr::vector<V>(mr_));
			while (!q.empty()) {
				V u = q.front();
				q.pop_front();
				int d = dist[u];
				if (d > 0 && d <= maxd) {
					auto it = cand.find(d);
					if (it != cand.end() && !Eq{}(u, s))
						it->second.push_back(u);
				}
				if (d == maxd)
					continue;
				auto itN = base_adj_.find(u);
				if (itN == base_adj_.end())
					continue;
				for (auto const &w : itN->second) {
					if (!dist.contains(w)) {
						dist.emplace(w, d + 1);
						q.push_back(w);
					}
				}
			}
			for (int d : targets) {
				auto it = cand.find(d);
				if (it == cand.end() || it->second.empty())
					continue;
				const auto &lst = it->second;
				V chosen;
				if (Stable) {
					chosen = *std::min_element(lst.begin(), lst.end(), Cmp{});
				} else {
					const auto ia = old_aux.find(s);
					if (ia != old_aux.end()) {
						V best = V{};
						bool found = false;
						for (auto const &x : lst) {
							if (ia->second.contains(x)) {
								if (!found || Cmp{}(x, best)) {
									best = x;
									found = true;
								}
							}
						}
						if (found)
							chosen = best;
						else
							chosen = *std::min_element(lst.begin(), lst.end(),
							                           Cmp{});
					} else {
						chosen =
						    *std::min_element(lst.begin(), lst.end(), Cmp{});
					}
				}
				if (!Eq{}(s, chosen))
					desired_aux.insert(canon_(s, chosen));
			}
		}
		// Remove obsolete aux edges
		for (auto it = aux_handles_.begin(); it != aux_handles_.end();) {
			if (!desired_aux.contains(it->first)) {
				destroy_(it->second);
				auto [a, b] = it->first;
				auto ita = aux_adj_.find(a);
				if (ita != aux_adj_.end()) {
					ita->second.erase(b);
					if (ita->second.empty())
						aux_adj_.erase(ita);
				}
				auto itb = aux_adj_.find(b);
				if (itb != aux_adj_.end()) {
					itb->second.erase(a);
					if (itb->second.empty())
						aux_adj_.erase(itb);
				}
				it = aux_handles_.erase(it);
			} else {
				++it;
			}
		}
		// Add new aux edges
		for (auto const &e : desired_aux) {
			if (!aux_handles_.contains(e)) {
				auto h = create_(e.first, e.second);
				aux_handles_.emplace(e, std::move(h));
				aux_adj_[e.first].insert(e.second);
				aux_adj_[e.second].insert(e.first);
			}
		}
	}

  private:
	CreateEdge create_;
	DestroyEdge destroy_;
	std::pmr::memory_resource *mr_;
	AdjMap base_adj_;
	AdjMap aux_adj_;
	HandleMap aux_handles_;
};

} // namespace lego_assemble
