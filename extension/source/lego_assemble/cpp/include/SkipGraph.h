#pragma once

#include <algorithm>
#include <cassert>
#include <concepts>
#include <deque>
#include <functional>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace lego_assemble {

// ---------- Concepts ----------
template <class Create, class V, class Handle>
concept EdgeCreateFn = requires(Create c, const V &a, const V &b) {
	{ c(a, b) } -> std::same_as<Handle>;
};

template <class Destroy, class Handle>
concept EdgeDestroyFn = requires(Destroy d, Handle h) {
	{ d(h) } -> std::same_as<void>;
};

template <class Cmp, class V>
concept LessLike = requires(Cmp cmp, const V &a, const V &b) {
	{ cmp(a, b) } -> std::convertible_to<bool>;
	{ cmp(b, a) } -> std::convertible_to<bool>;
};

// ---------- Utility: hash for pair using underlying Hash ----------
template <class V, class Hash> struct PairHash {
	size_t operator()(const std::pair<V, V> &p) const noexcept {
		size_t h1 = Hash{}(p.first);
		size_t h2 = Hash{}(p.second);
		// A standard hash-combine (boost-like)
		h2 ^= h1 + 0x9e3779b97f4a7c15ULL + (h2 << 6) + (h2 >> 2);
		return h2;
	}
};

template <class V, class Eq> struct PairEq {
	bool operator()(const std::pair<V, V> &a,
	                const std::pair<V, V> &b) const noexcept {
		return Eq{}(a.first, b.first) && Eq{}(a.second, b.second);
	}
};

template <
    class V,      // Vertex / body id
    class Handle, // Edge handle returned by CreateEdge and fed to DestroyEdge
    class CreateEdge,            // (V const&, V const&) -> Handle
    class DestroyEdge,           // (Handle) -> void
    class Hash = std::hash<V>,   // hash for V
    class Eq = std::equal_to<V>, // equality for V
    class Cmp = std::less<V>     // comparator for stable ordering (min)
    >
    requires EdgeCreateFn<CreateEdge, V, Handle> &&
             EdgeDestroyFn<DestroyEdge, Handle> && LessLike<Cmp, V>
class NoOpSkipGraphScheduler {
  public:
	using Vertex = V;
	using EdgeKey = std::pair<V, V>; // canonicalized (a<=b by cmp_)
	using AdjSet = std::unordered_set<V, Hash, Eq>;
	using AdjMap = std::unordered_map<V, AdjSet, Hash, Eq>;
	using HandleMap =
	    std::unordered_map<EdgeKey, Handle, PairHash<V, Hash>, PairEq<V, Eq>>;

	NoOpSkipGraphScheduler(CreateEdge create_edge, DestroyEdge destroy_edge,
	                       Hash h = Hash{}, Eq e = Eq{}, Cmp cmp = Cmp{})
	    : eq_(std::move(e)), cmp_(std::move(cmp)) {}

	~NoOpSkipGraphScheduler() {}

	// ---- Base graph API ----
	// You call this when your base/body constraints should exist between a and b.
	bool connect(const V &a, const V &b) {
		if (eq_(a, b))
			return false;
		bool inserted = add_base_adj_(a, b);
		if (!inserted)
			return false;
		return true;
	}

	// You call this when your base constraint should be removed.
	bool disconnect(const V &a, const V &b) {
		if (eq_(a, b))
			return false;
		bool removed = remove_base_adj_(a, b);
		if (!removed)
			return false;
		return true;
	}

	// ---- Inspection ----
	std::vector<EdgeKey> aux_edges() const {
		return {};
	}

	// base adjacency snapshot (for diagnostics)
	const AdjMap &base_adjacency() const noexcept {
		return base_adj_;
	}

	void clear() {
		base_adj_.clear();
	}

  private:
	// Add/remove base adjacency
	bool add_base_adj_(const V &a, const V &b) {
		auto &A = base_adj_[a];
		auto [itA, insA] = A.insert(b);
		auto &B = base_adj_[b];
		auto [itB, insB] = B.insert(a);
		if (!(insA && insB)) {
			// rollback partial insert to keep symmetry
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

		size_t ea = ita->second.erase(b);
		size_t eb = itb->second.erase(a);
		if (ea == 0 || eb == 0) {
			// keep consistent
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
	// functors
	Eq eq_;
	Cmp cmp_;

	// state
	AdjMap base_adj_;
};

template <
    class V,      // Vertex / body id
    class Handle, // Edge handle returned by CreateEdge and fed to DestroyEdge
    class CreateEdge,                   // (V const&, V const&) -> Handle
    class DestroyEdge,                  // (Handle) -> void
    bool DestroyEdgesOnDestruct = true, // destroy all edges on dtor
    int K = 8, // number of skip-levels (distances 2^1 ... 2^K)
    bool Stable =
        true, // pick min target deterministically; otherwise prefer existing aux link
    class Hash = std::hash<V>,   // hash for V
    class Eq = std::equal_to<V>, // equality for V
    class Cmp = std::less<V>     // comparator for stable ordering (min)
    >
    requires EdgeCreateFn<CreateEdge, V, Handle> &&
             EdgeDestroyFn<DestroyEdge, Handle> && LessLike<Cmp, V>
class SimpleSkipGraphScheduler {
  public:
	using Vertex = V;
	using EdgeKey = std::pair<V, V>; // canonicalized (a<=b by cmp_)
	using AdjSet = std::unordered_set<V, Hash, Eq>;
	using AdjMap = std::unordered_map<V, AdjSet, Hash, Eq>;
	using HandleMap =
	    std::unordered_map<EdgeKey, Handle, PairHash<V, Hash>, PairEq<V, Eq>>;

	SimpleSkipGraphScheduler(CreateEdge create_edge, DestroyEdge destroy_edge,
	                         Hash h = Hash{}, Eq e = Eq{}, Cmp cmp = Cmp{})
	    : create_(std::move(create_edge)), destroy_(std::move(destroy_edge)),
	      hash_(std::move(h)), eq_(std::move(e)), cmp_(std::move(cmp)) {}

	~SimpleSkipGraphScheduler() {
		if (DestroyEdgesOnDestruct) {
			// destroy all edges we created
			clear();
		}
	}

	// ---- Base graph API ----
	// You call this when your base/body constraints should exist between a and b.
	bool connect(const V &a, const V &b) {
		if (eq_(a, b))
			return false;
		bool inserted = add_base_adj_(a, b);
		if (!inserted)
			return false;
		recompute_aux_();
		return true;
	}

	// You call this when your base constraint should be removed.
	bool disconnect(const V &a, const V &b) {
		if (eq_(a, b))
			return false;
		bool removed = remove_base_adj_(a, b);
		if (!removed)
			return false;
		recompute_aux_();
		return true;
	}

	// ---- Inspection ----
	std::vector<EdgeKey> aux_edges() const {
		std::vector<EdgeKey> out;
		out.reserve(aux_handles_.size());
		for (auto const &[e, _] : aux_handles_)
			out.push_back(e);
		return out;
	}

	// base adjacency snapshot (for diagnostics)
	const AdjMap &base_adjacency() const noexcept {
		return base_adj_;
	}
	const AdjMap &aux_adjacency() const noexcept {
		return aux_adj_;
	}

	void clear() {
		// destroy aux edges first
		for (auto &[_, h] : aux_handles_)
			destroy_(h);
		aux_handles_.clear();
		aux_adj_.clear();
		base_adj_.clear();
	}

  private:
	// -------- internals --------
	// Canonicalize edge as (min,max) according to cmp_
	EdgeKey canon_(const V &u, const V &v) const {
		if (cmp_(v, u))
			return {v, u};
		return {u, v};
	}

	// Add/remove base adjacency
	bool add_base_adj_(const V &a, const V &b) {
		auto &A = base_adj_[a];
		auto [itA, insA] = A.insert(b);
		auto &B = base_adj_[b];
		auto [itB, insB] = B.insert(a);
		if (!(insA && insB)) {
			// rollback partial insert to keep symmetry
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

		size_t ea = ita->second.erase(b);
		size_t eb = itb->second.erase(a);
		if (ea == 0 || eb == 0) {
			// keep consistent
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

	// Collect current vertex set from base graph only (like Python code)
	std::vector<V> vertex_set_() const {
		std::vector<V> verts;
		verts.reserve(base_adj_.size() * 2);
		for (auto const &[u, nbrs] : base_adj_) {
			verts.push_back(u);
			for (auto const &v : nbrs)
				verts.push_back(v);
		}
		// de-dup
		std::sort(verts.begin(), verts.end(),
		          [&](auto const &x, auto const &y) { return cmp_(x, y); });
		verts.erase(std::unique(verts.begin(), verts.end(),
		                        [&](auto const &x, auto const &y) {
			                        return eq_(x, y);
		                        }),
		            verts.end());
		return verts;
	}

	// Compute desired aux edges and reconcile with current aux set
	void recompute_aux_() {
		// Keep a snapshot of old aux for "preference" in unstable mode
		const AdjMap old_aux = aux_adj_;

		// Rebuild target aux set from scratch
		std::unordered_set<EdgeKey, PairHash<V, Hash>, PairEq<V, Eq>>
		    desired_aux;

		// Precompute distances we want: 2^1 .. 2^K
		std::vector<int> targets;
		targets.reserve(static_cast<size_t>(K));
		for (int i = 1; i <= K; ++i)
			targets.push_back(1 << i);
		const int maxd = targets.back();

		auto verts = vertex_set_();
		// BFS from each vertex
		for (const V &s : verts) {
			// distance map
			std::unordered_map<V, int, Hash, Eq> dist;
			dist.emplace(s, 0);
			std::deque<V> q;
			q.push_back(s);

			// candidates per requested distance
			std::unordered_map<int, std::vector<V>> cand;
			for (int d : targets)
				cand.emplace(d, std::vector<V>{});

			while (!q.empty()) {
				V u = q.front();
				q.pop_front();
				int d = dist[u];
				if (d > 0 && d <= maxd) {
					auto it = cand.find(d);
					if (it != cand.end() && !eq_(u, s)) {
						it->second.push_back(u);
					}
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

			// choose one target per requested distance
			for (int d : targets) {
				auto it = cand.find(d);
				if (it == cand.end() || it->second.empty())
					continue;

				const auto &lst = it->second;
				V chosen;

				if (Stable) {
					chosen =
					    *std::min_element(lst.begin(), lst.end(),
					                      [&](auto const &x, auto const &y) {
						                      return cmp_(x, y);
					                      });
				} else {
					// prefer an existing aux neighbor at this distance (if any), else pick min
					const auto ia = old_aux.find(s);
					if (ia != old_aux.end()) {
						// find intersection lst ∩ old_aux[s]
						V best = V{};
						bool found = false;
						for (auto const &x : lst) {
							if (ia->second.contains(x)) {
								if (!found || cmp_(x, best)) {
									best = x;
									found = true;
								}
							}
						}
						if (found)
							chosen = best;
						else {
							chosen = *std::min_element(
							    lst.begin(), lst.end(),
							    [&](auto const &x, auto const &y) {
								    return cmp_(x, y);
							    });
						}
					} else {
						chosen = *std::min_element(
						    lst.begin(), lst.end(),
						    [&](auto const &x, auto const &y) {
							    return cmp_(x, y);
						    });
					}
				}

				if (!eq_(s, chosen)) {
					desired_aux.insert(canon_(s, chosen));
				}
			}
		}

		// Diff: remove obsolete aux edges
		// (Use aux_handles_ as the current set of realized aux edges)
		for (auto it = aux_handles_.begin(); it != aux_handles_.end();) {
			if (!desired_aux.contains(it->first)) {
				destroy_(it->second);
				// update aux adjacency
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

		// Add newly required aux edges
		for (auto const &e : desired_aux) {
			if (!aux_handles_.contains(e)) {
				auto h = create_(e.first, e.second);
				aux_handles_.emplace(e, std::move(h));
				aux_adj_[e.first].insert(e.second);
				aux_adj_[e.second].insert(e.first);
			}
		}

		// Ensure aux_adj_ contains only realized aux edges (already maintained above)
	}

  private:
	// injected ops
	CreateEdge create_;
	DestroyEdge destroy_;

	// functors
	Hash hash_;
	Eq eq_;
	Cmp cmp_;

	// state
	AdjMap base_adj_;
	AdjMap aux_adj_;
	HandleMap aux_handles_;
};

} // namespace lego_assemble
