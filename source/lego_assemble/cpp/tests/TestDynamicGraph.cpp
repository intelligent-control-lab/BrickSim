import std;
import lego_assemble.utils.dynamic_graph;

#include <cassert>

using namespace lego_assemble;

static void test_vertex_add_delete_basic() {
	HolmDeLichtenbergThorup G(0);
	NaiveDynamicGraph O(0);

	std::vector<vertex_id> v(6);
	for (int i = 0; i < 6; ++i) {
		v[i] = G.add_vertex();
		O.add_vertex();
	}

	auto chk = [&](vertex_id a, vertex_id b) {
		bool g = G.connected(a, b);
		bool o = O.connected(a, b);
		return g == o;
	};

	for (int i = 0; i < 5; ++i) {
		assert(G.add_edge(v[i], v[i + 1]));
		assert(O.add_edge(v[i], v[i + 1]));
	}
	assert(chk(v[0], v[5]));

	assert(G.erase_vertex(v[3]));
	assert(O.erase_vertex(v[3]));

	assert(chk(v[0], v[2]));
	assert(chk(v[4], v[5]));
	assert(chk(v[0], v[5]));

	assert(G._debug_invariant_i_holds());
}

// Exercise multi-level cut correctness: promote some tree edges to level 1, then delete them.
static void test_cut_all_levels() {
	const int N = 6;
	HolmDeLichtenbergThorup G(N);
	NaiveDynamicGraph O(N);

	for (int i = 0; i + 1 < N; i++) {
		assert(G.add_edge(i, i + 1));
		assert(O.add_edge(i, i + 1));
	}
	// non-tree to serve as replacement when splitting the path
	assert(G.add_edge(0, N - 1));
	assert(O.add_edge(0, N - 1));

	// Delete middle tree edge to trigger Step-2 promotions on the small side
	assert(G.erase_edge(2, 3));
	assert(O.erase_edge(2, 3));

	// Now some tree edges are at level 1. Delete one of them; correctness requires cutting from F0 and F1.
	// Choose an edge on the promoted side; 1-2 is a good candidate in this construction.
	assert(G.erase_edge(1, 2));
	assert(O.erase_edge(1, 2));

	// Cross-check connectivity
	for (int a = 0; a < N; a++) {
		for (int b = 0; b < N; b++) {
			assert(G.connected(a, b) == O.connected(a, b));
		}
	}

	assert(G._debug_invariant_i_holds());
}

static void test_random(std::uint32_t seed = 20240229) {
	constexpr int INIT = 80;
	constexpr int OPS = 3000;
	std::mt19937_64 rng(seed);

	HolmDeLichtenbergThorup G(INIT);
	NaiveDynamicGraph O(INIT);

	auto pick = [&](int hi) -> vertex_id {
		return static_cast<vertex_id>(rng() % hi);
	};
	int max_id = INIT;

	for (int op = 0; op < OPS; ++op) {
		int t = static_cast<int>(rng() % 5);
		if (t == 0) {
			auto g = G.add_vertex();
			auto o = O.add_vertex();
			assert(g == o);
			max_id = std::max(max_id, (int)g + 1);
		} else if (t == 1) {
			vertex_id v = pick(max_id);
			assert(G.erase_vertex(v) == O.erase_vertex(v));
		} else if (t == 2) {
			vertex_id u = pick(max_id), v = pick(max_id);
			if (u == v)
				v = (v + 1) % std::max(1, max_id);
			assert(G.add_edge(u, v) == O.add_edge(u, v));
		} else if (t == 3) {
			vertex_id u = pick(max_id), v = pick(max_id);
			if (u == v)
				v = (v + 1) % std::max(1, max_id);
			assert(G.erase_edge(u, v) == O.erase_edge(u, v));
		} else {
			vertex_id u = pick(max_id), v = pick(max_id);
			assert(G.connected(u, v) == O.connected(u, v));
		}
		if ((op % 50) == 0) {
			assert(G._debug_invariant_i_holds());
		}
	}
}

struct FailResource : std::pmr::memory_resource {
	void *do_allocate(std::size_t, std::size_t) override {
		assert(false && "[PMR TEST] ERROR: default pmr used!");
		std::unreachable();
	}
	void do_deallocate(void *, std::size_t, std::size_t) override {
		assert(false && "[PMR TEST] ERROR: default pmr deallocate called!");
		std::unreachable();
	}
	bool do_is_equal(
	    const std::pmr::memory_resource &other) const noexcept override {
		return this == &other;
	}
};

struct CountingResource : std::pmr::memory_resource {
	std::pmr::memory_resource *upstream;
	std::atomic<std::size_t> allocs{0}, deallocs{0};
	std::atomic<std::size_t> bytes_alloc{0}, bytes_dealloc{0};

	explicit CountingResource(
	    std::pmr::memory_resource *up = std::pmr::new_delete_resource())
	    : upstream(up) {}

	void *do_allocate(std::size_t bytes, std::size_t align) override {
		allocs.fetch_add(1, std::memory_order_relaxed);
		bytes_alloc.fetch_add(bytes, std::memory_order_relaxed);
		return upstream->allocate(bytes, align);
	}
	void do_deallocate(void *p, std::size_t bytes, std::size_t align) override {
		deallocs.fetch_add(1, std::memory_order_relaxed);
		bytes_dealloc.fetch_add(bytes, std::memory_order_relaxed);
		upstream->deallocate(p, bytes, align);
	}
	bool do_is_equal(
	    const std::pmr::memory_resource &other) const noexcept override {
		return this == &other;
	}
};

struct DefaultResourceGuard {
	std::pmr::memory_resource *prev{};
	explicit DefaultResourceGuard(std::pmr::memory_resource *r)
	    : prev(std::pmr::set_default_resource(r)) {}
	~DefaultResourceGuard() {
		std::pmr::set_default_resource(prev);
	}
};

static void test_naive_dynamic_graph_uses_given_pmr() {
	FailResource fail;
	DefaultResourceGuard guard(&fail); // any use of default PMR aborts

	CountingResource arena;

	// 1) Build with explicit arena and force allocations in reset/assign
	NaiveDynamicGraph g(0, &arena);
	g.reset(64); // adj_.assign + alive_.assign allocate

	// 2) Force adj_ growth (vector reallocation) and inner set construction
	for (int i = 0; i < 200; ++i)
		(void)g.add_vertex(); // grow adj_, alive_, free_ path later

	// 3) Create a high-degree star to force unordered_set bucket/node allocations and rehash
	for (vertex_id v = 1; v < 180; ++v) {
		[[maybe_unused]] bool ok = g.add_edge(0, v);
		(void)ok;
	}

	// 4) BFS allocations: pmr::deque and pmr::vector in connected()
	[[maybe_unused]] bool conn = g.connected(0, 179);

	// 5) Force erase_vertex path that allocates a pmr::vector<vertex_id> neigh with reserve()
	[[maybe_unused]] bool erased = g.erase_vertex(100);

	// 6) Fill free_ to force its growth (pmr::vector reallocation)
	for (vertex_id v = 150; v < 200; ++v) {
		(void)g.erase_vertex(v);
	}

	// 7) Trigger more unordered_set activity (erase, then re-add)
	for (vertex_id v = 1; v < 90; ++v) {
		(void)g.erase_edge(0, v);
	}
	for (vertex_id v = 1; v < 90; ++v) {
		(void)g.add_edge(0, v);
	}

	// 8) Clear to free pmr allocations (dealloc counters should move)
	g.clear();

	// If we reached here, default PMR was never used (would have aborted).
	// Also sanity-check we actually allocated via our arena.
	assert(arena.allocs.load() > 0 &&
	       "No allocations observed on arena for NaiveDynamicGraph");
}

static void test_hlt_uses_given_pmr_and_exercises_allocation_branches() {
	FailResource fail;
	DefaultResourceGuard guard(&fail); // any use of default PMR aborts

	CountingResource arena;

	// Construct with n0>0 to exercise reset(cap_, L_) and initial emplace_back of forests_.
	// IMPORTANT: This will immediately fail if LinkCutForest objects inside forests_
	// are created with the default PMR (i.e., if you forgot to pass mr_).
	HolmDeLichtenbergThorup hlt(8, &arena);

	// 1) Touch Link-Cut Forest splay path allocations via connected()
	[[maybe_unused]] bool c01 =
	    hlt.connected(0, 1); // causes push_path stack alloc in LCT

	// 2) Grow capacity to add new levels/forests and resize nested structures
	for (int i = 0; i < 40; ++i) {
		(void)hlt.add_vertex();
	} // triggers grow_capacity()

	// 3) Build a tree (chain) to allocate in treeAdj_, forests_ link() paths
	for (vertex_id v = 0; v + 1 < 20; ++v) {
		[[maybe_unused]] bool ok = hlt.add_edge(v, v + 1);
		(void)ok;
	}

	// 4) Add several non-tree edges at level 0 to allocate in nonTreeAdj_[0] sets + edges_ map
	// They are all already connected through the chain.
	auto add_nt = [&](vertex_id u, vertex_id v) {
		[[maybe_unused]] bool ok = hlt.add_edge(u, v);
		(void)ok;
	};
	add_nt(0, 2);
	add_nt(2, 4);
	add_nt(4, 6);
	add_nt(6, 8);
	add_nt(1, 3);
	add_nt(3, 5);
	add_nt(5, 7);
	add_nt(7, 9);

	// 5) Non-tree erase branch (no replacement search)
	[[maybe_unused]] bool er1 = hlt.erase_edge(7, 9);

	// 6) Tree erase + replacement exists (should convert a non-tree crossing edge into a tree edge)
	// Cutting the middle edge splits the chain; (2,4) or (4,6) crosses and should be chosen.
	[[maybe_unused]] bool er2 =
	    hlt.erase_edge(9, 10); // remove end tree edge first (no replacement)
	[[maybe_unused]] bool er3 =
	    hlt.erase_edge(3, 4); // central cut ⇒ replacement search at level 0

	// 7) Force internal non-tree promotions (Step 3) by cutting again elsewhere
	// so a non-tree internal to the small side gets promoted to the next level.
	[[maybe_unused]] bool er4 =
	    hlt.erase_edge(11, 12); // another tree edge erase at a different place

	// 8) Exercise enumerate_Fi_component() BFS allocations
	[[maybe_unused]] bool c = hlt.connected(0, 12);

	// 9) Exercise component_size() which hits LCT push_path and splay again
	[[maybe_unused]] std::size_t s = hlt.num_vertices();
	(void)s;

	// 10) Clear to free pmr memory everywhere
	hlt.clear();

	// If we’re here, no default PMR was used (allocation would have aborted).
	assert(arena.allocs.load() > 0 &&
	       "No allocations observed on arena for HLT");
}

static void test_visit_path_both_impls() {
	HolmDeLichtenbergThorup G(0);
	NaiveDynamicGraph O(0);

	const int N = 16;
	for (int i = 0; i < N; ++i) {
		auto g = G.add_vertex();
		auto o = O.add_vertex();
		assert(g == o);
	}

	// Track the set of present (undirected) edges for validation.
	auto pack = [](vertex_id a, vertex_id b) -> std::uint64_t {
		if (a > b)
			std::swap(a, b);
		return (static_cast<std::uint64_t>(a) << 32) ^
		       static_cast<std::uint64_t>(b);
	};
	std::unordered_set<std::uint64_t> edges;

	auto addE = [&](vertex_id u, vertex_id v) {
		bool g = G.add_edge(u, v);
		bool o = O.add_edge(u, v);
		assert(g == o);
		if (g)
			edges.insert(pack(u, v));
	};

	// Build a chain (tree) plus some chords (non-tree edges).
	for (int i = 0; i + 1 < N; ++i)
		addE(i, i + 1); // chain
	addE(0, 3);
	addE(2, 5);
	addE(4, 7);
	addE(6, 9); // chords
	addE(1, 4);
	addE(3, 6);
	addE(5, 8);     // more chords
	addE(0, N - 1); // long chord

	// Helper to validate a visited path against the inserted edge set.
	auto check_path =
	    [&](vertex_id s, vertex_id t,
	        const std::vector<std::pair<vertex_id, vertex_id>> &E) {
		    if (s == t) {
			    assert(E.empty());
			    return;
		    }
		    assert(!E.empty());
		    assert(E.front().first == s);
		    assert(E.back().second == t);
		    vertex_id cur = s;
		    for (auto [a, b] : E) {
			    assert(a == cur);
			    assert(edges.count(pack(a, b)) &&
			           "visited step must be an actual edge");
			    cur = b;
		    }
		    assert(cur == t);
	    };

	// Random queries; compare connectivity and validate each visited path.
	std::mt19937_64 rng(0xBADC0FFEEULL);
	for (int it = 0; it < 200; ++it) {
		vertex_id u = static_cast<vertex_id>(rng() % N);
		vertex_id v = static_cast<vertex_id>(rng() % N);

		bool cG = G.connected(u, v);
		bool cO = O.connected(u, v);
		assert(cG == cO);

		std::vector<std::pair<vertex_id, vertex_id>> pg, po;
		bool rG = G.visit_path(
		    u, v, [&](vertex_id a, vertex_id b) { pg.emplace_back(a, b); });
		bool rO = O.visit_path(
		    u, v, [&](vertex_id a, vertex_id b) { po.emplace_back(a, b); });

		assert(rG == cG);
		assert(rO == cO);

		if (cG) {
			check_path(u, v, pg); // HLT path (tree path)
			check_path(u, v, po); // Naive path (any simple path)
		} else {
			assert(pg.empty());
			assert(po.empty());
		}
	}

	// A quick "dead vertex" check: erase a vertex and ensure visit_path fails.
	assert(G.erase_vertex(5) == O.erase_vertex(5));
	std::vector<std::pair<vertex_id, vertex_id>> tmp;
	assert(G.visit_path(5, 2, [&](vertex_id, vertex_id) {
		tmp.emplace_back(0, 0);
	}) == false);
	assert(tmp.empty());
	tmp.clear();
	assert(O.visit_path(5, 2, [&](vertex_id, vertex_id) {
		tmp.emplace_back(0, 0);
	}) == false);
	assert(tmp.empty());
}

// ==================
// Benchmark
// ==================

namespace {

// --- Workload description ---------------------------------------------------

enum class OpKind : std::uint8_t { Add, Del, Conn };
struct Op {
	OpKind k;
	vertex_id u, v;
};

struct Workload {
	std::string name;
	std::size_t N{};
	std::vector<std::pair<vertex_id, vertex_id>> initial_edges;
	std::vector<Op> ops;
	std::size_t n_add{}, n_del{}, n_qry{};
	std::uint64_t seed{};
};

static inline std::pair<vertex_id, vertex_id> canon_edge(vertex_id a,
                                                         vertex_id b) noexcept {
	if (a > b)
		std::swap(a, b);
	return {a, b};
}

struct PairHash {
	std::size_t
	operator()(const std::pair<vertex_id, vertex_id> &p) const noexcept {
		// 32-bit ids; mix to 64-bit then splitmix
		std::uint64_t x =
		    (std::uint64_t{p.first} << 32) ^ p.second ^ 0x9e3779b97f4a7c15ull;
		x ^= x >> 30;
		x *= 0xbf58476d1ce4e5b9ull;
		x ^= x >> 27;
		x *= 0x94d049bb133111ebull;
		x ^= x >> 31;
		return static_cast<std::size_t>(x);
	}
};

// Build a random dynamic workload over a fixed vertex set.
// - Starts with `init_edges` random edges.
// - Then performs `ops` operations with given ratios (p_ins, p_qry, rest deletions).
static Workload make_random_workload(std::string name, std::size_t N,
                                     std::size_t init_edges, std::size_t ops,
                                     double p_ins, double p_qry,
                                     std::uint64_t seed) {
	std::mt19937_64 rng(seed);
	std::uniform_int_distribution<std::uint32_t> U(
	    0u, static_cast<std::uint32_t>(N - 1));

	std::unordered_set<std::pair<vertex_id, vertex_id>, PairHash> present;
	present.reserve(init_edges * 2 + 1024);

	auto sample_pair = [&] {
		vertex_id u = static_cast<vertex_id>(U(rng));
		vertex_id v = static_cast<vertex_id>(U(rng));
		if (u == v)
			v = static_cast<vertex_id>((v + 1) % N);
		return canon_edge(u, v);
	};

	Workload W;
	W.name = std::move(name);
	W.N = N;
	W.seed = seed;
	W.initial_edges.reserve(init_edges);

	// Initial edges
	while (W.initial_edges.size() < init_edges) {
		auto e = sample_pair();
		if (present.insert(e).second)
			W.initial_edges.push_back(e);
	}

	// Ops
	W.ops.reserve(ops);
	std::size_t max_edges = N * (N - 1) / 2;
	for (std::size_t i = 0; i < ops; ++i) {
		double r = std::generate_canonical<double, 53>(rng);
		OpKind k;
		if (r < p_ins)
			k = OpKind::Add;
		else if (r < p_ins + p_qry)
			k = OpKind::Conn;
		else
			k = OpKind::Del;

		// Avoid impossible ops at extremes
		if (k == OpKind::Add && present.size() >= max_edges)
			k = OpKind::Del;
		if (k == OpKind::Del && present.empty())
			k = OpKind::Add;

		if (k == OpKind::Conn) {
			auto [u, v] = sample_pair();
			W.ops.push_back(Op{OpKind::Conn, u, v});
			++W.n_qry;
		} else if (k == OpKind::Add) {
			// Try until we get a non-present edge
			std::pair<vertex_id, vertex_id> e;
			do {
				e = sample_pair();
			} while (present.count(e));
			present.insert(e);
			W.ops.push_back(Op{OpKind::Add, e.first, e.second});
			++W.n_add;
		} else {
			// Delete a random present edge
			std::size_t idx = static_cast<std::size_t>(rng() % present.size());
			auto it = present.begin();
			std::advance(it, static_cast<long>(idx));
			auto e = *it;
			present.erase(it);
			W.ops.push_back(Op{OpKind::Del, e.first, e.second});
			++W.n_del;
		}
	}
	return W;
}

// Purpose-built workload that forces HLT replacement searches:
// 1) start with a path 0-1-2-...-(N-1)
// 2) add evenly spaced chords (non-tree)
// 3) repeatedly cut middle tree edges + interleave queries
static Workload make_chain_cut_replacement(std::string name, std::size_t N,
                                           std::size_t chord_stride,
                                           std::size_t rounds,
                                           std::uint64_t seed) {
	std::mt19937_64 rng(seed);
	Workload W;
	W.name = std::move(name);
	W.N = N;
	W.seed = seed;

	// Initial path tree
	for (vertex_id i = 0; i + 1 < N; ++i)
		W.initial_edges.emplace_back(i, i + 1);

	// Add non-tree chords at stride (i, i+stride)
	for (vertex_id i = 0; i + chord_stride < N; i += chord_stride)
		W.initial_edges.emplace_back(i,
		                             static_cast<vertex_id>(i + chord_stride));

	// Repeatedly cut near the middle and query random pairs
	std::uniform_int_distribution<std::uint32_t> U(
	    0u, static_cast<std::uint32_t>(N - 1));
	vertex_id midL = static_cast<vertex_id>((N - 1) / 2 - 1);
	vertex_id midR = static_cast<vertex_id>((N - 1) / 2);

	for (std::size_t r = 0; r < rounds; ++r) {
		// Cut a central tree edge to split path
		W.ops.push_back(Op{OpKind::Del, midL, midR});
		++W.n_del;

		// Add a few queries probing across the cut
		for (int q = 0; q < 5; ++q) {
			vertex_id u = static_cast<vertex_id>(U(rng));
			vertex_id v = static_cast<vertex_id>(U(rng));
			if (u == v)
				v = static_cast<vertex_id>((v + 1) % N);
			W.ops.push_back(Op{OpKind::Conn, u, v});
			++W.n_qry;
		}

		// Re-stitch by adding back the edge (may fail if replacement edge chosen first)
		W.ops.push_back(Op{OpKind::Add, midL, midR});
		++W.n_add;
	}
	return W;
}

// --- Benchmark runner -------------------------------------------------------

template <class G>
static std::uint64_t
build_graph(G &g, std::size_t N,
            const std::vector<std::pair<vertex_id, vertex_id>> &init) {
	for (std::size_t i = 0; i < N; ++i)
		(void)g.add_vertex();
	std::uint64_t ok = 0;
	for (auto [u, v] : init)
		ok += g.add_edge(u, v) ? 1 : 0;
	return ok;
}

template <class G>
static std::pair<std::uint64_t, std::uint64_t>
apply_ops(G &g, const std::vector<Op> &ops) {
	using clk = std::chrono::steady_clock;
	volatile std::uint64_t checksum = 0; // keep queries "live"
	auto t0 = clk::now();
	for (const Op &op : ops) {
		switch (op.k) {
		case OpKind::Add:
			(void)g.add_edge(op.u, op.v);
			break;
		case OpKind::Del:
			(void)g.erase_edge(op.u, op.v);
			break;
		case OpKind::Conn:
			checksum ^= static_cast<std::uint64_t>(g.connected(op.u, op.v));
			break;
		}
	}
	auto t1 = clk::now();
	auto ns = static_cast<std::uint64_t>(
	    std::chrono::duration_cast<std::chrono::nanoseconds>(t1 - t0).count());
	return {ns, checksum};
}

struct BenchRes {
	std::string impl;
	std::string wname;
	std::size_t N{};
	std::size_t ops{};
	std::size_t n_add{}, n_del{}, n_qry{};
	std::uint64_t ns{};
	double ns_per_op{};
	double mops{};
	std::uint64_t checksum{};
};

template <class G>
static BenchRes
run_once(const char *impl_name, const Workload &W,
         std::pmr::memory_resource *mr = std::pmr::get_default_resource()) {
	G g(0, mr);
	(void)build_graph(g, W.N, W.initial_edges);
	auto [ns, sum] = apply_ops(g, W.ops);
	BenchRes R;
	R.impl = impl_name;
	R.wname = W.name;
	R.N = W.N;
	R.ops = W.ops.size();
	R.n_add = W.n_add;
	R.n_del = W.n_del;
	R.n_qry = W.n_qry;
	R.ns = ns;
	R.ns_per_op = R.ops ? double(ns) / double(R.ops) : 0.0;
	R.mops = R.ns ? (1e3 / (R.ns_per_op))
	              : 0.0; // Mops/s = (1e9 ns/s) / (ns/op) / 1e6
	R.checksum = sum;
	return R;
}

static void print_result(const BenchRes &A, const BenchRes &B) {
	using std::cout;
	using std::left;
	using std::right;
	using std::setw;
	cout << "\n=== Workload: " << A.wname << " ===\n";
	cout << "N=" << A.N << ", ops=" << A.ops << "  [adds=" << A.n_add
	     << " dels=" << A.n_del << " qrys=" << A.n_qry << "]\n";
	auto row = [&](const BenchRes &R) {
		cout << "  " << std::left << setw(26) << R.impl << " time "
		     << std::right << setw(12) << R.ns << " ns"
		     << "  ns/op " << std::setw(8) << std::fixed << std::setprecision(2)
		     << R.ns_per_op << "  Mops/s " << std::setw(6)
		     << std::setprecision(2) << R.mops << "  checksum " << R.checksum
		     << "\n";
	};
	row(A);
	row(B);
	double speedup = (B.ns && A.ns) ? double(B.ns) / double(A.ns) : 0.0;
	cout << "  Speedup (" << A.impl << " / " << B.impl << "): " << std::fixed
	     << std::setprecision(2) << speedup << "×\n";
}

// Convenience: run each workload twice and take the best (lower) time to reduce jitter.
template <class G>
static BenchRes best_of_two(const char *impl, const Workload &W) {
	auto R1 = run_once<G>(impl, W);
	auto R2 = run_once<G>(impl, W);
	return (R1.ns <= R2.ns) ? R1 : R2;
}

} // namespace

static void run_performance_benchmarks() {
	std::vector<Workload> suite;
	suite.push_back(make_random_workload(
	    "Query-heavy giant component (80% qry, 10% add, 10% del)",
	    /*N=*/20000,
	    /*init_edges=*/3 * 20000,
	    /*ops=*/4000,
	    /*p_ins=*/0.10,
	    /*p_qry=*/0.80,
	    /*seed=*/0xC0FFEE01ull));

	suite.push_back(make_random_workload(
	    "Moderate-queries giant component (30% qry, 35% add, 35% del)",
	    /*N=*/20000,
	    /*init_edges=*/3 * 20000,
	    /*ops=*/4000,
	    /*p_ins=*/0.35,
	    /*p_qry=*/0.30,
	    /*seed=*/0xC0FFEE02ull));

	suite.push_back(make_chain_cut_replacement(
	    "Chain+chords: repeated central cuts (replacement stress)",
	    /*N=*/30000,
	    /*chord_stride=*/4,
	    /*rounds=*/200,
	    /*seed=*/0xC0FFEE03ull));

	for (const auto &W : suite) {
		auto H =
		    best_of_two<HolmDeLichtenbergThorup>("HolmDeLichtenbergThorup", W);
		auto N = best_of_two<NaiveDynamicGraph>("NaiveDynamicGraph", W);
		print_result(H, N);
	}
}

int main() {
	test_vertex_add_delete_basic();
	test_cut_all_levels();
	test_random();
	test_naive_dynamic_graph_uses_given_pmr();
	test_hlt_uses_given_pmr_and_exercises_allocation_branches();
	test_visit_path_both_impls();
	// run_performance_benchmarks();
	return 0;
}
