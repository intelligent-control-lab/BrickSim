import std;
import lego_assemble.core.specs;
import lego_assemble.core.graph;
import lego_assemble.utils.unordered_pair;
import lego_assemble.physx.constraint_scheduler;

#include <cassert>

using namespace lego_assemble;

// -----------------------------------------------------------------------------
// Minimal test graph + ComponentView, only what the policy needs
// -----------------------------------------------------------------------------

struct TestGraph {
	struct ComponentView {
		std::vector<PartId> verts;

		std::size_t size() const {
			return verts.size();
		}

		std::generator<PartId> vertices() const {
			for (PartId pid : verts) {
				co_yield pid;
			}
		}
	};
};

// Bring the policy into scope (adjust name/namespace if you put it elsewhere)
template <class Graph>
using RamanujanPolicy = lego_assemble::RamanujanLikeSchedulingPolicy<Graph>;

// Helper: build adjacency from the edge set
static std::unordered_map<PartId, std::unordered_set<PartId>>
build_adjacency(const std::vector<UnorderedPair<PartId>> &edges) {
	std::unordered_map<PartId, std::unordered_set<PartId>> adj;
	for (const auto &ek : edges) {
		auto [a, b] = ek;
		adj[a].insert(b);
		adj[b].insert(a);
	}
	return adj;
}

// -----------------------------------------------------------------------------
// Test 1: empty component -> no edges
// -----------------------------------------------------------------------------

static void test_empty_component() {
	TestGraph::ComponentView cc;
	cc.verts = {}; // empty

	RamanujanPolicy<TestGraph> policy(/*degree=*/4);

	std::size_t edge_count = 0;
	for (auto ek : policy.compute(cc)) {
		(void)ek;
		++edge_count;
	}
	assert(edge_count == 0);
}

// -----------------------------------------------------------------------------
// Test 2: single-vertex component -> no edges
// -----------------------------------------------------------------------------

static void test_single_vertex_component() {
	TestGraph::ComponentView cc;
	cc.verts = {PartId(1)};

	RamanujanPolicy<TestGraph> policy(/*degree=*/4);

	std::size_t edge_count = 0;
	for (auto ek : policy.compute(cc)) {
		(void)ek;
		++edge_count;
	}
	assert(edge_count == 0);
}

// -----------------------------------------------------------------------------
// Test 3: degree == 0 -> no edges, even for non-empty CC
// -----------------------------------------------------------------------------

static void test_zero_degree() {
	TestGraph::ComponentView cc;
	cc.verts = {PartId(1), PartId(2), PartId(3)};

	RamanujanPolicy<TestGraph> policy(/*degree=*/0);

	std::size_t edge_count = 0;
	for (auto ek : policy.compute(cc)) {
		(void)ek;
		++edge_count;
	}
	assert(edge_count == 0);
}

// -----------------------------------------------------------------------------
// Test 4: small CC invariants: no self loops, no multi-edges, degree <= target
// -----------------------------------------------------------------------------

static void test_small_cc_invariants() {
	TestGraph::ComponentView cc;
	cc.verts = {PartId(10), PartId(11), PartId(12), PartId(13), PartId(14)};

	const std::size_t target_degree = 3;
	RamanujanPolicy<TestGraph> policy(target_degree);

	std::unordered_set<UnorderedPair<PartId>> edge_set;
	std::vector<UnorderedPair<PartId>> edges;

	for (auto ek : policy.compute(cc)) {
		// no duplicate edges
		assert(edge_set.insert(ek).second);
		edges.push_back(ek);
	}

	auto adj = build_adjacency(edges);

	// 1) No self-loops
	for (const auto &ek : edges) {
		auto [a, b] = ek;
		assert(a != b);
	}

	// 2) Degree bounds: deg(v) <= target_degree
	for (PartId v : cc.verts) {
		auto it = adj.find(v);
		if (it == adj.end())
			continue;
		std::size_t deg = it->second.size();
		assert(deg <= target_degree);
	}
}

// -----------------------------------------------------------------------------
// Test 5: larger CC invariants + basic sanity
// -----------------------------------------------------------------------------

static void test_large_cc_invariants() {
	TestGraph::ComponentView cc;
	const std::size_t n = 100;
	cc.verts.reserve(n);
	for (std::size_t i = 0; i < n; ++i) {
		cc.verts.push_back(PartId(i));
	}

	const std::size_t target_degree = 4;
	RamanujanPolicy<TestGraph> policy(target_degree);

	std::unordered_set<UnorderedPair<PartId>> edge_set;
	std::vector<UnorderedPair<PartId>> edges;

	for (auto ek : policy.compute(cc)) {
		// no duplicates
		assert(edge_set.insert(ek).second);
		edges.push_back(ek);
	}

	auto adj = build_adjacency(edges);

	// 1) No self-loops
	for (const auto &ek : edges) {
		auto [a, b] = ek;
		assert(a != b);
	}

	// 2) Degree bounds
	for (PartId v : cc.verts) {
		auto it = adj.find(v);
		if (it == adj.end())
			continue;
		std::size_t deg = it->second.size();
		assert(deg <= target_degree);
	}

	// 3) At least some edges exist for n=100, degree=4
	assert(!edges.empty());
}

// -----------------------------------------------------------------------------
// Test 6: determinism: same CC + same degree/seed => identical edge set
// -----------------------------------------------------------------------------

static void test_determinism() {
	TestGraph::ComponentView cc;
	for (std::size_t i = 0; i < 32; ++i) {
		cc.verts.push_back(PartId(100 + i));
	}

	const std::size_t target_degree = 4;
	const std::uint64_t seed = 0x123456789abcdef0ull;

	RamanujanPolicy<TestGraph> policy1(target_degree, seed);
	RamanujanPolicy<TestGraph> policy2(target_degree, seed);

	std::unordered_set<UnorderedPair<PartId>> set1;
	std::unordered_set<UnorderedPair<PartId>> set2;

	for (auto ek : policy1.compute(cc)) {
		set1.insert(ek);
	}
	for (auto ek : policy2.compute(cc)) {
		set2.insert(ek);
	}

	assert(set1 == set2);
}

// -----------------------------------------------------------------------------
// main
// -----------------------------------------------------------------------------

int main() {
	test_empty_component();
	test_single_vertex_component();
	test_zero_degree();
	test_small_cc_invariants();
	test_large_cc_invariants();
	test_determinism();
	return 0;
}
