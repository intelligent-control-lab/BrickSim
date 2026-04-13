import std;
import bricksim.core.graph;
import bricksim.core.specs;
import bricksim.core.connections;
import bricksim.utils.transforms;
import bricksim.vendor;

#include <cassert>

using namespace bricksim;

namespace {

// Helpers to build interface specs quickly
static Transformd Ixf() {
	return SE3d{}.identity();
}

static InterfaceSpec mk_stud(InterfaceId id, BrickUnit L, BrickUnit W,
                             Transformd pose = Ixf()) {
	return InterfaceSpec{
	    .id = id, .type = InterfaceType::Stud, .L = L, .W = W, .pose = pose};
}
static InterfaceSpec mk_hole(InterfaceId id, BrickUnit L, BrickUnit W,
                             Transformd pose = Ixf()) {
	return InterfaceSpec{
	    .id = id, .type = InterfaceType::Hole, .L = L, .W = W, .pose = pose};
}

using G = LegoGraph<PartList<CustomPart>>; // single-type graph (CustomPart)

// Build a tiny graph with two parts (pid 0 and 1) and optional third isolated
static void build_three_parts(G &g) {
	// Part 0: has one Stud(10) and one Hole(20)
	auto ifs0 = std::initializer_list<InterfaceSpec>{
	    mk_stud(10, 2, 2), mk_stud(12, 2, 2), mk_hole(20, 2, 2),
	    mk_hole(22, 2, 2)};
	[[maybe_unused]] auto ok0 =
	    g.add_part<CustomPart>(0.1, BrickColor{255, 0, 0}, ifs0);
	assert(ok0);

	// Part 1: also one Stud(11) and one Hole(21)
	auto ifs1 = std::initializer_list<InterfaceSpec>{
	    mk_stud(11, 2, 2), mk_stud(13, 2, 2), mk_hole(21, 2, 2),
	    mk_hole(23, 2, 2)};
	[[maybe_unused]] auto ok1 =
	    g.add_part<CustomPart>(0.2, BrickColor{0, 255, 0}, ifs1);
	assert(ok1);

	// Part 2: isolated; only a stud(12)
	auto ifs2 = std::initializer_list<InterfaceSpec>{mk_stud(30, 1, 1),
	                                                 mk_hole(31, 1, 1)};
	[[maybe_unused]] auto ok2 =
	    g.add_part<CustomPart>(0.3, BrickColor{0, 0, 255}, ifs2);
	assert(ok2);
}

// Convenience aliases for readable InterfaceRefs
static InterfaceRef IR(PartId pid, InterfaceId iid) {
	return {pid, iid};
}

// Collect connected components as sorted PartId sets for easy comparison
static std::vector<std::vector<PartId>> collect_components(const G &g) {
	std::vector<std::vector<PartId>> comps;
	for (auto comp : g.components()) {
		std::vector<PartId> verts;
		for (PartId pid : comp.vertices()) {
			verts.push_back(pid);
		}
		std::sort(verts.begin(), verts.end());
		if (!verts.empty()) {
			assert(comp.size() == verts.size());
		}
		comps.push_back(std::move(verts));
	}
	std::sort(comps.begin(), comps.end());
	return comps;
}

// --------------------------- tests ---------------------------

static void test_find_interface_spec_and_lookup_visit() {
	G g; // default resource is fine here
	build_three_parts(g);

	// find_interface_spec: existing and missing
	auto s0 = g.find_interface_spec(IR(0, 10));
	auto h0 = g.find_interface_spec(IR(0, 20));
	auto missing = g.find_interface_spec(IR(0, 999));
	assert(s0 && s0->type == InterfaceType::Stud);
	assert(h0 && h0->type == InterfaceType::Hole);
	assert(!missing);

	// part_path: same vertex yields an empty sequence
	int steps_same = 0;
	for (auto [a, b] : g.part_path<PartId>(0, 0)) {
		(void)a;
		(void)b;
		++steps_same;
	}
	assert(steps_same == 0);

	// part_path: missing part id throws
	bool ex_caught = false;
	int steps_missing = 0;
	try {
		for (auto [a, b] : g.part_path<PartId>(PartId{123456u}, PartId{0})) {
			(void)a;
			(void)b;
			++steps_missing;
		}
	} catch (const std::out_of_range &) {
		ex_caught = true;
	}
	assert(ex_caught);
	assert(steps_missing == 0);

	// lookup_transform: identity for u==v; nullopt for unconnected pair
	auto T00 = g.lookup_transform<PartId>(0, 0);
	assert(T00.has_value() && SE3d{}.almost_equal(*T00, SE3d{}.identity()));
	auto T01 = g.lookup_transform<PartId>(0, 1);
	assert(!T01.has_value());
}

static void test_part_bfs_invalid_and_isolated() {
	G g;
	build_three_parts(g); // three isolated parts; no connections yet

	// Helper to collect BFS traversal from a given start part id
	auto collect_bfs = [&](PartId start) {
		std::vector<PartId> order;
		std::unordered_map<PartId, Transformd> transforms;
		for (auto [pid, T] : g.component_view(start).transforms()) {
			order.push_back(pid);
			// part_bfs must not visit a part twice
			assert(!transforms.contains(pid));
			transforms.emplace(pid, T);
		}
		return std::pair{std::move(order), std::move(transforms)};
	};

	// Starting from an existing but isolated part: only itself is visited
	{
		auto [order0, T0] = collect_bfs(PartId{0});
		assert(order0.size() == 1);
		assert(order0[0] == PartId{0});
		auto it = T0.find(PartId{0});
		assert(it != T0.end());
		assert(SE3d{}.almost_equal(it->second, SE3d{}.identity()));
	}

	{
		auto [order2, T2] = collect_bfs(PartId{2});
		assert(order2.size() == 1);
		assert(order2[0] == PartId{2});
		auto it = T2.find(PartId{2});
		assert(it != T2.end());
		assert(SE3d{}.almost_equal(it->second, SE3d{}.identity()));
	}

	// Starting from a non-existent / dead part id: throws
	bool ex_caught = false;
	std::size_t count_invalid = 0;
	try {
		for (auto [pid, T] : g.component_view(PartId{9999}).transforms()) {
			(void)pid;
			(void)T;
			++count_invalid;
		}
	} catch (const std::out_of_range &) {
		ex_caught = true;
	}
	assert(ex_caught);
	assert(count_invalid == 0);
}

static void test_part_bfs_matches_lookup_transform() {
	G g;
	build_three_parts(g); // parts 0,1,2

	// Build a simple chain 0-1-2 with non-trivial transforms on each edge.
	ConnectionSegment cs01{};
	cs01.offset = Eigen::Vector2i{1, 0}; // translate along +x
	ConnectionSegment cs12{};
	cs12.offset =
	    Eigen::Vector2i{0, 1}; // translate along +y with valid overlap

	assert(g.connect(IR(0, 10), IR(1, 21), cs01));
	assert(g.connect(IR(1, 11), IR(2, 31), cs12));

	auto check_from = [&](PartId start) {
		std::vector<PartId> order;
		std::unordered_map<PartId, Transformd> seen;
		for (auto [pid, T] : g.component_view(start).transforms()) {
			// BFS should not revisit nodes
			assert(!seen.contains(pid));
			seen.emplace(pid, T);
			order.push_back(pid);
		}

		// We built a connected chain of three parts
		assert(seen.size() == 3);
		assert(order.size() == 3);
		assert(order.front() == start);

		// Start node must have identity transform
		auto it_start = seen.find(start);
		assert(it_start != seen.end());
		assert(SE3d{}.almost_equal(it_start->second, SE3d{}.identity()));

		// For every reachable part, BFS-reported transform must match lookup_transform
		for (PartId pid : {PartId{0}, PartId{1}, PartId{2}}) {
			auto it = seen.find(pid);
			assert(it != seen.end());
			auto T_expected = g.lookup_transform<PartId>(start, pid);
			assert(T_expected.has_value());
			assert(SE3d{}.almost_equal(it->second, *T_expected));
		}
	};

	check_from(PartId{0});
	check_from(PartId{1});
	check_from(PartId{2});
}

// components(): empty graph and graph with isolated parts
static void test_components_empty_and_singletons() {
	{
		G g;
		auto comps = collect_components(g);
		assert(comps.empty());
	}

	{
		G g;
		build_three_parts(g); // three isolated parts 0,1,2
		auto comps = collect_components(g);

		std::vector<std::vector<PartId>> expected{
		    {PartId{0}}, {PartId{1}}, {PartId{2}}};
		std::sort(expected.begin(), expected.end());

		assert(comps == expected);
	}
}

// components(): connectivity after adding edges between parts
static void test_components_with_connections() {
	G g;
	build_three_parts(g); // 0,1,2

	ConnectionSegment cs{}; // identity transform
	assert(g.connect(IR(0, 10), IR(1, 21), cs));

	auto comps = collect_components(g);

	std::vector<std::vector<PartId>> expected{{PartId{0}, PartId{1}},
	                                          {PartId{2}}};
	for (auto &v : expected) {
		std::sort(v.begin(), v.end());
	}
	std::sort(expected.begin(), expected.end());

	assert(comps == expected);
}

// components(): stay consistent after removing parts
static void test_components_after_removals() {
	G g;
	build_three_parts(g); // 0,1,2

	ConnectionSegment cs{}; // identity transform on each edge
	assert(g.connect(IR(0, 10), IR(1, 21), cs)); // 0-1
	assert(g.connect(IR(1, 11), IR(2, 31), cs)); // 1-2

	// All three parts should be in one component
	{
		auto comps = collect_components(g);
		std::vector<std::vector<PartId>> expected{
		    {PartId{0}, PartId{1}, PartId{2}}};
		for (auto &v : expected) {
			std::sort(v.begin(), v.end());
		}
		std::sort(expected.begin(), expected.end());
		assert(comps == expected);
	}

	// Remove middle part; remaining parts must become separate components
	assert(g.remove_part(PartId{1}));

	{
		auto comps = collect_components(g);
		std::vector<std::vector<PartId>> expected{{PartId{0}}, {PartId{2}}};
		std::sort(expected.begin(), expected.end());
		assert(comps == expected);
	}
}

static void test_connect_branches_and_bundle() {
	G g;
	build_three_parts(g);

	// Wrong types: stud/hole reversed → false
	ConnectionSegment cs0; // default offset(0,0), yaw=0
	auto r_wrong = g.connect(IR(0, 20), IR(1, 11), cs0);
	assert(!r_wrong);

	// Missing iface on either side → false
	auto r_miss1 = g.connect(IR(0, 999), IR(1, 21), cs0);
	auto r_miss2 = g.connect(IR(0, 10), IR(1, 999), cs0);
	assert(!r_miss1 && !r_miss2);

	// First valid connect (stud 0:10 -> hole 1:21). Bundle does not exist yet.
	auto r1 = g.connect(IR(0, 10), IR(1, 21), cs0);
	assert(r1);

	// Duplicate same segment (same (stud,hole)) → guarded by conn_segs_.contains → false
	auto r_dup_seg = g.connect(IR(0, 10), IR(1, 21), cs0);
	assert(!r_dup_seg);

	// Bundle exists now for endpoint {0,1}; attempt another segment between
	// the same endpoint but different interface pair and different transform.
	ConnectionSegment cs_diff;
	cs_diff.offset = Eigen::Vector2i{1, 0}; // different from default (0,0)
	auto r_diff_T = g.connect(IR(0, 12), IR(1, 23), cs_diff);
	assert(!r_diff_T); // rejected by bundle-exists with mismatched transform

	// Validate connection_segments count and bundle contents
	assert(g.connection_segments().size() == 1);
	assert(g.connection_bundles().size() == 1);

	// The single bundle should carry csid 0 and T_a_b/T_b_a as identity/inverse
	const auto &bundles = g.connection_bundles();
	ConnectionEndpoint ep{0u, 1u};
	auto it = bundles.find(ep);
	assert(it != bundles.end());
	const ConnectionBundle &bundle = it->second.wrapped();
	assert(bundle.conn_seg_ids.size() == 1 && bundle.conn_seg_ids.contains(0));
	// With cs0 == identity transform (poses identity, offset 0)
	Transformd T = SE3d{}.identity();
	assert(SE3d{}.almost_equal(bundle.T_a_b, T) ||
	       SE3d{}.almost_equal(bundle.T_b_a, T));
	Transformd Tinvt = inverse(T);
	assert(SE3d{}.almost_equal(bundle.T_a_b, Tinvt) ||
	       SE3d{}.almost_equal(bundle.T_b_a, Tinvt));

	// Part adjacency updated
	using PW = SimplePartWrapper<CustomPart>;
	const PW *p0 = g.parts().find_value<PW>(PartId{0});
	const PW *p1 = g.parts().find_value<PW>(PartId{1});
	assert(p0 && p1);
	assert(p0->outgoings().contains(ConnSegId{0}));
	assert(p1->incomings().contains(ConnSegId{0}));
	assert(p0->neighbor_parts().contains(PartId{1}));
	assert(p1->neighbor_parts().contains(PartId{0}));
}

// Validate reported sizes/connectedness around connect() and guard-rails for
// inexistent parts and interfaces.
static void test_connect_inputs_and_status() {
	G g;
	build_three_parts(g); // ids 0,1,2

	auto dg_conn = [&](PartId a, PartId b) {
		const auto *a_dg = g.parts().find_key<DgVertexId>(a);
		const auto *b_dg = g.parts().find_key<DgVertexId>(b);
		assert(a_dg && b_dg);
		return g.dynamic_graph().connected(a_dg->value(), b_dg->value());
	};

	// Baseline status
	assert(g.parts().size() == 3);
	assert(g.connection_segments().size() == 0);
	assert(g.connection_bundles().size() == 0);
	assert(!dg_conn(0, 1));

	ConnectionSegment cs{}; // default offset(0,0), yaw=0

	// Inexistent part id on stud side
	assert(!g.connect(IR(9999, 10), IR(1, 21), cs));
	// Inexistent part id on hole side
	assert(!g.connect(IR(0, 10), IR(9999, 21), cs));
	// Existing parts but non-existent interface ids
	assert(!g.connect(IR(0, 999), IR(1, 21), cs));
	assert(!g.connect(IR(0, 10), IR(1, 999), cs));

	// Status unchanged
	assert(g.connection_segments().size() == 0);
	assert(g.connection_bundles().size() == 0);
	assert(!dg_conn(0, 1));

	// Valid connection updates stores; dynamic_graph should reflect connectivity
	assert(g.connect(IR(0, 10), IR(1, 21), cs));
	assert(g.connection_segments().size() == 1);
	assert(g.connection_bundles().size() == 1);
	assert(dg_conn(0, 1));

	// Duplicate (existing) connection must fail
	assert(!g.connect(IR(0, 10), IR(1, 21), cs));

	// Disconnect non-existent connections (by id and by ref)
	assert(!g.disconnect(ConnSegId{999999}));
	ConnSegRef nonref{IR(0, 12), IR(1, 23)}; // was never connected in this test
	assert(!g.disconnect(nonref));
}

static void test_zero_overlap_cannot_connect() {
	G g;
	build_three_parts(g); // 0,1,2

	ConnectionSegment cs{};
	cs.offset =
	    Eigen::Vector2i{0, 2}; // touches boundary only; zero overlap in y

	assert(!g.connect(IR(1, 11), IR(2, 31), cs));
	assert(g.connection_segments().size() == 0);
	assert(g.connection_bundles().size() == 0);
}

// Multiple connections between the same two parts through different
// studs/holes: matching transforms could co-exist if the implementation
// maintains a consistent relative pose; mismatched must be rejected.
static void test_multi_connections_match_and_mismatch() {
	G g;
	// Parts 0 and 1 with two studs/holes each (from build_three_parts)
	build_three_parts(g);

	ConnectionSegment cs0{};         // offset (0,0)
	ConnectionSegment cs_mismatch{}; // offset (1,0) to alter transform
	cs_mismatch.offset = Eigen::Vector2i{1, 0};

	// First connect succeeds and establishes the bundle transform A->B
	assert(g.connect(IR(0, 10), IR(1, 21), cs0));
	// Duplicate identical connection should fail
	assert(!g.connect(IR(0, 10), IR(1, 21), cs0));
	assert(g.connection_segments().size() == 1);
	assert(g.connection_bundles().size() == 1);
	// DG shows connectivity
	auto dg_conn = [&](PartId a, PartId b) {
		const auto *a_dg = g.parts().find_key<DgVertexId>(a);
		const auto *b_dg = g.parts().find_key<DgVertexId>(b);
		assert(a_dg && b_dg);
		return g.dynamic_graph().connected(a_dg->value(), b_dg->value());
	};
	(void)dg_conn(0, 1);

	// Mismatched transform on a different s/h pair for the same endpoint must be rejected
	assert(!g.connect(IR(0, 12), IR(1, 23), cs_mismatch));

	// Disconnect the existing segment; now bundle removed and DG edge cleared
	ConnSegRef csref1{IR(0, 10), IR(1, 21)};
	assert(g.disconnect(csref1));
	assert(g.connection_segments().size() == 0);
	assert(g.connection_bundles().size() == 0);
	assert(!dg_conn(0, 1));
}

static void test_remove_part_nonexistent_and_existent() {
	G g;
	build_three_parts(g);

	// Remove non-existent part -> false; stores unchanged
	std::size_t parts_before = g.parts().size();
	assert(!g.remove_part(PartId{9999}));
	assert(g.parts().size() == parts_before);

	// Connect then remove an existent part and verify cleanup
	ConnectionSegment cs{};
	assert(g.connect(IR(0, 10), IR(1, 21), cs));
	assert(g.connection_segments().size() == 1);
	assert(g.connection_bundles().size() == 1);
	assert(g.remove_part(PartId{0}));
	assert(g.parts().size() == parts_before - 1);
	assert(g.connection_segments().size() == 0);
	assert(g.connection_bundles().size() == 0);
}

// Connect A-B and B-C, then delete B; verify dynamic_graph connectivity
// between A and C after deletion is false.
static void test_chain_connect_then_remove_middle_vertex() {
	G g;
	build_three_parts(g); // 0:A, 1:B, 2:C

	auto dg_conn = [&](PartId a, PartId b) {
		const auto *a_dg = g.parts().find_key<DgVertexId>(a);
		const auto *b_dg = g.parts().find_key<DgVertexId>(b);
		assert(a_dg && b_dg);
		return g.dynamic_graph().connected(a_dg->value(), b_dg->value());
	};

	ConnectionSegment cs{};                      // identity transform
	assert(g.connect(IR(0, 10), IR(1, 21), cs)); // A->B
	assert(g.connect(IR(1, 11), IR(2, 31), cs)); // B->C

	// Now A and C must be connected in DG through B
	assert(dg_conn(0, 2));

	// Remove middle vertex (B)
	assert(g.remove_part(PartId{1}));

	// A and C must not be connected afterwards in DG
	assert(!dg_conn(0, 2));
}

// A–B, B–C, then A–C: consistent vs inconsistent transforms
static void test_triangle_consistency_inconsistent() {
	G g;
	build_three_parts(g);    // 0:A with studs 10,12 and holes 20,22; 1:B; 2:C
	ConnectionSegment cs0{}; // identity transform
	ConnectionSegment cs_bad{}; // mismatch
	cs_bad.offset = Eigen::Vector2i{1, 0};

	auto dg_conn = [&](PartId a, PartId b) {
		const auto *a_dg = g.parts().find_key<DgVertexId>(a);
		const auto *b_dg = g.parts().find_key<DgVertexId>(b);
		assert(a_dg && b_dg);
		return g.dynamic_graph().connected(a_dg->value(), b_dg->value());
	};

	// A–B and B–C establish a DG path A↔C
	assert(g.connect(IR(0, 10), IR(1, 21), cs0));
	assert(g.connect(IR(1, 11), IR(2, 31), cs0));
	assert(dg_conn(0, 2));

	// A–C consistent with path (identity) should succeed
	assert(g.connect(IR(0, 12), IR(2, 31), cs0));
	assert(g.connection_bundles().size() >= 3); // A-B, B-C, A-C

	// A–C inconsistent transform should be rejected; if not, disconnect immediately
	assert(!g.connect(IR(0, 12), IR(2, 31), cs_bad));

	// Cleanup: disconnect A–C and verify DG A–C remains via A–B–C
	ConnSegRef ac_ref{IR(0, 12), IR(2, 31)};
	assert(g.disconnect(ac_ref));
	assert(dg_conn(0, 2));
}

static void test_remove_part_variants() {
	G g;
	build_three_parts(g); // parts 0,1,2

	// remove_part: missing key → false
	assert(!g.remove_part(PartId{999}));

	// Connect 0 -> 1 then remove 0; ensures all adjacency and bundles cleaned
	ConnectionSegment cs0; // identity transform
	[[maybe_unused]] auto ok = g.connect(IR(0, 10), IR(1, 21), cs0);
	assert(ok);
	assert(g.connection_segments().size() == 1);
	assert(g.connection_bundles().size() == 1);

	// Remove an isolated part first (pid 2) — touches empty loops
	assert(g.remove_part(PartId{2}));
	assert(g.parts().size() == 2);

	// Now remove pid 0 which has one outgoing and a neighbor
	assert(g.remove_part(PartId{0}));
	assert(g.parts().size() == 1);
	assert(g.connection_segments().size() == 0);
	assert(g.connection_bundles().size() == 0);

	// Remaining part (pid 1) must have empty adjacency sets
	using PW = SimplePartWrapper<CustomPart>;
	const PW *p1 = g.parts().find_value<PW>(PartId{1});
	assert(p1);
	assert(p1->incomings().size() == 0);
	assert(p1->outgoings().size() == 0);
	assert(p1->neighbor_parts().size() == 0);

	// Rebuild on a fresh instance and connect again; now remove the HOLE side (pid 1)
	G g2;
	build_three_parts(g2);
	ok = g2.connect(IR(0, 10), IR(1, 21), cs0);
	assert(ok);
	assert(g2.remove_part(PartId{1}));
	// After removing the hole side, the stud side's outgoings cleaned
	using PW2 = SimplePartWrapper<CustomPart>;
	const PW2 *p0 = g2.parts().find_value<PW2>(PartId{0});
	assert(p0);
	assert(p0->outgoings().size() == 0);
}

} // namespace

int main() {
	test_find_interface_spec_and_lookup_visit();
	test_part_bfs_invalid_and_isolated();
	test_part_bfs_matches_lookup_transform();
	test_components_empty_and_singletons();
	test_components_with_connections();
	test_components_after_removals();
	test_connect_branches_and_bundle();
	test_connect_inputs_and_status();
	test_zero_overlap_cannot_connect();
	test_multi_connections_match_and_mismatch();
	test_remove_part_nonexistent_and_existent();
	test_remove_part_variants();
	test_chain_connect_then_remove_middle_vertex();
	test_triangle_consistency_inconsistent();
	return 0;
}
