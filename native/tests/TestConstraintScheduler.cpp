import std;
import lego_assemble.core.graph;
import lego_assemble.core.specs;
import lego_assemble.core.connections;
import lego_assemble.utils.transforms;
import lego_assemble.utils.unordered_pair;
import lego_assemble.physx.constraint_scheduler;
import lego_assemble.vendor;

#include <cassert>

using namespace lego_assemble;

namespace {

// ----------------- Common helpers -----------------

using G = LegoGraph<PartList<CustomPart>>;

struct ConstraintEvents {
	struct EdgeRecord {
		PartId a{};
		PartId b{};
		Transformd T_a_b{};
		std::size_t handle{};
	};

	std::vector<EdgeRecord> created;
	std::vector<std::size_t> destroyed;
	std::unordered_map<std::size_t, EdgeRecord> live_by_handle;
	std::unordered_map<UnorderedPair<PartId>, std::size_t> live_by_edge;
	std::size_t next_handle{1};

	std::size_t create(PartId a, PartId b, const Transformd &T_a_b) {
		UnorderedPair<PartId> ek{a, b};
		assert(!live_by_edge.contains(ek));
		std::size_t h = next_handle++;
		EdgeRecord rec{a, b, T_a_b, h};
		created.push_back(rec);
		live_by_handle.emplace(h, rec);
		live_by_edge.emplace(ek, h);
		return h;
	}

	void destroy(std::size_t h) {
		destroyed.push_back(h);
		auto it = live_by_handle.find(h);
		if (it != live_by_handle.end()) {
			UnorderedPair<PartId> ek{it->second.a, it->second.b};
			live_by_edge.erase(ek);
			live_by_handle.erase(it);
		}
	}

	bool has_edge(PartId a, PartId b) const {
		return live_by_edge.contains(UnorderedPair<PartId>{a, b});
	}
};

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

// Same layout as in TestLegoGraph.cpp: three parts with multiple interfaces.
static void build_three_parts(G &g) {
	auto ifs0 = std::initializer_list<InterfaceSpec>{
	    mk_stud(10, 2, 2), mk_stud(12, 2, 2), mk_hole(20, 2, 2),
	    mk_hole(22, 2, 2)};
	auto p0 = g.add_part<CustomPart>(
	    std::tuple<>{}, 0.1, BrickColor{255, 0, 0}, ifs0);
	assert(p0 && *p0 == PartId{0});

	auto ifs1 = std::initializer_list<InterfaceSpec>{
	    mk_stud(11, 2, 2), mk_stud(13, 2, 2), mk_hole(21, 2, 2),
	    mk_hole(23, 2, 2)};
	auto p1 = g.add_part<CustomPart>(
	    std::tuple<>{}, 0.2, BrickColor{0, 255, 0}, ifs1);
	assert(p1 && *p1 == PartId{1});

	auto ifs2 = std::initializer_list<InterfaceSpec>{mk_stud(30, 1, 1),
	                                                 mk_hole(31, 1, 1)};
	auto p2 = g.add_part<CustomPart>(
	    std::tuple<>{}, 0.3, BrickColor{0, 0, 255}, ifs2);
	assert(p2 && *p2 == PartId{2});
}

static InterfaceRef IR(PartId pid, InterfaceId iid) {
	return {pid, iid};
}

template <class Strategy>
using Scheduler =
    ConstraintScheduler<G, Strategy,
                        std::function<std::size_t(PartId, PartId,
                                                  const Transformd &)>,
                        std::function<void(std::size_t)>, std::size_t>;

// ----------------- Constructor / destructor -----------------

static void test_scheduler_dtor_no_constraints() {
	G g;
	build_three_parts(g);

	ConstraintEvents events;
	using Strat = TreeOnlySchedulingPolicy<G>;
	{
		auto create_edge = [&events](PartId a, PartId b,
		                             const Transformd &T_a_b) {
			(void)a;
			(void)b;
			(void)T_a_b;
			return events.create(a, b, T_a_b);
		};
		auto destroy_edge = [&events](std::size_t h) { events.destroy(h); };

		Strat strat{};
		Scheduler<Strat> sched{
		    &g, strat, create_edge, destroy_edge,
		    std::pmr::get_default_resource()};

		// No events fired → no constraints allocated.
		assert(events.created.empty());
		assert(events.destroyed.empty());
	}
	// Destructor must not destroy anything.
	assert(events.destroyed.empty());
}

static void test_scheduler_dtor_with_constraints() {
	G g;
	build_three_parts(g);

	// Build 0-1-2 chain in the graph.
	ConnectionSegment cs{};
	assert(g.connect(IR(0, 10), IR(1, 21), std::tuple<>{}, cs));
	assert(g.connect(IR(1, 11), IR(2, 31), std::tuple<>{}, cs));

	ConstraintEvents events;
	using Strat = TreeOnlySchedulingPolicy<G>;
	{
		auto create_edge = [&events](PartId a, PartId b,
		                             const Transformd &T_a_b) {
			return events.create(a, b, T_a_b);
		};
		auto destroy_edge = [&events](std::size_t h) { events.destroy(h); };

		Strat strat{};
		Scheduler<Strat> sched{
		    &g, strat, create_edge, destroy_edge,
		    std::pmr::get_default_resource()};

		// Mark connection once; scheduler should create two tree edges {0,1},{1,2}
		sched.on_connected(0, 1);
		assert(events.created.size() == 2);
		assert(events.destroyed.empty());
		assert(events.has_edge(0, 1));
		assert(events.has_edge(1, 2));
	}
	// Destructor should destroy both handles exactly once.
	assert(events.destroyed.size() == 2);
	// All live edges must have been cleared.
	assert(events.live_by_edge.empty());
}

// ----------------- ChangeBlock / batching -----------------

static void test_single_change_block_batches_flush() {
	G g;
	build_three_parts(g);

	ConnectionSegment cs{};
	assert(g.connect(IR(0, 10), IR(1, 21), std::tuple<>{}, cs));
	assert(g.connect(IR(1, 11), IR(2, 31), std::tuple<>{}, cs));

	ConstraintEvents events;
	using Strat = FullGraphSchedulingPolicy<G>;

	auto create_edge = [&events](PartId a, PartId b,
	                             const Transformd &T_a_b) {
		return events.create(a, b, T_a_b);
	};
	auto destroy_edge = [&events](std::size_t h) { events.destroy(h); };

	Strat strat{};
	Scheduler<Strat> sched{
	    &g, strat, create_edge, destroy_edge,
	    std::pmr::get_default_resource()};

	{
		auto block = sched.change_block();
		// Connect events inside the block should not flush immediately.
		sched.on_connected(0, 1);
		sched.on_connected(1, 2);
		assert(events.created.empty());
	}
	// After outer block destruction, a single flush should have created all
	// constraints for the final topology (a 0-1-2 chain).
	assert(events.created.size() == 2);
	assert(events.has_edge(0, 1));
	assert(events.has_edge(1, 2));
	assert(events.destroyed.empty());
}

static void test_nested_change_blocks_flush_once() {
	G g;
	build_three_parts(g);

	ConnectionSegment cs{};
	assert(g.connect(IR(0, 10), IR(1, 21), std::tuple<>{}, cs));
	assert(g.connect(IR(1, 11), IR(2, 31), std::tuple<>{}, cs));

	ConstraintEvents events;
	using Strat = TreeOnlySchedulingPolicy<G>;

	auto create_edge = [&events](PartId a, PartId b,
	                             const Transformd &T_a_b) {
		return events.create(a, b, T_a_b);
	};
	auto destroy_edge = [&events](std::size_t h) { events.destroy(h); };

	Strat strat{};
	Scheduler<Strat> sched{
	    &g, strat, create_edge, destroy_edge,
	    std::pmr::get_default_resource()};

	{
		auto outer = sched.change_block();
		sched.on_connected(0, 1);
		assert(events.created.empty());
		{
			auto inner = sched.change_block();
			sched.on_connected(1, 2);
			assert(events.created.empty());
		}
		// inner destroyed, but depth>0 so still no flush
		assert(events.created.empty());
	}
	// Only the outermost destruction should have triggered flush.
	assert(events.created.size() == 2); // tree edges {0-1,1-2}
	assert(events.destroyed.empty());
}

static void test_change_block_moved_once() {
	G g;
	build_three_parts(g);

	ConnectionSegment cs{};
	assert(g.connect(IR(0, 10), IR(1, 21), std::tuple<>{}, cs));

	ConstraintEvents events;
	using Strat = TreeOnlySchedulingPolicy<G>;

	auto create_edge = [&events](PartId a, PartId b,
	                             const Transformd &T_a_b) {
		return events.create(a, b, T_a_b);
	};
	auto destroy_edge = [&events](std::size_t h) { events.destroy(h); };

	Strat strat{};
	Scheduler<Strat> sched{
	    &g, strat, create_edge, destroy_edge,
	    std::pmr::get_default_resource()};

	{
		auto block1 = sched.change_block();
		// Move into block2; block1's destructor must be a no-op.
		auto block2 = std::move(block1);
		sched.on_connected(0, 1);
		assert(events.created.empty());
		// block2 goes out of scope here; flush should run exactly once.
	}
	assert(events.created.size() == 1);
	assert(events.has_edge(0, 1));
}

// ----------------- on_part_added -----------------

static void test_on_part_added_outside_block() {
	G g;
	build_three_parts(g);

	ConstraintEvents events;
	using Strat = TreeOnlySchedulingPolicy<G>;

	auto create_edge = [&events](PartId a, PartId b,
	                             const Transformd &T_a_b) {
		return events.create(a, b, T_a_b);
	};
	auto destroy_edge = [&events](std::size_t h) { events.destroy(h); };

	Strat strat{};
	Scheduler<Strat> sched{
	    &g, strat, create_edge, destroy_edge,
	    std::pmr::get_default_resource()};

	// Add an extra isolated part (pid 3)
	auto ifs3 = std::initializer_list<InterfaceSpec>{mk_stud(40, 1, 1),
	                                                 mk_hole(41, 1, 1)};
	auto p3 = g.add_part<CustomPart>(
	    std::tuple<>{}, 0.4, BrickColor{10, 20, 30}, ifs3);
	assert(p3 && *p3 == PartId{3});

	sched.on_part_added(*p3);
	// Single-vertex CC → strategy emits no edges.
	assert(events.created.empty());
	assert(events.destroyed.empty());
}

static void test_on_part_added_inside_block() {
	G g;
	build_three_parts(g);

	ConstraintEvents events;
	using Strat = TreeOnlySchedulingPolicy<G>;

	auto create_edge = [&events](PartId a, PartId b,
	                             const Transformd &T_a_b) {
		return events.create(a, b, T_a_b);
	};
	auto destroy_edge = [&events](std::size_t h) { events.destroy(h); };

	Strat strat{};
	Scheduler<Strat> sched{
	    &g, strat, create_edge, destroy_edge,
	    std::pmr::get_default_resource()};

	auto ifs3 = std::initializer_list<InterfaceSpec>{mk_stud(40, 1, 1),
	                                                 mk_hole(41, 1, 1)};
	auto p3 = g.add_part<CustomPart>(
	    std::tuple<>{}, 0.4, BrickColor{10, 20, 30}, ifs3);
	assert(p3 && *p3 == PartId{3});

	{
		auto block = sched.change_block();
		sched.on_part_added(*p3);
		assert(events.created.empty());
	}
	// After block flush, still no constraints because CC has size 1.
	assert(events.created.empty());
	assert(events.destroyed.empty());
}

// ----------------- on_connected / on_disconnected -----------------

static void test_connect_two_single_parts() {
	G g;
	build_three_parts(g);

	ConnectionSegment cs{};
	assert(g.connect(IR(0, 10), IR(1, 21), std::tuple<>{}, cs));

	ConstraintEvents events;
	using Strat = TreeOnlySchedulingPolicy<G>;

	auto create_edge = [&events, &g](PartId a, PartId b,
	                                 const Transformd &T_a_b) {
		// Verify transform matches lookup_transform(a,b)
		auto T_expected = g.lookup_transform<PartId>(a, b);
		assert(T_expected.has_value());
		assert(SE3d{}.almost_equal(T_a_b, *T_expected));
		return events.create(a, b, T_a_b);
	};
	auto destroy_edge = [&events](std::size_t h) { events.destroy(h); };

	Strat strat{};
	Scheduler<Strat> sched{
	    &g, strat, create_edge, destroy_edge,
	    std::pmr::get_default_resource()};

	sched.on_connected(0, 1);
	assert(events.created.size() == 1);
	assert(events.has_edge(0, 1));
	assert(events.destroyed.empty());
}

static void test_connect_within_same_cc_tree_only_no_churn() {
	G g;
	build_three_parts(g);

	ConnectionSegment cs{};
	// Build chain 0-1-2
	assert(g.connect(IR(0, 10), IR(1, 21), std::tuple<>{}, cs));
	assert(g.connect(IR(1, 11), IR(2, 31), std::tuple<>{}, cs));

	ConstraintEvents events;
	using Strat = TreeOnlySchedulingPolicy<G>;

	auto create_edge = [&events](PartId a, PartId b,
	                             const Transformd &T_a_b) {
		return events.create(a, b, T_a_b);
	};
	auto destroy_edge = [&events](std::size_t h) { events.destroy(h); };

	Strat strat{};
	Scheduler<Strat> sched{
	    &g, strat, create_edge, destroy_edge,
	    std::pmr::get_default_resource()};

	// Initial scheduling from a single connect
	sched.on_connected(0, 1);
	assert(events.created.size() == 2);
	assert(events.has_edge(0, 1));
	assert(events.has_edge(1, 2));
	assert(events.destroyed.empty());

	// Now add an extra edge inside the same CC (0-2)
	assert(g.connect(IR(0, 12), IR(2, 31), std::tuple<>{}, cs));
	sched.on_connected(0, 2);

	// Tree-only policy should still keep only the spanning tree edges; no new
	// creates or destroys.
	assert(events.created.size() == 2);
	assert(events.destroyed.empty());
	assert(events.has_edge(0, 1));
	assert(events.has_edge(1, 2));
}

static void test_connect_within_same_cc_fullgraph_adds_edge() {
	G g;
	build_three_parts(g);

	ConnectionSegment cs{};
	// Build chain 0-1-2
	assert(g.connect(IR(0, 10), IR(1, 21), std::tuple<>{}, cs));
	assert(g.connect(IR(1, 11), IR(2, 31), std::tuple<>{}, cs));

	ConstraintEvents events;
	using Strat = FullGraphSchedulingPolicy<G>;

	auto create_edge = [&events](PartId a, PartId b,
	                             const Transformd &T_a_b) {
		return events.create(a, b, T_a_b);
	};
	auto destroy_edge = [&events](std::size_t h) { events.destroy(h); };

	Strat strat{};
	Scheduler<Strat> sched{
	    &g, strat, create_edge, destroy_edge,
	    std::pmr::get_default_resource()};

	// Initial scheduling from a single connect
	sched.on_connected(0, 1);
	assert(events.created.size() == 2); // edges {0-1,1-2}
	assert(events.has_edge(0, 1));
	assert(events.has_edge(1, 2));

	// Add extra edge inside same CC (0-2) and reschedule.
	assert(g.connect(IR(0, 12), IR(2, 31), std::tuple<>{}, cs));
	sched.on_connected(0, 2);

	// Full graph policy should add the new constraint edge {0,2}.
	assert(events.created.size() == 3);
	assert(events.has_edge(0, 2));
	assert(events.destroyed.empty());
}

static void test_disconnect_non_bridge_edge_fullgraph() {
	G g;
	build_three_parts(g);

	ConnectionSegment cs{};
	// Build triangle 0-1-2
	assert(g.connect(IR(0, 10), IR(1, 21), std::tuple<>{}, cs)); // 0-1
	assert(g.connect(IR(1, 11), IR(2, 31), std::tuple<>{}, cs)); // 1-2
	assert(g.connect(IR(0, 12), IR(2, 31), std::tuple<>{}, cs)); // 0-2

	ConstraintEvents events;
	using Strat = FullGraphSchedulingPolicy<G>;

	auto create_edge = [&events](PartId a, PartId b,
	                             const Transformd &T_a_b) {
		return events.create(a, b, T_a_b);
	};
	auto destroy_edge = [&events](std::size_t h) { events.destroy(h); };

	Strat strat{};
	Scheduler<Strat> sched{
	    &g, strat, create_edge, destroy_edge,
	    std::pmr::get_default_resource()};

	// Seed scheduling once
	sched.on_connected(0, 1);
	assert(events.created.size() == 3);
	assert(events.has_edge(0, 1));
	assert(events.has_edge(1, 2));
	assert(events.has_edge(0, 2));
	assert(events.destroyed.empty());

	// Remove non-bridge edge 0-2; CC remains {0,1,2}.
	ConnSegRef ref02{IR(0, 12), IR(2, 31)};
	assert(g.disconnect(ref02));
	sched.on_disconnected(0, 2);

	// Only edge {0,2} should be removed.
	assert(events.created.size() == 3);
	assert(events.destroyed.size() == 1);
	assert(!events.has_edge(0, 2));
	assert(events.has_edge(0, 1));
	assert(events.has_edge(1, 2));
}

static void test_disconnect_bridge_edge_splits_cc() {
	G g;
	build_three_parts(g);

	ConnectionSegment cs{};
	// Build chain 0-1-2
	assert(g.connect(IR(0, 10), IR(1, 21), std::tuple<>{}, cs));
	assert(g.connect(IR(1, 11), IR(2, 31), std::tuple<>{}, cs));

	ConstraintEvents events;
	using Strat = TreeOnlySchedulingPolicy<G>;

	auto create_edge = [&events](PartId a, PartId b,
	                             const Transformd &T_a_b) {
		return events.create(a, b, T_a_b);
	};
	auto destroy_edge = [&events](std::size_t h) { events.destroy(h); };

	Strat strat{};
	Scheduler<Strat> sched{
	    &g, strat, create_edge, destroy_edge,
	    std::pmr::get_default_resource()};

	// Seed scheduling
	sched.on_connected(0, 1);
	assert(events.created.size() == 2);
	assert(events.has_edge(0, 1));
	assert(events.has_edge(1, 2));

	// Disconnect bridge edge 1-2: CC splits into {0,1} and {2}.
	ConnSegRef ref12{IR(1, 11), IR(2, 31)};
	assert(g.disconnect(ref12));
	sched.on_disconnected(1, 2);

	// Only edge {1,2} should be removed; {0,1} preserved.
	assert(events.destroyed.size() == 1);
	assert(!events.has_edge(1, 2));
	assert(events.has_edge(0, 1));
}

// ----------------- on_part_removed -----------------

static void test_on_part_removed_no_constraints() {
	G g;
	build_three_parts(g);

	ConstraintEvents events;
	using Strat = TreeOnlySchedulingPolicy<G>;

	auto create_edge = [&events](PartId a, PartId b,
	                             const Transformd &T_a_b) {
		return events.create(a, b, T_a_b);
	};
	auto destroy_edge = [&events](std::size_t h) { events.destroy(h); };

	Strat strat{};
	Scheduler<Strat> sched{
	    &g, strat, create_edge, destroy_edge,
	    std::pmr::get_default_resource()};

	// Remove an isolated part (no constraint edges in scheduler).
	assert(g.remove_part(PartId{2}));
	sched.on_part_removed(PartId{2});
	assert(events.created.empty());
	assert(events.destroyed.empty());
}

static void test_on_part_removed_with_multiple_constraints() {
	G g;
	build_three_parts(g);

	ConnectionSegment cs{};
	// 0-1-2 chain
	assert(g.connect(IR(0, 10), IR(1, 21), std::tuple<>{}, cs));
	assert(g.connect(IR(1, 11), IR(2, 31), std::tuple<>{}, cs));

	ConstraintEvents events;
	using Strat = TreeOnlySchedulingPolicy<G>;

	auto create_edge = [&events](PartId a, PartId b,
	                             const Transformd &T_a_b) {
		return events.create(a, b, T_a_b);
	};
	auto destroy_edge = [&events](std::size_t h) { events.destroy(h); };

	Strat strat{};
	Scheduler<Strat> sched{
	    &g, strat, create_edge, destroy_edge,
	    std::pmr::get_default_resource()};

	// Initial schedule: edges {0-1,1-2}
	sched.on_connected(0, 1);
	assert(events.created.size() == 2);
	assert(events.has_edge(0, 1));
	assert(events.has_edge(1, 2));

	// Remove middle part from graph, then inform scheduler.
	assert(g.remove_part(PartId{1}));
	sched.on_part_removed(PartId{1});

	// Both incident constraint edges should be destroyed.
	assert(events.destroyed.size() == 2);
	assert(events.live_by_edge.empty());
}

// ----------------- flush_dirty_ccs_ behaviour -----------------

struct LoggingStrategy {
	std::shared_ptr<std::vector<std::vector<PartId>>> seen_components;

	explicit LoggingStrategy(
	    std::shared_ptr<std::vector<std::vector<PartId>>> seen)
	    : seen_components(std::move(seen)) {}

	std::generator<UnorderedPair<PartId>>
	compute(const G::ComponentView &cc) const {
		std::vector<PartId> verts;
		for (PartId pid : cc.vertices()) {
			verts.push_back(pid);
		}
		seen_components->push_back(std::move(verts));
		co_return;
	}
};

static void test_multiple_dirty_vertices_same_cc_only_one_compute() {
	G g;
	build_three_parts(g);

	ConnectionSegment cs{};
	// Build chain 0-1-2
	assert(g.connect(IR(0, 10), IR(1, 21), std::tuple<>{}, cs));
	assert(g.connect(IR(1, 11), IR(2, 31), std::tuple<>{}, cs));

	ConstraintEvents events;
	auto seen =
	    std::make_shared<std::vector<std::vector<PartId>>>();
	LoggingStrategy strat{seen};

	auto create_edge = [&events](PartId a, PartId b,
	                             const Transformd &T_a_b) {
		return events.create(a, b, T_a_b);
	};
	auto destroy_edge = [&events](std::size_t h) { events.destroy(h); };

	Scheduler<LoggingStrategy> sched{
	    &g, strat, create_edge, destroy_edge,
	    std::pmr::get_default_resource()};

	{
		auto block = sched.change_block();
		// Mark endpoints of the same CC as dirty.
		sched.on_connected(0, 2);
	}

	// compute() should have been called exactly once for the CC {0,1,2}.
	assert(seen->size() == 1);
	auto verts = seen->front();
	std::sort(verts.begin(), verts.end());
	std::vector<PartId> expected{0, 1, 2};
	assert(verts == expected);
}

static void test_flush_with_no_dirty_vertices_is_noop() {
	G g;
	build_three_parts(g);

	ConnectionSegment cs{};
	assert(g.connect(IR(0, 10), IR(1, 21), std::tuple<>{}, cs));

	ConstraintEvents events;
	using Strat = TreeOnlySchedulingPolicy<G>;

	auto create_edge = [&events](PartId a, PartId b,
	                             const Transformd &T_a_b) {
		return events.create(a, b, T_a_b);
	};
	auto destroy_edge = [&events](std::size_t h) { events.destroy(h); };

	Strat strat{};
	Scheduler<Strat> sched{
	    &g, strat, create_edge, destroy_edge,
	    std::pmr::get_default_resource()};

	// First schedule once so that edges exist.
	sched.on_connected(0, 1);
	assert(events.created.size() == 1);

	// Now call on_part_removed for a part that never had constraints;
	// this will cause a flush with an empty dirty set (no-op).
	sched.on_part_removed(PartId{2});
	assert(events.created.size() == 1);
	assert(events.destroyed.empty());
}

// ----------------- flush_ diff logic -----------------

static void test_no_change_in_desired_edges_no_churn() {
	G g;
	build_three_parts(g);

	ConnectionSegment cs{};
	assert(g.connect(IR(0, 10), IR(1, 21), std::tuple<>{}, cs));

	ConstraintEvents events;
	using Strat = TreeOnlySchedulingPolicy<G>;

	auto create_edge = [&events](PartId a, PartId b,
	                             const Transformd &T_a_b) {
		return events.create(a, b, T_a_b);
	};
	auto destroy_edge = [&events](std::size_t h) { events.destroy(h); };

	Strat strat{};
	Scheduler<Strat> sched{
	    &g, strat, create_edge, destroy_edge,
	    std::pmr::get_default_resource()};

	// First scheduling
	sched.on_connected(0, 1);
	assert(events.created.size() == 1);
	assert(events.destroyed.empty());

	// Mark the same CC dirty again without changing topology.
	sched.on_connected(0, 1);
	// Strategy still desires the same single edge; no churn expected.
	assert(events.created.size() == 1);
	assert(events.destroyed.empty());
}

static void test_add_new_desired_edges_only_create_new() {
	G g;
	build_three_parts(g);

	ConnectionSegment cs{};
	// Initially only connect 0-1
	assert(g.connect(IR(0, 10), IR(1, 21), std::tuple<>{}, cs));

	ConstraintEvents events;
	using Strat = TreeOnlySchedulingPolicy<G>;

	auto create_edge = [&events](PartId a, PartId b,
	                             const Transformd &T_a_b) {
		return events.create(a, b, T_a_b);
	};
	auto destroy_edge = [&events](std::size_t h) { events.destroy(h); };

	Strat strat{};
	Scheduler<Strat> sched{
	    &g, strat, create_edge, destroy_edge,
	    std::pmr::get_default_resource()};

	// First schedule: edge {0,1}
	sched.on_connected(0, 1);
	assert(events.created.size() == 1);
	assert(events.has_edge(0, 1));

	// Extend topology with 1-2; tree-only policy now wants {0,1} and {1,2}.
	assert(g.connect(IR(1, 11), IR(2, 31), std::tuple<>{}, cs));
	sched.on_connected(1, 2);

	// Existing edge kept; new one created; none destroyed.
	assert(events.created.size() == 2);
	assert(events.has_edge(1, 2));
	assert(events.destroyed.empty());
}

struct Only02Strategy {
	std::generator<UnorderedPair<PartId>>
	compute(const G::ComponentView &cc) const {
		bool has0 = false;
		bool has2 = false;
		for (PartId pid : cc.vertices()) {
			if (pid == PartId{0}) {
				has0 = true;
			}
			if (pid == PartId{2}) {
				has2 = true;
			}
		}
		if (has0 && has2) {
			co_yield UnorderedPair<PartId>{PartId{0}, PartId{2}};
		}
		co_return;
	}
};

static void test_cross_cc_edges_removed_after_split() {
	G g;
	build_three_parts(g);

	ConnectionSegment cs{};
	// Connect 0-1-2 chain
	assert(g.connect(IR(0, 10), IR(1, 21), std::tuple<>{}, cs));
	assert(g.connect(IR(1, 11), IR(2, 31), std::tuple<>{}, cs));

	ConstraintEvents events;
	using Strat = Only02Strategy;

	auto create_edge = [&events](PartId a, PartId b,
	                             const Transformd &T_a_b) {
		return events.create(a, b, T_a_b);
	};
	auto destroy_edge = [&events](std::size_t h) { events.destroy(h); };

	Strat strat{};
	Scheduler<Strat> sched{
	    &g, strat, create_edge, destroy_edge,
	    std::pmr::get_default_resource()};

	// Initial constraints: strategy requests edge {0,2} over CC {0,1,2}.
	sched.on_connected(0, 1);
	assert(events.has_edge(0, 2));

	// Now disconnect 1-2 so that topology splits, leaving 0 and 2 in
	// different CCs; cross-CC constraint {0,2} must be removed.
	ConnSegRef ref12{IR(1, 11), IR(2, 31)};
	assert(g.disconnect(ref12));
	sched.on_disconnected(1, 2);

	assert(!events.has_edge(0, 2));
}

// ----------------- Strategy-specific tests -----------------

static void test_tree_only_policy_on_path() {
	G g;
	build_three_parts(g);

	ConnectionSegment cs{};
	assert(g.connect(IR(0, 10), IR(1, 21), std::tuple<>{}, cs));
	assert(g.connect(IR(1, 11), IR(2, 31), std::tuple<>{}, cs));

	auto cc = g.component_view(PartId{1});
	TreeOnlySchedulingPolicy<G> policy{};

	std::vector<UnorderedPair<PartId>> edges;
	for (auto e : policy.compute(cc)) {
		edges.push_back(e);
	}
	std::sort(edges.begin(), edges.end(),
	          [](const auto &x, const auto &y) {
		          return std::tie(x.first, x.second) <
		                 std::tie(y.first, y.second);
	          });

	std::vector<UnorderedPair<PartId>> expected{
	    UnorderedPair<PartId>{0, 1}, UnorderedPair<PartId>{1, 2}};
	std::sort(expected.begin(), expected.end(),
	          [](const auto &x, const auto &y) {
		          return std::tie(x.first, x.second) <
		                 std::tie(y.first, y.second);
	          });
	assert(edges == expected);
}

static void test_fullgraph_policy_on_triangle() {
	G g;
	build_three_parts(g);

	ConnectionSegment cs{};
	// Triangle 0-1-2
	assert(g.connect(IR(0, 10), IR(1, 21), std::tuple<>{}, cs));
	assert(g.connect(IR(1, 11), IR(2, 31), std::tuple<>{}, cs));
	assert(g.connect(IR(0, 12), IR(2, 31), std::tuple<>{}, cs));

	auto cc = g.component_view(PartId{0});
	FullGraphSchedulingPolicy<G> policy{};

	std::vector<UnorderedPair<PartId>> edges;
	for (auto e : policy.compute(cc)) {
		edges.push_back(e);
	}
	std::sort(edges.begin(), edges.end(),
	          [](const auto &x, const auto &y) {
		          return std::tie(x.first, x.second) <
		                 std::tie(y.first, y.second);
	          });

	std::vector<UnorderedPair<PartId>> expected{
	    UnorderedPair<PartId>{0, 1}, UnorderedPair<PartId>{0, 2},
	    UnorderedPair<PartId>{1, 2}};
	std::sort(expected.begin(), expected.end(),
	          [](const auto &x, const auto &y) {
		          return std::tie(x.first, x.second) <
		                 std::tie(y.first, y.second);
	          });
	assert(edges == expected);
}

static void test_exponential_skip_policy_n1() {
	G g;

	// Single part (pid 0)
	auto ifs = std::initializer_list<InterfaceSpec>{mk_stud(50, 1, 1),
	                                                mk_hole(51, 1, 1)};
	auto p0 = g.add_part<CustomPart>(
	    std::tuple<>{}, 0.1, BrickColor{1, 2, 3}, ifs);
	assert(p0 && *p0 == PartId{0});

	auto cc = g.component_view(PartId{0});
	ExponentialSkipSchedulingPolicy<G> policy{8};

	std::size_t count = 0;
	for (auto e : policy.compute(cc)) {
		(void)e;
		++count;
	}
	assert(count == 0);
}

static void test_exponential_skip_policy_n2_k1() {
	G g;

	auto ifs0 = std::initializer_list<InterfaceSpec>{mk_stud(60, 1, 1),
	                                                 mk_hole(61, 1, 1)};
	auto ifs1 = std::initializer_list<InterfaceSpec>{mk_stud(62, 1, 1),
	                                                 mk_hole(63, 1, 1)};
	auto p0 = g.add_part<CustomPart>(
	    std::tuple<>{}, 0.1, BrickColor{1, 2, 3}, ifs0);
	auto p1 = g.add_part<CustomPart>(
	    std::tuple<>{}, 0.2, BrickColor{4, 5, 6}, ifs1);
	assert(p0 && p1);

	ConnectionSegment cs{};
	assert(g.connect(IR(*p0, 60), IR(*p1, 63), std::tuple<>{}, cs));

	auto cc = g.component_view(*p0);
	ExponentialSkipSchedulingPolicy<G> policy{1};

	std::vector<UnorderedPair<PartId>> edges;
	for (auto e : policy.compute(cc)) {
		edges.push_back(e);
	}
	assert(edges.size() == 1);
	assert(edges[0].contains(*p0));
	assert(edges[0].contains(*p1));
}

static void test_exponential_skip_policy_n4_k2() {
	G g;

	// Four parts 0,1,2,3; connect in a chain so they are in one CC.
	for (int i = 0; i < 4; ++i) {
		auto ifs = std::initializer_list<InterfaceSpec>{
		    mk_stud(100 + 2 * i, 1, 1), mk_hole(101 + 2 * i, 1, 1)};
		auto p = g.add_part<CustomPart>(
		    std::tuple<>{}, 0.1 + 0.1 * i,
		    BrickColor{std::uint8_t(10 * i), 0, 0}, ifs);
		assert(p && *p == static_cast<PartId>(i));
	}

	ConnectionSegment cs{};
	for (int i = 0; i < 3; ++i) {
		assert(g.connect(IR(static_cast<PartId>(i),
		                    static_cast<InterfaceId>(100 + 2 * i)),
		                 IR(static_cast<PartId>(i + 1),
		                    static_cast<InterfaceId>(101 + 2 * (i + 1))),
		                 std::tuple<>{}, cs));
	}

	auto cc = g.component_view(PartId{0});
	ExponentialSkipSchedulingPolicy<G> policy{2};

	std::vector<UnorderedPair<PartId>> edges;
	for (auto e : policy.compute(cc)) {
		edges.push_back(e);
	}

	// For n=4, k=2, expected edges:
	// i=0: (0,1), (0,2)
	// i=1: (1,2), (1,3)
	// i=2: (2,3)
	// i=3: none
	std::vector<UnorderedPair<PartId>> expected{
	    {PartId{0}, PartId{1}}, {PartId{0}, PartId{2}},
	    {PartId{1}, PartId{2}}, {PartId{1}, PartId{3}},
	    {PartId{2}, PartId{3}}};

	std::sort(edges.begin(), edges.end(),
	          [](const auto &x, const auto &y) {
		          return std::tie(x.first, x.second) <
		                 std::tie(y.first, y.second);
	          });
	std::sort(expected.begin(), expected.end(),
	          [](const auto &x, const auto &y) {
		          return std::tie(x.first, x.second) <
		                 std::tie(y.first, y.second);
	          });

	assert(edges == expected);
}

} // namespace

int main() {
	test_scheduler_dtor_no_constraints();
	test_scheduler_dtor_with_constraints();
	test_single_change_block_batches_flush();
	test_nested_change_blocks_flush_once();
	test_change_block_moved_once();

	test_on_part_added_outside_block();
	test_on_part_added_inside_block();

	test_connect_two_single_parts();
	test_connect_within_same_cc_tree_only_no_churn();
	test_connect_within_same_cc_fullgraph_adds_edge();
	test_disconnect_non_bridge_edge_fullgraph();
	test_disconnect_bridge_edge_splits_cc();

	test_on_part_removed_no_constraints();
	test_on_part_removed_with_multiple_constraints();

	test_multiple_dirty_vertices_same_cc_only_one_compute();
	test_flush_with_no_dirty_vertices_is_noop();

	test_no_change_in_desired_edges_no_churn();
	test_add_new_desired_edges_only_create_new();
	test_cross_cc_edges_removed_after_split();

	test_tree_only_policy_on_path();
	test_fullgraph_policy_on_triangle();
	test_exponential_skip_policy_n1();
	test_exponential_skip_policy_n2_k1();
	test_exponential_skip_policy_n4_k2();
	return 0;
}
