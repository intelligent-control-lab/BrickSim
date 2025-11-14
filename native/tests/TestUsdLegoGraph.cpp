import std;
import lego_assemble.core.specs;
import lego_assemble.core.connections;
import lego_assemble.core.graph;
import lego_assemble.usd.usd_graph;
import lego_assemble.usd.author;
import lego_assemble.usd.allocator;
import lego_assemble.usd.parse;
import lego_assemble.utils.transforms;
import lego_assemble.utils.type_list;
import lego_assemble.vendor.eigen;
import lego_assemble.vendor.pxr;

#include <cassert>

using namespace lego_assemble;

namespace {

using Parts = PartList<BrickPart>;
using PartAuthors = type_list<PrototypeBrickAuthor>;
using PartParsers = type_list<BrickParser>;
using G = UsdLegoGraph<Parts, PartAuthors, PartParsers>;

struct CountingResource : std::pmr::memory_resource {
	std::pmr::memory_resource *upstream;
	std::atomic<std::size_t> allocs{0}, deallocs{0};
	std::atomic<std::size_t> bytes_alloc{0}, bytes_dealloc{0};

	explicit CountingResource(
	    std::pmr::memory_resource *up = std::pmr::new_delete_resource())
	    : upstream(up) {}

  private:
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

static pxr::UsdStageRefPtr make_stage() {
	pxr::UsdStageRefPtr stage = pxr::UsdStage::CreateInMemory();
	// create default /World prim
	auto world_prim = pxr::SdfCreatePrimInLayer(
	    stage->GetEditTarget().GetLayer(), pxr::SdfPath("/World"));
	world_prim->SetSpecifier(pxr::SdfSpecifierDef);
	world_prim->SetTypeName(pxr::UsdGeomTokens->Xform);
	return stage;
}

static void create_env_root(const pxr::UsdStageRefPtr &stage,
                            std::int64_t env_id) {
	if (env_id == -1) {
		return;
	}
	auto layer = stage->GetEditTarget().GetLayer();
	pxr::SdfPath EnvRootPath("/World/envs");
	if (!layer->GetPrimAtPath(EnvRootPath)) {
		auto class_root = pxr::SdfCreatePrimInLayer(layer, EnvRootPath);
		class_root->SetSpecifier(pxr::SdfSpecifierDef);
		class_root->SetTypeName(pxr::UsdGeomTokens->Scope);
	}
	auto env_name = std::format("env_{}", env_id);
	auto env_path = EnvRootPath.AppendChild(pxr::TfToken(env_name));
	if (!layer->GetPrimAtPath(env_path)) {
		auto env_prim = pxr::SdfCreatePrimInLayer(layer, env_path);
		env_prim->SetSpecifier(pxr::SdfSpecifierDef);
		env_prim->SetTypeName(pxr::UsdGeomTokens->Xform);
	}
}

// Convenience helper
static BrickPart make_brick(BrickUnit L, BrickUnit W, PlateUnit H,
                            BrickColor color) {
	return BrickPart(L, W, H, color);
}

// --------------------------- tests ---------------------------

// Stage is populated before UsdLegoGraph is constructed; initial_sync should
// discover all parts and connections.
static void test_initial_sync_prepopulated_stage() {
	auto stage = make_stage();
	LegoAllocator alloc(stage);

	BrickColor red{255, 0, 0};
	BrickColor green{0, 255, 0};

	BrickPart brickA = make_brick(2, 4, BrickHeightPerPlate, red);
	BrickPart brickB = make_brick(2, 4, BrickHeightPerPlate, green);

	std::int64_t env_id = -1;
	create_env_root(stage, env_id);
	pxr::SdfPath pathA =
	    alloc.allocate_part_managed<PrototypeBrickAuthor>(env_id, brickA);
	pxr::SdfPath pathB =
	    alloc.allocate_part_managed<PrototypeBrickAuthor>(env_id, brickB);

	ConnectionSegment cs{}; // default offset(0,0), yaw=0
	pxr::SdfPath connPath = alloc.allocate_conn_managed(
	    pathA, BrickPart::StudId, pathB, BrickPart::HoleId, cs);
	(void)connPath;

	CountingResource arena;
	G g(stage, &arena);

	// Two physical bricks and a single realized connection between them.
	assert(g.parts().size() == 2);
	assert(g.connection_segments().size() == 1);
	assert(g.connection_bundles().size() == 1);

	// Path → PartId mapping
	const PartId *pidA = g.parts().project_key<pxr::SdfPath, PartId>(pathA);
	const PartId *pidB = g.parts().project_key<pxr::SdfPath, PartId>(pathB);
	assert(pidA && pidB);

	using PW = SimplePartWrapper<BrickPart>;
	const PW *pA = g.parts().get<PW>(*pidA);
	const PW *pB = g.parts().get<PW>(*pidB);
	assert(pA && pB);

	// Adjacency
	assert(pA->outgoings().size() == 1);
	assert(pB->incomings().size() == 1);
	assert(pA->neighbor_parts().contains(*pidB));
	assert(pB->neighbor_parts().contains(*pidA));

	// Dynamic graph connectivity
	const DgVertexId *a_dg = g.parts().project_key<PartId, DgVertexId>(*pidA);
	const DgVertexId *b_dg = g.parts().project_key<PartId, DgVertexId>(*pidB);
	assert(a_dg && b_dg);
	assert(g.dynamic_graph().connected(a_dg->value(), b_dg->value()));

	// pmr wiring: some allocations must have gone through our arena
	assert(arena.allocs.load() > 0);
}

// UsdLegoGraph is constructed first on an empty stage; later authoring bricks
// and connections via LegoAllocator should be picked up by the TfNotice sink.
static void test_incremental_alloc_via_allocator() {
	auto stage = make_stage();
	LegoAllocator alloc(stage);

	CountingResource arena;
	G g(stage, &arena);

	assert(g.parts().size() == 0);
	assert(g.connection_segments().size() == 0);
	assert(g.connection_bundles().size() == 0);

	BrickColor red{255, 0, 0};
	BrickColor green{0, 255, 0};

	BrickPart brickA = make_brick(2, 4, BrickHeightPerPlate, red);
	BrickPart brickB = make_brick(2, 4, BrickHeightPerPlate, green);

	std::int64_t env_id = 1;
	create_env_root(stage, env_id);
	pxr::SdfPath pathA =
	    alloc.allocate_part_managed<PrototypeBrickAuthor>(env_id, brickA);
	pxr::SdfPath pathB =
	    alloc.allocate_part_managed<PrototypeBrickAuthor>(env_id, brickB);

	// After authoring the two bricks, the graph should now see two parts
	assert(g.parts().size() == 2);
	assert(g.connection_segments().size() == 0);
	assert(g.connection_bundles().size() == 0);

	const PartId *pidA = g.parts().project_key<pxr::SdfPath, PartId>(pathA);
	const PartId *pidB = g.parts().project_key<pxr::SdfPath, PartId>(pathB);
	assert(pidA && pidB);

	// Now author a connection; graph must realize it
	ConnectionSegment cs{};
	pxr::SdfPath connPath = alloc.allocate_conn_managed(
	    pathA, BrickPart::StudId, pathB, BrickPart::HoleId, cs);
	(void)connPath;

	assert(g.connection_segments().size() == 1);
	assert(g.connection_bundles().size() == 1);

	using PW = SimplePartWrapper<BrickPart>;
	const PW *pA = g.parts().get<PW>(*pidA);
	const PW *pB = g.parts().get<PW>(*pidB);
	assert(pA && pB);

	assert(pA->outgoings().size() == 1);
	assert(pB->incomings().size() == 1);
	assert(pA->neighbor_parts().contains(*pidB));
	assert(pB->neighbor_parts().contains(*pidA));

	const DgVertexId *a_dg = g.parts().project_key<PartId, DgVertexId>(*pidA);
	const DgVertexId *b_dg = g.parts().project_key<PartId, DgVertexId>(*pidB);
	assert(a_dg && b_dg);
	assert(g.dynamic_graph().connected(a_dg->value(), b_dg->value()));
}

// Connection prim exists before either part prim; initial_sync should treat it
// as unrealized. Once bricks are authored at the referenced paths, the
// pending connection must be realized via resync_tree.
static void test_connection_before_parts_unrealized_then_realized() {
	auto stage = make_stage();

	pxr::SdfPath pathA("/World/BrickA");
	pxr::SdfPath pathB("/World/BrickB");
	pxr::SdfPath connPath("/World/ConnAB");

	ConnectionSegment cs{};
	author_connection(stage, connPath, pathA, BrickPart::StudId, pathB,
	                  BrickPart::HoleId, cs);

	CountingResource arena;
	G g(stage, &arena);

	// initial_sync: connection parsed but cannot be realized yet (no parts)
	assert(g.parts().size() == 0);
	assert(g.connection_segments().size() == 0);
	assert(g.connection_bundles().size() == 0);

	// Now author the bricks exactly at the paths referenced by the connection
	BrickColor red{255, 0, 0};
	BrickColor green{0, 255, 0};
	BrickPart brickA = make_brick(2, 4, BrickHeightPerPlate, red);
	BrickPart brickB = make_brick(2, 4, BrickHeightPerPlate, green);

	PrototypeBrickAuthor{}(stage, pathA, brickA);
	PrototypeBrickAuthor{}(stage, pathB, brickB);

	// After TfNotice-driven resync, the graph should have 2 parts and 1
	// realized connection
	assert(g.parts().size() == 2);
	assert(g.connection_segments().size() == 1);
	assert(g.connection_bundles().size() == 1);

	const PartId *pidA = g.parts().project_key<pxr::SdfPath, PartId>(pathA);
	const PartId *pidB = g.parts().project_key<pxr::SdfPath, PartId>(pathB);
	assert(pidA && pidB);

	using PW = SimplePartWrapper<BrickPart>;
	const PW *pA = g.parts().get<PW>(*pidA);
	const PW *pB = g.parts().get<PW>(*pidB);
	assert(pA && pB);

	assert(pA->outgoings().size() == 1);
	assert(pB->incomings().size() == 1);
	assert(pA->neighbor_parts().contains(*pidB));
	assert(pB->neighbor_parts().contains(*pidA));

	const DgVertexId *a_dg = g.parts().project_key<PartId, DgVertexId>(*pidA);
	const DgVertexId *b_dg = g.parts().project_key<PartId, DgVertexId>(*pidB);
	assert(a_dg && b_dg);
	assert(g.dynamic_graph().connected(a_dg->value(), b_dg->value()));
}

// Use LegoAllocator's deallocation APIs and verify that UsdLegoGraph keeps its
// internal state (parts, connections, adjacency, dynamic graph) consistent.
static void test_deallocate_managed_removes_graph_state() {
	auto stage = make_stage();
	LegoAllocator alloc(stage);

	BrickColor red{255, 0, 0};
	BrickColor green{0, 255, 0};
	BrickPart brickA = make_brick(2, 4, BrickHeightPerPlate, red);
	BrickPart brickB = make_brick(2, 4, BrickHeightPerPlate, green);

	std::int64_t env_id = 2;
	create_env_root(stage, env_id);
	pxr::SdfPath pathA =
	    alloc.allocate_part_managed<PrototypeBrickAuthor>(env_id, brickA);
	pxr::SdfPath pathB =
	    alloc.allocate_part_managed<PrototypeBrickAuthor>(env_id, brickB);

	ConnectionSegment cs{};
	pxr::SdfPath connPath = alloc.allocate_conn_managed(
	    pathA, BrickPart::StudId, pathB, BrickPart::HoleId, cs);

	G g(stage);

	assert(g.parts().size() == 2);
	assert(g.connection_segments().size() == 1);
	assert(g.connection_bundles().size() == 1);

	const PartId *pidA = g.parts().project_key<pxr::SdfPath, PartId>(pathA);
	const PartId *pidB = g.parts().project_key<pxr::SdfPath, PartId>(pathB);
	assert(pidA && pidB);

	using PW = SimplePartWrapper<BrickPart>;
	const PW *pA = g.parts().get<PW>(*pidA);
	const PW *pB = g.parts().get<PW>(*pidB);
	assert(pA && pB);
	assert(pA->outgoings().size() == 1);
	assert(pB->incomings().size() == 1);

	// First deallocate the connection prim
	bool removed_conn = alloc.deallocate_managed(connPath);
	assert(removed_conn);

	// Graph should have removed the connection & bundle and cleaned adjacency
	assert(g.connection_segments().size() == 0);
	assert(g.connection_bundles().size() == 0);

	pA = g.parts().get<PW>(*pidA);
	pB = g.parts().get<PW>(*pidB);
	assert(pA && pB);
	assert(pA->outgoings().size() == 0);
	assert(pB->incomings().size() == 0);
	assert(!pA->neighbor_parts().contains(*pidB));
	assert(!pB->neighbor_parts().contains(*pidA));

	// Now blow away all managed prims for this environment; both parts should
	// be removed from the graph.
	bool removed_all = alloc.deallocate_managed_all(env_id);
	assert(removed_all);

	assert(g.parts().size() == 0);
	assert(g.connection_segments().size() == 0);
	assert(g.connection_bundles().size() == 0);
}

} // namespace

int main() {
	test_initial_sync_prepopulated_stage();
	test_incremental_alloc_via_allocator();
	test_connection_before_parts_unrealized_then_realized();
	test_deallocate_managed_removes_graph_state();
	return 0;
}
