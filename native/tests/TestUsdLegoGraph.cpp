import std;
import lego_assemble.core.specs;
import lego_assemble.core.connections;
import lego_assemble.core.graph;
import lego_assemble.usd.usd_graph;
import lego_assemble.usd.author;
import lego_assemble.usd.allocator;
import lego_assemble.usd.parse;
import lego_assemble.usd.tokens;
import lego_assemble.utils.transforms;
import lego_assemble.utils.type_list;
import lego_assemble.utils.usd_envs;
import lego_assemble.utils.sdf;
import lego_assemble.utils.conversions;
import lego_assemble.utils.metric_system;
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

static bool contains_path(const std::vector<pxr::SdfPath> &paths,
                          const pxr::SdfPath &target) {
	return std::ranges::find(paths, target) != paths.end();
}

static bool almost_equal(const pxr::GfVec3d &a, const pxr::GfVec3d &b,
                         double eps = 1e-6) {
	return (a - b).GetLengthSq() <= eps * eps;
}

static bool almost_equal(const pxr::GfQuatd &a, const pxr::GfQuatd &b,
                         double eps = 1e-6) {
	pxr::GfQuatd da = a;
	da.Normalize();
	pxr::GfQuatd db = b;
	db.Normalize();
	// Quaternions q and -q represent the same rotation.
	double dot = da.GetReal() * db.GetReal() +
	             da.GetImaginary()[0] * db.GetImaginary()[0] +
	             da.GetImaginary()[1] * db.GetImaginary()[1] +
	             da.GetImaginary()[2] * db.GetImaginary()[2];
	return std::abs(std::abs(dot) - 1.0) <= eps;
}

static void assert_brick_colliders(const UsdPartWrapper<BrickPart> &pw,
                                   const pxr::SdfPath &path) {
	auto colls = pw.colliders();
	assert(colls.size() == 2);

	pxr::SdfPath hole_path = path.AppendChild(LegoTokens->BodyCollider);
	pxr::SdfPath stud_path = path.AppendChild(LegoTokens->TopCollider);

	bool seen_hole = false;
	bool seen_stud = false;
	for (const auto &[iface, collider_path] : colls) {
		if (iface == BrickPart::HoleId) {
			seen_hole = true;
			assert(collider_path == hole_path);
		} else if (iface == BrickPart::StudId) {
			seen_stud = true;
			assert(collider_path == stud_path);
		} else {
			assert(false);
		}
	}
	assert(seen_hole && seen_stud);
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
	auto allocA =
	    alloc.allocate_part_managed<PrototypeBrickAuthor>(env_id, brickA);
	auto allocB =
	    alloc.allocate_part_managed<PrototypeBrickAuthor>(env_id, brickB);
	pxr::SdfPath pathA = std::get<0>(allocA);
	pxr::SdfPath pathB = std::get<0>(allocB);

	ConnectionSegment cs{}; // default offset(0,0), yaw=0
	pxr::SdfPath connPath = alloc.allocate_conn_managed(
	    pathA, BrickPart::StudId, pathB, BrickPart::HoleId, cs);
	(void)connPath;

	CountingResource arena;
	G g(stage, &arena);

	// Two physical bricks and a single realized connection between them.
	assert(g.topology().parts().size() == 2);
	assert(g.topology().connection_segments().size() == 1);
	assert(g.topology().connection_bundles().size() == 1);

	// Path → PartId mapping
	const PartId *pidA =
	    g.topology().parts().project_key<pxr::SdfPath, PartId>(pathA);
	const PartId *pidB =
	    g.topology().parts().project_key<pxr::SdfPath, PartId>(pathB);
	assert(pidA && pidB);

	using PW = UsdPartWrapper<BrickPart>;
	const PW *pA = g.topology().parts().get<PW>(*pidA);
	const PW *pB = g.topology().parts().get<PW>(*pidB);
	assert(pA && pB);
	assert_brick_colliders(*pA, pathA);
	assert_brick_colliders(*pB, pathB);

	// Adjacency
	assert(pA->outgoings().size() == 1);
	assert(pB->incomings().size() == 1);
	assert(pA->neighbor_parts().contains(*pidB));
	assert(pB->neighbor_parts().contains(*pidA));

	// Dynamic graph connectivity
	const DgVertexId *a_dg =
	    g.topology().parts().project_key<PartId, DgVertexId>(*pidA);
	const DgVertexId *b_dg =
	    g.topology().parts().project_key<PartId, DgVertexId>(*pidB);
	assert(a_dg && b_dg);
	assert(
	    g.topology().dynamic_graph().connected(a_dg->value(), b_dg->value()));

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

	assert(g.topology().parts().size() == 0);
	assert(g.topology().connection_segments().size() == 0);
	assert(g.topology().connection_bundles().size() == 0);

	BrickColor red{255, 0, 0};
	BrickColor green{0, 255, 0};

	BrickPart brickA = make_brick(2, 4, BrickHeightPerPlate, red);
	BrickPart brickB = make_brick(2, 4, BrickHeightPerPlate, green);

	std::int64_t env_id = 1;
	create_env_root(stage, env_id);
	auto allocA =
	    alloc.allocate_part_managed<PrototypeBrickAuthor>(env_id, brickA);
	auto allocB =
	    alloc.allocate_part_managed<PrototypeBrickAuthor>(env_id, brickB);
	pxr::SdfPath pathA = std::get<0>(allocA);
	pxr::SdfPath pathB = std::get<0>(allocB);

	// After authoring the two bricks, the graph should now see two parts
	assert(g.topology().parts().size() == 2);
	assert(g.topology().connection_segments().size() == 0);
	assert(g.topology().connection_bundles().size() == 0);

	const PartId *pidA =
	    g.topology().parts().project_key<pxr::SdfPath, PartId>(pathA);
	const PartId *pidB =
	    g.topology().parts().project_key<pxr::SdfPath, PartId>(pathB);
	assert(pidA && pidB);

	// Now author a connection; graph must realize it
	ConnectionSegment cs{};
	pxr::SdfPath connPath = alloc.allocate_conn_managed(
	    pathA, BrickPart::StudId, pathB, BrickPart::HoleId, cs);
	(void)connPath;

	assert(g.topology().connection_segments().size() == 1);
	assert(g.topology().connection_bundles().size() == 1);

	using PW = UsdPartWrapper<BrickPart>;
	const PW *pA = g.topology().parts().get<PW>(*pidA);
	const PW *pB = g.topology().parts().get<PW>(*pidB);
	assert(pA && pB);
	assert_brick_colliders(*pA, pathA);
	assert_brick_colliders(*pB, pathB);

	assert(pA->outgoings().size() == 1);
	assert(pB->incomings().size() == 1);
	assert(pA->neighbor_parts().contains(*pidB));
	assert(pB->neighbor_parts().contains(*pidA));

	const DgVertexId *a_dg =
	    g.topology().parts().project_key<PartId, DgVertexId>(*pidA);
	const DgVertexId *b_dg =
	    g.topology().parts().project_key<PartId, DgVertexId>(*pidB);
	assert(a_dg && b_dg);
	assert(
	    g.topology().dynamic_graph().connected(a_dg->value(), b_dg->value()));
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
	assert(g.topology().parts().size() == 0);
	assert(g.topology().connection_segments().size() == 0);
	assert(g.topology().connection_bundles().size() == 0);

	// Now author the bricks exactly at the paths referenced by the connection
	BrickColor red{255, 0, 0};
	BrickColor green{0, 255, 0};
	BrickPart brickA = make_brick(2, 4, BrickHeightPerPlate, red);
	BrickPart brickB = make_brick(2, 4, BrickHeightPerPlate, green);

	PrototypeBrickAuthor{}(stage, pathA, brickA);
	PrototypeBrickAuthor{}(stage, pathB, brickB);

	// After TfNotice-driven resync, the graph should have 2 parts and 1
	// realized connection
	assert(g.topology().parts().size() == 2);
	assert(g.topology().connection_segments().size() == 1);
	assert(g.topology().connection_bundles().size() == 1);

	const PartId *pidA =
	    g.topology().parts().project_key<pxr::SdfPath, PartId>(pathA);
	const PartId *pidB =
	    g.topology().parts().project_key<pxr::SdfPath, PartId>(pathB);
	assert(pidA && pidB);

	using PW = UsdPartWrapper<BrickPart>;
	const PW *pA = g.topology().parts().get<PW>(*pidA);
	const PW *pB = g.topology().parts().get<PW>(*pidB);
	assert(pA && pB);
	assert_brick_colliders(*pA, pathA);
	assert_brick_colliders(*pB, pathB);

	assert(pA->outgoings().size() == 1);
	assert(pB->incomings().size() == 1);
	assert(pA->neighbor_parts().contains(*pidB));
	assert(pB->neighbor_parts().contains(*pidA));

	const DgVertexId *a_dg =
	    g.topology().parts().project_key<PartId, DgVertexId>(*pidA);
	const DgVertexId *b_dg =
	    g.topology().parts().project_key<PartId, DgVertexId>(*pidB);
	assert(a_dg && b_dg);
	assert(
	    g.topology().dynamic_graph().connected(a_dg->value(), b_dg->value()));
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
	auto allocA =
	    alloc.allocate_part_managed<PrototypeBrickAuthor>(env_id, brickA);
	auto allocB =
	    alloc.allocate_part_managed<PrototypeBrickAuthor>(env_id, brickB);
	pxr::SdfPath pathA = std::get<0>(allocA);
	pxr::SdfPath pathB = std::get<0>(allocB);

	ConnectionSegment cs{};
	pxr::SdfPath connPath = alloc.allocate_conn_managed(
	    pathA, BrickPart::StudId, pathB, BrickPart::HoleId, cs);

	G g(stage);

	assert(g.topology().parts().size() == 2);
	assert(g.topology().connection_segments().size() == 1);
	assert(g.topology().connection_bundles().size() == 1);

	const PartId *pidA =
	    g.topology().parts().project_key<pxr::SdfPath, PartId>(pathA);
	const PartId *pidB =
	    g.topology().parts().project_key<pxr::SdfPath, PartId>(pathB);
	assert(pidA && pidB);

	using PW = UsdPartWrapper<BrickPart>;
	const PW *pA = g.topology().parts().get<PW>(*pidA);
	const PW *pB = g.topology().parts().get<PW>(*pidB);
	assert(pA && pB);
	assert_brick_colliders(*pA, pathA);
	assert_brick_colliders(*pB, pathB);
	assert(pA->outgoings().size() == 1);
	assert(pB->incomings().size() == 1);

	// First deallocate the connection prim
	bool removed_conn = alloc.deallocate_managed_conn(connPath);
	assert(removed_conn);

	// Graph should have removed the connection & bundle and cleaned adjacency
	assert(g.topology().connection_segments().size() == 0);
	assert(g.topology().connection_bundles().size() == 0);

	pA = g.topology().parts().get<PW>(*pidA);
	pB = g.topology().parts().get<PW>(*pidB);
	assert(pA && pB);
	assert(pA->outgoings().size() == 0);
	assert(pB->incomings().size() == 0);
	assert(!pA->neighbor_parts().contains(*pidB));
	assert(!pB->neighbor_parts().contains(*pidA));

	// Now blow away all managed prims for this environment; both parts should
	// be removed from the graph.
	bool removed_all = alloc.deallocate_managed_all(env_id);
	assert(removed_all);

	assert(g.topology().parts().size() == 0);
	assert(g.topology().connection_segments().size() == 0);
	assert(g.topology().connection_bundles().size() == 0);
}

// Modifying a recognized brick prim in-place (e.g., changing its color) should
// be picked up by resync_tree and update the stored BrickPart while keeping the
// part realized.
static void test_resync_part_modified_updates_brick() {
	auto stage = make_stage();
	LegoAllocator alloc(stage);

	BrickColor red{255, 0, 0};
	BrickColor green{0, 255, 0};
	BrickPart brick = make_brick(2, 4, BrickHeightPerPlate, red);

	std::int64_t env_id = 9;
	create_env_root(stage, env_id);
	auto alloc_result =
	    alloc.allocate_part_managed<PrototypeBrickAuthor>(env_id, brick);
	pxr::SdfPath path = std::get<0>(alloc_result);

	CountingResource arena;
	G g(stage, &arena);

	assert(g.topology().parts().size() == 1);
	assert(g.topology().connection_segments().size() == 0);
	assert(g.topology().connection_bundles().size() == 0);

	const PartId *pid_before =
	    g.topology().parts().project_key<pxr::SdfPath, PartId>(path);
	assert(pid_before);
	using PW = UsdPartWrapper<BrickPart>;
	const PW *p_before = g.topology().parts().get<PW>(*pid_before);
	assert(p_before);
	assert_brick_colliders(*p_before, path);
	assert(p_before->wrapped().color() == red);

	pxr::UsdPrim prim = stage->GetPrimAtPath(path);
	assert(prim.IsValid());

	pxr::GfVec3i new_color_gf(0, 255, 0);
	prim.GetAttribute(LegoTokens->BrickColor).Set(new_color_gf);

	const PartId *pid_after =
	    g.topology().parts().project_key<pxr::SdfPath, PartId>(path);
	assert(pid_after);
	const PW *p_after = g.topology().parts().get<PW>(*pid_after);
	assert(p_after);
	assert_brick_colliders(*p_after, path);
	assert(p_after->wrapped().color() == green);

	assert(g.topology().parts().size() == 1);
	assert(g.topology().connection_segments().size() == 0);
	assert(g.topology().connection_bundles().size() == 0);
	assert(g.unrealized_parts().empty());
	assert(g.unrealized_connections().empty());
}

// If a previously recognized brick prim is edited so that BrickParser no
// longer recognizes it (e.g., lego:part_kind changed), resync_tree should
// remove the part from the topology while leaving the USD prim intact.
static void test_resync_part_becomes_unrecognized() {
	auto stage = make_stage();
	LegoAllocator alloc(stage);

	BrickColor red{255, 0, 0};
	BrickPart brick = make_brick(2, 4, BrickHeightPerPlate, red);

	std::int64_t env_id = 10;
	create_env_root(stage, env_id);
	auto alloc_result =
	    alloc.allocate_part_managed<PrototypeBrickAuthor>(env_id, brick);
	pxr::SdfPath path = std::get<0>(alloc_result);

	CountingResource arena;
	G g(stage, &arena);

	assert(g.topology().parts().size() == 1);

	pxr::UsdPrim prim = stage->GetPrimAtPath(path);
	assert(prim.IsValid());

	prim.GetAttribute(LegoTokens->PartKind)
	    .Set(pxr::TfToken("not_a_brick_kind"));

	assert(g.topology().parts().size() == 0);
	assert(stage->GetPrimAtPath(path).IsValid());
	assert(g.unrealized_parts().empty());
	assert(g.unrealized_connections().empty());
}

// Modifying an existing realized connection prim in-place (offset/yaw) through
// USD authoring should go through resync_tree and update the ConnectionSegment
// stored in the topology while keeping the connection realized.
static void test_resync_connection_modified_segment_updates_topology() {
	auto stage = make_stage();
	LegoAllocator alloc(stage);

	BrickColor red{255, 0, 0};
	BrickColor green{0, 255, 0};
	BrickPart brickA = make_brick(2, 4, BrickHeightPerPlate, red);
	BrickPart brickB = make_brick(2, 4, BrickHeightPerPlate, green);

	std::int64_t env_id = 11;
	create_env_root(stage, env_id);
	auto allocA =
	    alloc.allocate_part_managed<PrototypeBrickAuthor>(env_id, brickA);
	auto allocB =
	    alloc.allocate_part_managed<PrototypeBrickAuthor>(env_id, brickB);
	pxr::SdfPath pathA = std::get<0>(allocA);
	pxr::SdfPath pathB = std::get<0>(allocB);

	ConnectionSegment cs{};
	pxr::SdfPath connPath = alloc.allocate_conn_managed(
	    pathA, BrickPart::StudId, pathB, BrickPart::HoleId, cs);

	CountingResource arena;
	G g(stage, &arena);

	assert(g.topology().connection_segments().size() == 1);
	assert(g.topology().connection_bundles().size() == 1);

	pxr::UsdPrim connPrim = stage->GetPrimAtPath(connPath);
	assert(connPrim.IsValid());

	ConnectionSegment cs_modified{};
	cs_modified.offset = Eigen::Vector2i(1, -2);
	author_connection(stage, connPath, pathA, BrickPart::StudId, pathB,
	                  BrickPart::HoleId, cs_modified);

	const lego_assemble::ConnSegId *csid_after =
	    g.topology()
	        .connection_segments()
	        .project<pxr::SdfPath, lego_assemble::ConnSegId>(connPath);
	assert(csid_after);
	const SimpleWrapper<ConnectionSegment> *csw_after =
	    g.topology().connection_segments().find(*csid_after);
	assert(csw_after);
	assert(csw_after->wrapped().offset == cs_modified.offset);
	assert(csw_after->wrapped().yaw == cs_modified.yaw);

	assert(g.topology().connection_segments().size() == 1);
	assert(g.topology().connection_bundles().size() == 1);
	assert(g.unrealized_connections().empty());
	assert(g.unrealized_parts().empty());
}

// Deleting an unrealized managed connection prim from USD after the graph has
// seen it should remove the connection from bookkeeping and clear the
// corresponding unrealized parts.
static void test_resync_unrealized_connection_deleted_cleans_state() {
	auto stage = make_stage();
	LegoAllocator alloc(stage);

	pxr::SdfPath stud_path("/World/BrickA");
	pxr::SdfPath hole_path("/World/BrickB");
	ConnectionSegment cs{};
	pxr::SdfPath connPath = alloc.allocate_conn_managed(
	    stud_path, BrickPart::StudId, hole_path, BrickPart::HoleId, cs);

	CountingResource arena;
	G g(stage, &arena);

	assert(g.topology().parts().size() == 0);
	assert(g.topology().connection_segments().size() == 0);
	assert(g.topology().connection_bundles().size() == 0);

	auto unreal_parts_before = g.unrealized_parts();
	auto unreal_conns_before = g.unrealized_connections();
	assert(unreal_conns_before.size() == 1);
	assert(unreal_conns_before[0] == connPath);
	assert(unreal_parts_before.size() == 2);
	assert(contains_path(unreal_parts_before, stud_path));
	assert(contains_path(unreal_parts_before, hole_path));

	bool removed = alloc.deallocate_managed_conn(connPath);
	assert(removed);

	assert(g.topology().parts().size() == 0);
	assert(g.topology().connection_segments().size() == 0);
	assert(g.topology().connection_bundles().size() == 0);

	auto unreal_parts_after = g.unrealized_parts();
	auto unreal_conns_after = g.unrealized_connections();
	assert(unreal_conns_after.empty());
	assert(unreal_parts_after.empty());
}

// --------------------------- graph -> USD tests ---------------------------

// add_part should author a managed prim in USD and a corresponding part
// in the topology, for both kNoEnv (/World) and a regular env (/World/envs).
static void test_add_part_graph_to_usd() {
	auto stage = make_stage();

	BrickColor red{255, 0, 0};
	BrickColor green{0, 255, 0};
	BrickPart brickA = make_brick(2, 4, BrickHeightPerPlate, red);
	BrickPart brickB = make_brick(2, 4, BrickHeightPerPlate, green);

	CountingResource arena;
	G g(stage, &arena);

	// kNoEnv -> /World/managed/Parts/...
	std::int64_t env_world = kNoEnv;
	auto pathA_opt = g.add_part(env_world, brickA);
	assert(pathA_opt.has_value());
	pxr::SdfPath pathA = *pathA_opt;

	pxr::UsdPrim primA = stage->GetPrimAtPath(pathA);
	assert(primA.IsValid());

	const PartId *pidA =
	    g.topology().parts().project_key<pxr::SdfPath, PartId>(pathA);
	assert(pidA);

	// Regular env -> /World/envs/env_X/managed/Parts/...
	std::int64_t env_id = 10;
	create_env_root(stage, env_id);
	auto pathB_opt = g.add_part(env_id, brickB);
	assert(pathB_opt.has_value());
	pxr::SdfPath pathB = *pathB_opt;

	pxr::UsdPrim primB = stage->GetPrimAtPath(pathB);
	assert(primB.IsValid());

	const PartId *pidB =
	    g.topology().parts().project_key<pxr::SdfPath, PartId>(pathB);
	assert(pidB);

	// Two realized parts, no connections, nothing unrealized.
	assert(g.topology().parts().size() == 2);
	assert(g.topology().connection_segments().size() == 0);
	assert(g.topology().connection_bundles().size() == 0);
	assert(g.unrealized_parts().empty());
	assert(g.unrealized_connections().empty());
}

// remove_part on a managed part with no connections should:
// - remove the prim from USD
// - remove the part from the topology
// - return true the first time and false on repeated calls.
static void test_remove_part_no_connections_graph_api() {
	auto stage = make_stage();

	BrickColor red{255, 0, 0};
	BrickColor green{0, 255, 0};
	BrickPart brickA = make_brick(2, 4, BrickHeightPerPlate, red);
	BrickPart brickB = make_brick(2, 4, BrickHeightPerPlate, green);

	CountingResource arena;
	G g(stage, &arena);

	std::int64_t env_id = 3;
	create_env_root(stage, env_id);

	auto pathA_opt = g.add_part(env_id, brickA);
	auto pathB_opt = g.add_part(env_id, brickB);
	assert(pathA_opt.has_value() && pathB_opt.has_value());
	pxr::SdfPath pathA = *pathA_opt;
	pxr::SdfPath pathB = *pathB_opt;

	assert(g.topology().parts().size() == 2);

	// Remove B, which has no connections.
	bool removedB = g.remove_part(pathB);
	assert(removedB);
	assert(g.topology().parts().size() == 1);
	assert(!stage->GetPrimAtPath(pathB).IsValid());

	// Second attempt: nothing to remove -> false, no further changes.
	bool removedB_again = g.remove_part(pathB);
	assert(!removedB_again);
	assert(g.topology().parts().size() == 1);
	assert(stage->GetPrimAtPath(pathA).IsValid());
}

// remove_part on a recognized but unmanaged part should return false and leave
// both USD and graph state unchanged.
static void test_remove_part_unmanaged_returns_false() {
	auto stage = make_stage();

	BrickColor red{255, 0, 0};
	BrickPart brick = make_brick(2, 4, BrickHeightPerPlate, red);

	// Author a brick directly under /World via PrototypeBrickAuthor, which
	// is NOT under any managed group; initial_sync should still recognize it.
	pxr::SdfPath unmanaged_path("/World/FreeBrick");
	PrototypeBrickAuthor{}(stage, unmanaged_path, brick);

	CountingResource arena;
	G g(stage, &arena);

	assert(g.topology().parts().size() == 1);

	// remove_part should fail for unmanaged bricks and leave state intact.
	bool removed = g.remove_part(unmanaged_path);
	assert(!removed);
	assert(g.topology().parts().size() == 1);
	assert(stage->GetPrimAtPath(unmanaged_path).IsValid());
}

// Helper behavior: with only a managed connection authored in USD that
// references non-existent parts, the graph should report one unrealized
// connection and two unrealized part paths, and no realized topology.
static void test_unrealized_helpers_for_managed_connection_only() {
	auto stage = make_stage();

	// Author a managed connection under /World/managed/Conns that references
	// two brick paths that do not exist yet.
	LegoAllocator alloc(stage);
	pxr::SdfPath stud_path("/World/BrickA");
	pxr::SdfPath hole_path("/World/BrickB");
	ConnectionSegment cs{};
	pxr::SdfPath connPath = alloc.allocate_conn_managed(
	    stud_path, BrickPart::StudId, hole_path, BrickPart::HoleId, cs);

	CountingResource arena;
	G g(stage, &arena);

	// No realized topology yet.
	assert(g.topology().parts().size() == 0);
	assert(g.topology().connection_segments().size() == 0);
	assert(g.topology().connection_bundles().size() == 0);

	auto unreal_parts = g.unrealized_parts();
	auto unreal_conns = g.unrealized_connections();

	assert(unreal_conns.size() == 1);
	assert(unreal_conns[0] == connPath);
	assert(unreal_parts.size() == 2);
	assert(contains_path(unreal_parts, stud_path));
	assert(contains_path(unreal_parts, hole_path));
}

// remove_part on a realized part that participates in an unrealized
// managed connection should:
// - remove the part prim from USD
// - remove the unrealized connection prim from USD
// - clean up unrealized bookkeeping so that there are no lingering
//   unrealized parts or connections.
static void test_remove_part_with_unrealized_connection() {
	auto stage = make_stage();
	LegoAllocator alloc(stage);

	BrickColor red{255, 0, 0};
	BrickPart brickA = make_brick(2, 4, BrickHeightPerPlate, red);

	std::int64_t env_id = 5;
	create_env_root(stage, env_id);

	// Realized brick part at a managed path.
	auto alloc_result =
	    alloc.allocate_part_managed<PrototypeBrickAuthor>(env_id, brickA);
	pxr::SdfPath part_path = std::get<0>(alloc_result);

	// Hole path lives in the same env, but we deliberately do not
	// author a brick there so that the connection is unrealized.
	pxr::SdfPath hole_path = pathForEnv(env_id)
	                             .AppendChild(pxr::TfToken("managed"))
	                             .AppendChild(pxr::TfToken("Parts"))
	                             .AppendChild(pxr::TfToken("HoleOnly"));

	// Author a managed connection between part_path and hole_path.
	ConnectionSegment cs{};
	pxr::SdfPath conn_path = pathForEnv(env_id)
	                             .AppendChild(pxr::TfToken("managed"))
	                             .AppendChild(pxr::TfToken("Conns"))
	                             .AppendChild(pxr::TfToken("ConnUnrealized"));

	// When using allocate_conn_unmanaged, caller must ensure the parent
	// hierarchy exists in USD so that traversal can see the prim.
	{
		auto layer = stage->GetEditTarget().GetLayer();
		pxr::SdfPath managed_root =
		    pathForEnv(env_id).AppendChild(pxr::TfToken("managed"));
		if (!layer->GetPrimAtPath(managed_root)) {
			pxr::SdfChangeBlock _changes;
			auto managed_prim = pxr::SdfCreatePrimInLayer(layer, managed_root);
			managed_prim->SetSpecifier(pxr::SdfSpecifierDef);
			managed_prim->SetTypeName(pxr::UsdGeomTokens->Scope);
		}
		pxr::SdfPath conns_root =
		    managed_root.AppendChild(pxr::TfToken("Conns"));
		if (!layer->GetPrimAtPath(conns_root)) {
			pxr::SdfChangeBlock _changes;
			auto conns_prim = pxr::SdfCreatePrimInLayer(layer, conns_root);
			conns_prim->SetSpecifier(pxr::SdfSpecifierDef);
			conns_prim->SetTypeName(pxr::UsdGeomTokens->Scope);
		}
	}

	alloc.allocate_conn_unmanaged(conn_path, part_path, BrickPart::StudId,
	                              hole_path, BrickPart::HoleId, cs);

	CountingResource arena;
	G g(stage, &arena);

	// initial_sync: part_path is realized, hole_path is not; the connection
	// is therefore unrealized.
	assert(g.topology().parts().size() == 1);
	assert(g.topology().connection_segments().size() == 0);
	assert(g.topology().connection_bundles().size() == 0);

	auto unreal_parts_before = g.unrealized_parts();
	auto unreal_conns_before = g.unrealized_connections();
	assert(unreal_conns_before.size() == 1);
	assert(unreal_conns_before[0] == conn_path);
	// Only the missing endpoint should appear as an unrealized part.
	assert(!contains_path(unreal_parts_before, part_path));
	assert(contains_path(unreal_parts_before, hole_path));

	// Now remove the realized part via the graph API.
	bool removed = g.remove_part(part_path);
	assert(removed);

	// The part prim and connection prim should be gone from USD.
	assert(!stage->GetPrimAtPath(part_path).IsValid());
	assert(!stage->GetPrimAtPath(conn_path).IsValid());

	// No realized topology and no unrealized bookkeeping left that
	// refers to the removed part or connection.
	assert(g.topology().parts().size() == 0);
	assert(g.topology().connection_segments().size() == 0);
	assert(g.topology().connection_bundles().size() == 0);

	auto unreal_parts_after = g.unrealized_parts();
	auto unreal_conns_after = g.unrealized_connections();
	assert(unreal_conns_after.empty());
	assert(!contains_path(unreal_parts_after, part_path));
	assert(!contains_path(unreal_parts_after, hole_path));
}

// connect() should create a managed connection prim and corresponding
// topology state; disconnect() should tear both down.
static void test_connect_and_disconnect_realized_graph_api() {
	auto stage = make_stage();

	BrickColor red{255, 0, 0};
	BrickColor green{0, 255, 0};
	BrickPart brickA = make_brick(2, 4, BrickHeightPerPlate, red);
	BrickPart brickB = make_brick(2, 4, BrickHeightPerPlate, green);

	CountingResource arena;
	G g(stage, &arena);

	std::int64_t env_id = 7;
	create_env_root(stage, env_id);

	auto pathA_opt = g.add_part(env_id, brickA);
	auto pathB_opt = g.add_part(env_id, brickB);
	assert(pathA_opt.has_value() && pathB_opt.has_value());
	pxr::SdfPath pathA = *pathA_opt;
	pxr::SdfPath pathB = *pathB_opt;

	const PartId *pidA =
	    g.topology().parts().project_key<pxr::SdfPath, PartId>(pathA);
	const PartId *pidB =
	    g.topology().parts().project_key<pxr::SdfPath, PartId>(pathB);
	assert(pidA && pidB);

	InterfaceRef stud_if{*pidA, BrickPart::StudId};
	InterfaceRef hole_if{*pidB, BrickPart::HoleId};
	ConnectionSegment cs{};

	auto connPath_opt = g.connect(stud_if, hole_if, cs);
	assert(connPath_opt.has_value());
	auto [csid, connPath] = *connPath_opt;

	// Graph topology should now have exactly one realized connection.
	assert(g.topology().connection_segments().size() == 1);
	assert(g.topology().connection_bundles().size() == 1);

	using PW = UsdPartWrapper<BrickPart>;
	const PW *pA = g.topology().parts().get<PW>(*pidA);
	const PW *pB = g.topology().parts().get<PW>(*pidB);
	assert(pA && pB);
	assert_brick_colliders(*pA, pathA);
	assert_brick_colliders(*pB, pathB);
	assert(pA->outgoings().size() == 1);
	assert(pB->incomings().size() == 1);
	assert(pA->neighbor_parts().contains(*pidB));
	assert(pB->neighbor_parts().contains(*pidA));

	pxr::UsdPrim connPrim = stage->GetPrimAtPath(connPath);
	assert(connPrim.IsValid());

	// disconnect should succeed and tear down both USD and topology state.
	bool disconnected = g.disconnect(connPath);
	assert(disconnected);

	assert(!stage->GetPrimAtPath(connPath).IsValid());
	assert(g.topology().connection_segments().size() == 0);
	assert(g.topology().connection_bundles().size() == 0);

	pA = g.topology().parts().get<PW>(*pidA);
	pB = g.topology().parts().get<PW>(*pidB);
	assert(pA && pB);
	assert(pA->outgoings().size() == 0);
	assert(pB->incomings().size() == 0);
	assert(!pA->neighbor_parts().contains(*pidB));
	assert(!pB->neighbor_parts().contains(*pidA));

	// No unrealized connections should be left behind.
	assert(g.unrealized_connections().empty());
}

// connect() given InterfaceRefs that do not map to any existing parts in the
// topology should return std::nullopt and author no USD connection prim.
static void test_connect_invalid_interfaces_returns_nullopt() {
	auto stage = make_stage();
	CountingResource arena;
	G g(stage, &arena);

	InterfaceRef bad_stud{PartId{123u}, BrickPart::StudId};
	InterfaceRef bad_hole{PartId{456u}, BrickPart::HoleId};
	ConnectionSegment cs{};

	auto connPath_opt = g.connect(bad_stud, bad_hole, cs);
	assert(!connPath_opt.has_value());
}

// connect() with AlignPolicy::MoveHoleCC should:
// - keep the stud's connected component pose fixed (when it already has
//   an env-local pose),
// - move the hole's connected component so that the gain-graph relative
//   transform between stud and hole is satisfied in env coordinates.
static void test_connect_align_move_hole_component() {
	auto stage = make_stage();
	CountingResource arena;
	G g(stage, &arena);

	BrickColor red{255, 0, 0};
	BrickColor green{0, 255, 0};
	BrickPart brickA = make_brick(2, 4, BrickHeightPerPlate, red);
	BrickPart brickB = make_brick(2, 4, BrickHeightPerPlate, green);

	std::int64_t env_id = 11;
	create_env_root(stage, env_id);

	auto pathA_opt = g.add_part(env_id, brickA);
	auto pathB_opt = g.add_part(env_id, brickB);
	assert(pathA_opt.has_value() && pathB_opt.has_value());
	pxr::SdfPath pathA = *pathA_opt;
	pxr::SdfPath pathB = *pathB_opt;

	const PartId *pidA =
	    g.topology().parts().project_key<pxr::SdfPath, PartId>(pathA);
	const PartId *pidB =
	    g.topology().parts().project_key<pxr::SdfPath, PartId>(pathB);
	assert(pidA && pidB);

	// Give the stud's connected component (A) a non-trivial env-local pose.
	Eigen::Quaterniond qA(
	    Eigen::AngleAxisd(0.35 * std::numbers::pi, Eigen::Vector3d::UnitX()));
	Eigen::Vector3d tA(0.1, -0.2, 0.3);
	Transformd T_env_A{qA, tA};

	std::size_t updated = g.set_component_transform(*pidA, T_env_A);
	// Only A is in its component at this point.
	assert(updated == 1);

	// Record the stud's env-local pose before connect.
	std::optional<Transformd> poseA_before_opt =
	    g.part_pose_relative_to_env(*pidA);
	assert(poseA_before_opt.has_value());
	const Transformd &T_env_A_before = *poseA_before_opt;

	// Connect A (stud) to B (hole) and let connect() move hole's CC.
	InterfaceRef stud_if{*pidA, BrickPart::StudId};
	InterfaceRef hole_if{*pidB, BrickPart::HoleId};
	ConnectionSegment cs{};
	auto connPath_opt =
	    g.connect(stud_if, hole_if, cs, AlignPolicy::MoveHoleCC);
	assert(connPath_opt.has_value());

	// After connect, A's env pose must be unchanged.
	std::optional<Transformd> poseA_after_opt =
	    g.part_pose_relative_to_env(*pidA);
	std::optional<Transformd> poseB_after_opt =
	    g.part_pose_relative_to_env(*pidB);
	assert(poseA_after_opt.has_value());
	assert(poseB_after_opt.has_value());

	const Transformd &T_env_A_after = *poseA_after_opt;
	const Transformd &T_env_B_after = *poseB_after_opt;

	const auto &[qA_before, tA_before] = T_env_A_before;
	const auto &[qA_after, tA_after] = T_env_A_after;
	assert(
	    almost_equal(as<pxr::GfQuatd>(qA_before), as<pxr::GfQuatd>(qA_after)));
	assert(
	    almost_equal(as<pxr::GfVec3d>(tA_before), as<pxr::GfVec3d>(tA_after)));

	// The env-local transform between A and B must match the gain-graph
	// prediction {}^{A}T_B.
	auto T_A_B_opt = g.topology().lookup_transform<PartId>(*pidA, *pidB);
	assert(T_A_B_opt.has_value());
	const Transformd &T_A_B_graph = *T_A_B_opt;

	// {}^{A}T_B_read = {}^{A}T_env^{-1} * {}^{env}T_B
	Transformd T_A_B_read =
	    SE3d{}.project(inverse(T_env_A_after) * T_env_B_after);

	const auto &[q_graph, t_graph] = T_A_B_graph;
	const auto &[q_read, t_read] = T_A_B_read;

	assert(almost_equal(as<pxr::GfQuatd>(q_read), as<pxr::GfQuatd>(q_graph)));
	assert(almost_equal(as<pxr::GfVec3d>(t_read), as<pxr::GfVec3d>(t_graph)));
}

// connect() with AlignPolicy::MoveStudCC should:
// - keep the hole's connected component pose fixed (when it already has
//   an env-local pose),
// - move the stud's connected component so that the gain-graph relative
//   transform between stud and hole is satisfied in env coordinates.
static void test_connect_align_move_stud_component() {
	auto stage = make_stage();
	CountingResource arena;
	G g(stage, &arena);

	BrickColor red{255, 0, 0};
	BrickColor green{0, 255, 0};
	BrickPart brickA = make_brick(2, 4, BrickHeightPerPlate, red);
	BrickPart brickB = make_brick(2, 4, BrickHeightPerPlate, green);

	std::int64_t env_id = 12;
	create_env_root(stage, env_id);

	auto pathA_opt = g.add_part(env_id, brickA);
	auto pathB_opt = g.add_part(env_id, brickB);
	assert(pathA_opt.has_value() && pathB_opt.has_value());
	pxr::SdfPath pathA = *pathA_opt;
	pxr::SdfPath pathB = *pathB_opt;

	const PartId *pidA =
	    g.topology().parts().project_key<pxr::SdfPath, PartId>(pathA);
	const PartId *pidB =
	    g.topology().parts().project_key<pxr::SdfPath, PartId>(pathB);
	assert(pidA && pidB);

	// Give the hole's connected component (B) a non-trivial env-local pose.
	Eigen::Quaterniond qB(
	    Eigen::AngleAxisd(-0.2 * std::numbers::pi, Eigen::Vector3d::UnitZ()));
	Eigen::Vector3d tB(-0.4, 0.5, 0.2);
	Transformd T_env_B{qB, tB};

	std::size_t updated = g.set_component_transform(*pidB, T_env_B);
	// Only B is in its component at this point.
	assert(updated == 1);

	std::optional<Transformd> poseB_before_opt =
	    g.part_pose_relative_to_env(*pidB);
	assert(poseB_before_opt.has_value());
	const Transformd &T_env_B_before = *poseB_before_opt;

	// Connect A (stud) to B (hole) and let connect() move stud's CC.
	InterfaceRef stud_if{*pidA, BrickPart::StudId};
	InterfaceRef hole_if{*pidB, BrickPart::HoleId};
	ConnectionSegment cs{};
	auto connPath_opt =
	    g.connect(stud_if, hole_if, cs, AlignPolicy::MoveStudCC);
	assert(connPath_opt.has_value());

	std::optional<Transformd> poseA_after_opt =
	    g.part_pose_relative_to_env(*pidA);
	std::optional<Transformd> poseB_after_opt =
	    g.part_pose_relative_to_env(*pidB);
	assert(poseA_after_opt.has_value());
	assert(poseB_after_opt.has_value());

	const Transformd &T_env_A_after = *poseA_after_opt;
	const Transformd &T_env_B_after = *poseB_after_opt;

	// Hole's env pose must be unchanged.
	const auto &[qB_before, tB_before] = T_env_B_before;
	const auto &[qB_after, tB_after] = T_env_B_after;
	assert(
	    almost_equal(as<pxr::GfQuatd>(qB_before), as<pxr::GfQuatd>(qB_after)));
	assert(
	    almost_equal(as<pxr::GfVec3d>(tB_before), as<pxr::GfVec3d>(tB_after)));

	// Stud's env pose must agree with the gain-graph:
	// {}^{env}T_A_expected = {}^{env}T_B * ({}^{A}T_B)^{-1}
	auto T_A_B_opt = g.topology().lookup_transform<PartId>(*pidA, *pidB);
	assert(T_A_B_opt.has_value());
	const Transformd &T_A_B_graph = *T_A_B_opt;
	Transformd T_env_A_expected =
	    SE3d{}.project(T_env_B_after * inverse(T_A_B_graph));

	const auto &[qA_exp, tA_exp] = T_env_A_expected;
	const auto &[qA_after, tA_after2] = T_env_A_after;

	assert(almost_equal(as<pxr::GfQuatd>(qA_after), as<pxr::GfQuatd>(qA_exp)));
	assert(almost_equal(as<pxr::GfVec3d>(tA_after2), as<pxr::GfVec3d>(tA_exp)));
}

// connect() with AlignPolicy::None should not change env-local poses of either
// connected component; it should only add topology + USD connection prim.
static void test_connect_align_policy_none_preserves_env_poses() {
	auto stage = make_stage();
	CountingResource arena;
	G g(stage, &arena);

	BrickColor red{255, 0, 0};
	BrickColor green{0, 255, 0};
	BrickPart brickA = make_brick(2, 4, BrickHeightPerPlate, red);
	BrickPart brickB = make_brick(2, 4, BrickHeightPerPlate, green);

	std::int64_t env_id = 13;
	create_env_root(stage, env_id);

	auto pathA_opt = g.add_part(env_id, brickA);
	auto pathB_opt = g.add_part(env_id, brickB);
	assert(pathA_opt.has_value() && pathB_opt.has_value());
	pxr::SdfPath pathA = *pathA_opt;
	pxr::SdfPath pathB = *pathB_opt;

	const PartId *pidA =
	    g.topology().parts().project_key<pxr::SdfPath, PartId>(pathA);
	const PartId *pidB =
	    g.topology().parts().project_key<pxr::SdfPath, PartId>(pathB);
	assert(pidA && pidB);

	// Give both components distinct env-local poses while they are still
	// disconnected.
	Eigen::Quaterniond qA(
	    Eigen::AngleAxisd(0.1 * std::numbers::pi, Eigen::Vector3d::UnitY()));
	Eigen::Vector3d tA(0.2, 0.1, -0.3);
	Transformd T_env_A{qA, tA};

	Eigen::Quaterniond qB(
	    Eigen::AngleAxisd(-0.3 * std::numbers::pi, Eigen::Vector3d::UnitX()));
	Eigen::Vector3d tB(-0.5, 0.4, 0.6);
	Transformd T_env_B{qB, tB};

	std::size_t updatedA = g.set_component_transform(*pidA, T_env_A);
	std::size_t updatedB = g.set_component_transform(*pidB, T_env_B);
	assert(updatedA == 1);
	assert(updatedB == 1);

	std::optional<Transformd> poseA_before_opt =
	    g.part_pose_relative_to_env(*pidA);
	std::optional<Transformd> poseB_before_opt =
	    g.part_pose_relative_to_env(*pidB);
	assert(poseA_before_opt.has_value());
	assert(poseB_before_opt.has_value());

	const Transformd &T_env_A_before = *poseA_before_opt;
	const Transformd &T_env_B_before = *poseB_before_opt;

	InterfaceRef stud_if{*pidA, BrickPart::StudId};
	InterfaceRef hole_if{*pidB, BrickPart::HoleId};
	ConnectionSegment cs{};
	auto connPath_opt = g.connect(stud_if, hole_if, cs, AlignPolicy::None);
	assert(connPath_opt.has_value());

	std::optional<Transformd> poseA_after_opt =
	    g.part_pose_relative_to_env(*pidA);
	std::optional<Transformd> poseB_after_opt =
	    g.part_pose_relative_to_env(*pidB);
	assert(poseA_after_opt.has_value());
	assert(poseB_after_opt.has_value());

	const Transformd &T_env_A_after = *poseA_after_opt;
	const Transformd &T_env_B_after = *poseB_after_opt;

	const auto &[qA_before, tA_before] = T_env_A_before;
	const auto &[qA_after, tA_after2] = T_env_A_after;
	assert(
	    almost_equal(as<pxr::GfQuatd>(qA_before), as<pxr::GfQuatd>(qA_after)));
	assert(
	    almost_equal(as<pxr::GfVec3d>(tA_before), as<pxr::GfVec3d>(tA_after2)));

	const auto &[qB_before, tB_before] = T_env_B_before;
	const auto &[qB_after, tB_after] = T_env_B_after;
	assert(
	    almost_equal(as<pxr::GfQuatd>(qB_before), as<pxr::GfQuatd>(qB_after)));
	assert(
	    almost_equal(as<pxr::GfVec3d>(tB_before), as<pxr::GfVec3d>(tB_after)));
}

// disconnect() on a non-existent or unmanaged path should return false and
// not affect state.
static void test_disconnect_nonexistent_or_unmanaged_returns_false() {
	auto stage = make_stage();
	CountingResource arena;
	G g(stage, &arena);

	// Non-existent managed-like path.
	pxr::SdfPath bogus_managed("/World/managed/Conns/DoesNotExist");
	assert(!g.disconnect(bogus_managed));

	// Unmanaged connection prim: author it directly, but outside managed
	// groups, so LegoAllocator's deallocate_managed_conn will return false.
	pxr::SdfPath stud_path("/World/BrickA");
	pxr::SdfPath hole_path("/World/BrickB");
	BrickColor red{255, 0, 0};
	BrickColor green{0, 255, 0};
	BrickPart brickA = make_brick(2, 4, BrickHeightPerPlate, red);
	BrickPart brickB = make_brick(2, 4, BrickHeightPerPlate, green);
	PrototypeBrickAuthor{}(stage, stud_path, brickA);
	PrototypeBrickAuthor{}(stage, hole_path, brickB);

	LegoAllocator alloc(stage);
	ConnectionSegment cs{};
	pxr::SdfPath unmanaged_conn("/World/UnmanagedConn");
	author_connection(stage, unmanaged_conn, stud_path, BrickPart::StudId,
	                  hole_path, BrickPart::HoleId, cs);

	// Reconstruct G so it picks up the bricks and the unmanaged connection.
	G g2(stage, &arena);

	// The unmanaged connection is realized in topology/conn_path_table_, but
	// disconnect via graph API should return false because the prim is not
	// in a managed group.
	assert(!g2.disconnect(unmanaged_conn));
}

// set_component_transform on a single managed part should author translate,
// orient and scale xformOps that match the requested env-local pose.
static void test_set_component_transform_single_part() {
	auto stage = make_stage();
	CountingResource arena;
	G g(stage, &arena);

	BrickColor red{255, 0, 0};
	BrickPart brick = make_brick(2, 4, BrickHeightPerPlate, red);

	std::int64_t env_id = kNoEnv;
	auto path_opt = g.add_part(env_id, brick);
	assert(path_opt.has_value());
	pxr::SdfPath path = *path_opt;

	const PartId *pid =
	    g.topology().parts().project_key<pxr::SdfPath, PartId>(path);
	assert(pid);

	// Desired env-local pose for the anchor (in meters).
	Eigen::Quaterniond q = Eigen::Quaterniond::Identity();
	Eigen::Vector3d t(0.1, -0.2, 0.3);
	Transformd T_env{q, t};

	std::size_t updated = g.set_component_transform(*pid, T_env);
	assert(updated == 1);

	pxr::UsdPrim prim = stage->GetPrimAtPath(path);
	assert(prim.IsValid());
	pxr::UsdGeomXformable xf(prim);
	assert(bool(xf));

	bool resets_stack = false;
	std::vector<pxr::UsdGeomXformOp> ops = xf.GetOrderedXformOps(&resets_stack);
	assert(ops.size() == 3);
	assert(ops[0].GetOpName() == pxr::TfToken("xformOp:translate"));
	assert(ops[1].GetOpName() == pxr::TfToken("xformOp:orient"));
	assert(ops[2].GetOpName() == pxr::TfToken("xformOp:scale"));

	pxr::GfMatrix4d M_local;
	bool resets = false;
	assert(xf.GetLocalTransformation(&M_local, &resets));

	pxr::GfVec3d got_t = M_local.ExtractTranslation();
	pxr::GfQuatd got_q = M_local.ExtractRotationQuat();

	MetricSystem metrics(stage);
	pxr::GfVec3d exp_t(metrics.from_m(t[0]), metrics.from_m(t[1]),
	                   metrics.from_m(t[2]));

	assert(almost_equal(got_t, exp_t));
	assert(almost_equal(got_q, pxr::GfQuatd(1.0, 0.0, 0.0, 0.0)));
}

// set_component_transform on a connected component should move both parts
// consistently with the gain-graph relative transform.
static void test_set_component_transform_two_parts_connected() {
	auto stage = make_stage();
	CountingResource arena;
	G g(stage, &arena);

	BrickColor red{255, 0, 0};
	BrickColor green{0, 255, 0};
	BrickPart brickA = make_brick(2, 4, BrickHeightPerPlate, red);
	BrickPart brickB = make_brick(2, 4, BrickHeightPerPlate, green);

	std::int64_t env_id = 7;
	create_env_root(stage, env_id);

	auto pathA_opt = g.add_part(env_id, brickA);
	auto pathB_opt = g.add_part(env_id, brickB);
	assert(pathA_opt.has_value() && pathB_opt.has_value());
	pxr::SdfPath pathA = *pathA_opt;
	pxr::SdfPath pathB = *pathB_opt;

	const PartId *pidA =
	    g.topology().parts().project_key<pxr::SdfPath, PartId>(pathA);
	const PartId *pidB =
	    g.topology().parts().project_key<pxr::SdfPath, PartId>(pathB);
	assert(pidA && pidB);

	// Connect stud(A) to hole(B) with default ConnectionSegment,
	// which encodes a deterministic transform between A and B.
	InterfaceRef stud_if{*pidA, BrickPart::StudId};
	InterfaceRef hole_if{*pidB, BrickPart::HoleId};
	ConnectionSegment cs{};
	auto connPath_opt = g.connect(stud_if, hole_if, cs);
	assert(connPath_opt.has_value());

	// Choose a non-trivial env pose for A.
	Eigen::Quaterniond qA(
	    Eigen::AngleAxisd(0.25 * std::numbers::pi, Eigen::Vector3d::UnitZ()));
	Eigen::Vector3d tA(0.3, 0.4, 0.5);
	Transformd T_env_A{qA, tA};

	std::size_t updated = g.set_component_transform(*pidA, T_env_A);
	// Both A and B should be updated.
	assert(updated == 2);

	pxr::UsdPrim primA = stage->GetPrimAtPath(pathA);
	pxr::UsdPrim primB = stage->GetPrimAtPath(pathB);
	assert(primA.IsValid() && primB.IsValid());
	pxr::UsdGeomXformable xfA(primA);
	pxr::UsdGeomXformable xfB(primB);
	assert(bool(xfA) && bool(xfB));

	// Check A's env-local pose matches the requested one.
	pxr::UsdPrim env_prim =
	    stage->GetPrimAtPath(pathForEnv(env_id)); // /World/envs/env_7

	pxr::GfMatrix4d M_env_A = ComputeRelativeTransform(primA, env_prim);
	pxr::GfMatrix4d M_env_B = ComputeRelativeTransform(primB, env_prim);

	pxr::GfVec3d got_tA = M_env_A.ExtractTranslation();
	pxr::GfQuatd got_qA = M_env_A.ExtractRotationQuat();

	MetricSystem metricsA(stage);
	Transformd T_env_A_stage = metricsA.from_m(T_env_A);
	pxr::GfTransform env_A_tf =
	    as<pxr::GfTransform>(T_env_A_stage.first, T_env_A_stage.second);
	pxr::GfMatrix4d M_env_A_expected = env_A_tf.GetMatrix();
	pxr::GfVec3d exp_tA = M_env_A_expected.ExtractTranslation();
	pxr::GfQuatd exp_qA = M_env_A_expected.ExtractRotationQuat();

	assert(almost_equal(got_tA, exp_tA));
	assert(almost_equal(got_qA, exp_qA));

	// And that B's env pose matches the gain-graph prediction:
	// {}^{env}T_B_expected = {}^{env}T_A * {}^{A}T_B
	auto T_A_B_opt = g.topology().lookup_transform<PartId>(*pidA, *pidB);
	assert(T_A_B_opt.has_value());
	Transformd T_env_B_expected = SE3d{}.project(T_env_A * *T_A_B_opt);
	Transformd T_env_B_expected_stage =
	    MetricSystem(stage).from_m(T_env_B_expected);
	pxr::GfTransform env_B_tf = as<pxr::GfTransform>(
	    T_env_B_expected_stage.first, T_env_B_expected_stage.second);
	pxr::GfMatrix4d M_env_B_expected = env_B_tf.GetMatrix();

	pxr::GfVec3d got_tB = M_env_B.ExtractTranslation();
	pxr::GfQuatd got_qB = M_env_B.ExtractRotationQuat();
	pxr::GfVec3d exp_tB = M_env_B_expected.ExtractTranslation();
	pxr::GfQuatd exp_qB = M_env_B_expected.ExtractRotationQuat();

	assert(almost_equal(got_tB, exp_tB));
	assert(almost_equal(got_qB, exp_qB));
}

// part_pose_relative_to_env on a single managed part with default pose
// should return identity (env-local) in SI.
static void test_part_pose_relative_to_env_single_part() {
	auto stage = make_stage();
	CountingResource arena;
	G g(stage, &arena);

	BrickColor red{255, 0, 0};
	BrickPart brick = make_brick(2, 4, BrickHeightPerPlate, red);

	std::int64_t env_id = kNoEnv;
	auto path_opt = g.add_part(env_id, brick);
	assert(path_opt.has_value());
	pxr::SdfPath path = *path_opt;

	const PartId *pid =
	    g.topology().parts().project_key<pxr::SdfPath, PartId>(path);
	assert(pid);

	std::optional<Transformd> pose_opt = g.part_pose_relative_to_env(*pid);
	assert(pose_opt.has_value());
	const Transformd &T = *pose_opt;

	const auto &[q, t] = T;
	// Identity rotation and zero translation in SI.
	assert(almost_equal(as<pxr::GfQuatd>(q), pxr::GfQuatd(1.0, 0.0, 0.0, 0.0)));
	assert(almost_equal(as<pxr::GfVec3d>(t), pxr::GfVec3d(0.0, 0.0, 0.0)));
}

// part_pose_relative_to_env should agree with the gain-graph prediction
// after set_component_transform on a connected component.
static void test_part_pose_relative_to_env_two_parts_connected() {
	auto stage = make_stage();
	CountingResource arena;
	G g(stage, &arena);

	BrickColor red{255, 0, 0};
	BrickColor green{0, 255, 0};
	BrickPart brickA = make_brick(2, 4, BrickHeightPerPlate, red);
	BrickPart brickB = make_brick(2, 4, BrickHeightPerPlate, green);

	std::int64_t env_id = 9;
	create_env_root(stage, env_id);

	auto pathA_opt = g.add_part(env_id, brickA);
	auto pathB_opt = g.add_part(env_id, brickB);
	assert(pathA_opt.has_value() && pathB_opt.has_value());
	pxr::SdfPath pathA = *pathA_opt;
	pxr::SdfPath pathB = *pathB_opt;

	const PartId *pidA =
	    g.topology().parts().project_key<pxr::SdfPath, PartId>(pathA);
	const PartId *pidB =
	    g.topology().parts().project_key<pxr::SdfPath, PartId>(pathB);
	assert(pidA && pidB);

	// Connect A and B with default ConnectionSegment.
	InterfaceRef stud_if{*pidA, BrickPart::StudId};
	InterfaceRef hole_if{*pidB, BrickPart::HoleId};
	ConnectionSegment cs{};
	auto connPath_opt = g.connect(stud_if, hole_if, cs);
	assert(connPath_opt.has_value());

	// Set a non-trivial env pose for A in SI.
	Eigen::Quaterniond qA(
	    Eigen::AngleAxisd(0.25 * std::numbers::pi, Eigen::Vector3d::UnitZ()));
	Eigen::Vector3d tA(0.2, -0.1, 0.6);
	Transformd T_env_A{qA, tA};

	std::size_t updated = g.set_component_transform(*pidA, T_env_A);
	assert(updated == 2);

	// Read back env-local poses via part_pose_relative_to_env.
	std::optional<Transformd> poseA_opt = g.part_pose_relative_to_env(*pidA);
	std::optional<Transformd> poseB_opt = g.part_pose_relative_to_env(*pidB);
	assert(poseA_opt.has_value());
	assert(poseB_opt.has_value());

	const Transformd &T_env_A_read = *poseA_opt;
	const Transformd &T_env_B_read = *poseB_opt;

	// Build expected env-local transform for A using the same USD
	// pipeline as set_component_transform (SI -> stage -> GfTransform).
	MetricSystem metricsA(stage);
	Transformd T_env_A_stage = metricsA.from_m(T_env_A);
	pxr::GfTransform env_A_tf =
	    as<pxr::GfTransform>(T_env_A_stage.first, T_env_A_stage.second);
	pxr::GfMatrix4d M_env_A_expected = env_A_tf.GetMatrix();
	pxr::GfQuatd qA_exp_gf = M_env_A_expected.ExtractRotationQuat();
	pxr::GfVec3d tA_exp_stage = M_env_A_expected.ExtractTranslation();
	Eigen::Vector3d tA_exp_m = metricsA.to_m(as<Eigen::Vector3d>(tA_exp_stage));

	// A: pose must match the requested T_env_A (up to numeric tolerance),
	// as interpreted through USD's transform extraction.
	const auto &[qA_read, tA_read] = T_env_A_read;
	assert(almost_equal(as<pxr::GfQuatd>(qA_read), qA_exp_gf));
	assert(almost_equal(as<pxr::GfVec3d>(tA_read), as<pxr::GfVec3d>(tA_exp_m)));

	// B: pose must match T_env_A * {}^{A}T_B from the gain graph.
	auto T_A_B_opt = g.topology().lookup_transform<PartId>(*pidA, *pidB);
	assert(T_A_B_opt.has_value());
	Transformd T_env_B_expected = SE3d{}.project(T_env_A * *T_A_B_opt);
	const auto &[qB_read, tB_read] = T_env_B_read;

	// Expected env-local transform for B via the same USD pipeline.
	Transformd T_env_B_expected_stage = metricsA.from_m(T_env_B_expected);
	pxr::GfTransform env_B_tf = as<pxr::GfTransform>(
	    T_env_B_expected_stage.first, T_env_B_expected_stage.second);
	pxr::GfMatrix4d M_env_B_expected = env_B_tf.GetMatrix();
	pxr::GfQuatd qB_exp_gf = M_env_B_expected.ExtractRotationQuat();
	pxr::GfVec3d tB_exp_stage = M_env_B_expected.ExtractTranslation();
	Eigen::Vector3d tB_exp_m = metricsA.to_m(as<Eigen::Vector3d>(tB_exp_stage));

	assert(almost_equal(as<pxr::GfQuatd>(qB_read), qB_exp_gf));
	assert(almost_equal(as<pxr::GfVec3d>(tB_read), as<pxr::GfVec3d>(tB_exp_m)));
}

// --------------------------- hook tests ---------------------------

struct Hooks;
using GH = UsdLegoGraph<Parts, PartAuthors, PartParsers, Hooks>;

struct Hooks {
	using PW = UsdPartWrapper<BrickPart>;
	using CSW = SimpleWrapper<ConnectionSegment>;
	using CBW = SimpleWrapper<ConnectionBundle>;

	GH *g = nullptr;

	int added_calls = 0;
	int removing_calls = 0;
	int connected_calls = 0;
	int disconnecting_calls = 0;
	int bundle_created_calls = 0;
	int bundle_removing_calls = 0;

	PartId last_added_pid{};
	bool added_pw_matches_store = false;

	bool removing_alive_in_store = false;
	bool removing_has_any_connections = false;

	ConnSegId last_csid{};
	ConnSegRef last_csref{};
	bool connect_bundle_has_csid = false;
	bool disconnect_bundle_has_csid = false;

	ConnectionEndpoint last_created_ep{0, 0};
	ConnectionEndpoint last_removed_ep{0, 0};
	std::size_t created_bundle_size = 0;
	std::size_t removing_bundle_size = 0;

	template <class P> void on_part_added(PartId pid, UsdPartWrapper<P> &pw) {
		++added_calls;
		last_added_pid = pid;
		const auto *stored =
		    g->topology().parts().template get<UsdPartWrapper<P>>(pid);
		added_pw_matches_store = (stored == &pw);
	}

	template <class P>
	void on_part_removing(PartId pid, UsdPartWrapper<P> &pw) {
		++removing_calls;
		removing_alive_in_store = g->topology().parts().alive(pid);
		removing_has_any_connections =
		    (!pw.incomings().empty() || !pw.outgoings().empty());
	}

	void on_connected(ConnSegId csid, const ConnSegRef &csref,
	                  const InterfaceSpec &, const InterfaceSpec &, CSW &,
	                  CBW &cbw) {
		++connected_calls;
		last_csid = csid;
		last_csref = csref;
		connect_bundle_has_csid = cbw.wrapped().conn_seg_ids.contains(csid);
	}

	void on_disconnecting(ConnSegId csid, const ConnSegRef &, CSW &, CBW &cbw) {
		++disconnecting_calls;
		disconnect_bundle_has_csid = cbw.wrapped().conn_seg_ids.contains(csid);
	}

	void on_bundle_created(const ConnectionEndpoint &ep, CBW &cbw) {
		++bundle_created_calls;
		last_created_ep = ep;
		created_bundle_size = cbw.wrapped().conn_seg_ids.size();
	}

	void on_bundle_removing(const ConnectionEndpoint &ep, CBW &cbw) {
		++bundle_removing_calls;
		last_removed_ep = ep;
		removing_bundle_size = cbw.wrapped().conn_seg_ids.size();
	}
};

// Compile-time traits: default (NoHooks) graph should report no hooks.
static_assert(!G::HasAllOnPartAddedHooks);
static_assert(!G::HasAllOnPartRemovingHooks);
static_assert(!G::HasOnConnectedHook);
static_assert(!G::HasOnDisconnectingHook);
static_assert(!G::HasOnBundleCreatedHook);
static_assert(!G::HasOnBundleRemovingHook);
static_assert(!G::template HasOnPartAddedHook<BrickPart>);
static_assert(!G::template HasOnPartRemovingHook<BrickPart>);

// Hook-enabled graph should report all hooks available.
static_assert(GH::HasAllOnPartAddedHooks);
static_assert(GH::HasAllOnPartRemovingHooks);
static_assert(GH::HasOnConnectedHook);
static_assert(GH::HasOnDisconnectingHook);
static_assert(GH::HasOnBundleCreatedHook);
static_assert(GH::HasOnBundleRemovingHook);
static_assert(GH::template HasOnPartAddedHook<BrickPart>);
static_assert(GH::template HasOnPartRemovingHook<BrickPart>);

// For our single-part graph, per-part flags agree with aggregates.
static_assert(G::HasAllOnPartAddedHooks ==
              G::template HasOnPartAddedHook<BrickPart>);
static_assert(G::HasAllOnPartRemovingHooks ==
              G::template HasOnPartRemovingHook<BrickPart>);
static_assert(GH::HasAllOnPartAddedHooks ==
              GH::template HasOnPartAddedHook<BrickPart>);
static_assert(GH::HasAllOnPartRemovingHooks ==
              GH::template HasOnPartRemovingHook<BrickPart>);

// Hooks should be plumbed through get_hooks/set_hooks and invoked on graph ops.
static void test_usd_lego_graph_hooks_part_added_and_get_set() {
	auto stage = make_stage();
	CountingResource arena;
	GH g(stage, &arena);

	// Initially, hooks pointer is null.
	assert(g.get_hooks() == nullptr);

	Hooks hooks;
	g.set_hooks(&hooks);
	hooks.g = &g;

	assert(g.get_hooks() == &hooks);

	BrickColor red{255, 0, 0};
	BrickColor green{0, 255, 0};
	BrickPart brickA = make_brick(2, 4, BrickHeightPerPlate, red);
	BrickPart brickB = make_brick(2, 4, BrickHeightPerPlate, green);

	std::int64_t env_id = 42;
	create_env_root(stage, env_id);

	auto pathA_opt = g.add_part(env_id, brickA);
	auto pathB_opt = g.add_part(env_id, brickB);
	assert(pathA_opt.has_value() && pathB_opt.has_value());

	const PartId *pidA =
	    g.topology().parts().project_key<pxr::SdfPath, PartId>(*pathA_opt);
	const PartId *pidB =
	    g.topology().parts().project_key<pxr::SdfPath, PartId>(*pathB_opt);
	assert(pidA && pidB);

	// Hooks must have observed both part additions.
	assert(hooks.added_calls == 2);
	assert(hooks.last_added_pid == *pidB);
	assert(hooks.added_pw_matches_store);
}

// Connection lifecycle from graph API should drive connection-related hooks.
static void test_usd_lego_graph_hooks_connection_lifecycle() {
	auto stage = make_stage();
	CountingResource arena;
	GH g(stage, &arena);

	Hooks hooks;
	g.set_hooks(&hooks);
	hooks.g = &g;

	BrickColor red{255, 0, 0};
	BrickColor green{0, 255, 0};
	BrickPart brickA = make_brick(2, 4, BrickHeightPerPlate, red);
	BrickPart brickB = make_brick(2, 4, BrickHeightPerPlate, green);

	std::int64_t env_id = 7;
	create_env_root(stage, env_id);

	auto pathA_opt = g.add_part(env_id, brickA);
	auto pathB_opt = g.add_part(env_id, brickB);
	assert(pathA_opt.has_value() && pathB_opt.has_value());
	pxr::SdfPath pathA = *pathA_opt;
	pxr::SdfPath pathB = *pathB_opt;

	const PartId *pidA =
	    g.topology().parts().project_key<pxr::SdfPath, PartId>(pathA);
	const PartId *pidB =
	    g.topology().parts().project_key<pxr::SdfPath, PartId>(pathB);
	assert(pidA && pidB);

	InterfaceRef stud_if{*pidA, BrickPart::StudId};
	InterfaceRef hole_if{*pidB, BrickPart::HoleId};
	ConnectionSegment cs{};

	auto connPath_opt = g.connect(stud_if, hole_if, cs);
	assert(connPath_opt.has_value());
	auto [csid, connPath] = *connPath_opt;

	// First realized connection between this pair creates a bundle and a segment.
	assert(hooks.bundle_created_calls == 1);
	assert(hooks.connected_calls == 1);
	assert((hooks.last_created_ep == ConnectionEndpoint{*pidA, *pidB}));
	assert(hooks.created_bundle_size == 1);
	assert(hooks.connect_bundle_has_csid);

	// Disconnect via USD-facing API should drive disconnecting + bundle_removing.
	bool disconnected = g.disconnect(connPath);
	assert(disconnected);

	assert(hooks.disconnecting_calls == 1);
	assert(hooks.bundle_removing_calls == 1);
	assert((hooks.last_removed_ep == ConnectionEndpoint{*pidA, *pidB}));
	// By convention, bundle_removing sees the bundle while it still contains
	// the last connection segment.
	assert(hooks.removing_bundle_size == 1);
}

} // namespace

int main() {
	test_initial_sync_prepopulated_stage();
	test_incremental_alloc_via_allocator();
	test_connection_before_parts_unrealized_then_realized();
	test_deallocate_managed_removes_graph_state();
	test_resync_part_modified_updates_brick();
	test_resync_part_becomes_unrecognized();
	test_resync_connection_modified_segment_updates_topology();
	test_resync_unrealized_connection_deleted_cleans_state();
	test_add_part_graph_to_usd();
	test_remove_part_no_connections_graph_api();
	test_remove_part_unmanaged_returns_false();
	test_unrealized_helpers_for_managed_connection_only();
	test_remove_part_with_unrealized_connection();
	test_connect_and_disconnect_realized_graph_api();
	test_connect_invalid_interfaces_returns_nullopt();
	test_connect_align_move_hole_component();
	test_connect_align_move_stud_component();
	test_connect_align_policy_none_preserves_env_poses();
	test_disconnect_nonexistent_or_unmanaged_returns_false();
	test_set_component_transform_single_part();
	test_set_component_transform_two_parts_connected();
	test_part_pose_relative_to_env_single_part();
	test_part_pose_relative_to_env_two_parts_connected();
	test_usd_lego_graph_hooks_part_added_and_get_set();
	test_usd_lego_graph_hooks_connection_lifecycle();
	return 0;
}
