import std;
import lego_assemble.core.specs;
import lego_assemble.core.graph;
import lego_assemble.physx.physics_graph;
import lego_assemble.utils.type_list;
import lego_assemble.utils.metric_system;
import lego_assemble.vendor.physx;
import lego_assemble.vendor.pxr;

#include <cassert>

using namespace lego_assemble;

using TestGraph = PhysicsLegoGraph<PartList<BrickPart>>;
using G = TestGraph::Base;
static_assert(G::HasOnPartAdded<TestGraph, BrickPart>);
static_assert(G::HasOnPartRemoving<TestGraph, BrickPart>);
static_assert(G::HasOnConnected<TestGraph>);
static_assert(G::HasOnDisconnecting<TestGraph>);
static_assert(G::HasOnBundleCreated<TestGraph>);
static_assert(G::HasOnBundleRemoving<TestGraph>);

// Ensure PhysX actor key is integrated into the part key set
static_assert(G::PartKeys::template contains<physx::PxRigidActor *>);
// Graph constructibility signature (PxPhysics*, pmr)
static_assert(
    std::is_constructible_v<TestGraph, MetricSystem, physx::PxPhysics *,
                            std::pmr::memory_resource *>);

// The graph exposes useful public aliases
using SkipGraphT = TestGraph::SkipGraph;
using ShapeMappingT = TestGraph::ShapeMapping;
static_assert(std::is_class_v<SkipGraphT>);
static_assert(std::is_class_v<ShapeMappingT>);

// Verify that augmenting extra part keys compiles and propagates
using TG2 =
    PhysicsLegoGraph<PartList<BrickPart>,
                     type_list<pxr::SdfPath>,       // extra part key
                     type_list<pxr::SdfPath::Hash>, // hash for extra key
                     type_list<std::equal_to<>>>;   // equality for extra key
using BG2 = TG2::Base;
static_assert(BG2::PartKeys::template contains<physx::PxRigidActor *>);
static_assert(BG2::PartKeys::template contains<pxr::SdfPath>);

// Compile-only smoke: construct graph and touch a few APIs without executing
// Anything inside the if(false) is still type-checked, forcing template checks.
static void compile_only_smoke() {
	if (1 + 1 != 2) {
		physx::PxPhysics *px = reinterpret_cast<physx::PxPhysics *>(1);
		TestGraph g(MetricSystem{}, px);
		// Free function utility also compiles
		physx::PxActor *a = nullptr;
		// Use public aliases to ensure types are visible and usable
		[[maybe_unused]] SkipGraphT *sg = nullptr;
		[[maybe_unused]] ShapeMappingT *sm = nullptr;
		(void)sg;
		(void)sm;
		(void)g;
		g.bind_physx_scene(reinterpret_cast<physx::PxScene *>(1));
		g.unbind_physx_scene();
	}
}

int main() {
	// Just link in the TU; all checks are compile-time.
	(void)compile_only_smoke;
	return 0;
}
