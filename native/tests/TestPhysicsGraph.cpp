import lego_assemble.core.specs;
import lego_assemble.core.graph;
import lego_assemble.core.connections;
import lego_assemble.core.assembly;
import lego_assemble.physx.physics_graph;
import lego_assemble.physx.patcher;
import lego_assemble.utils.type_list;
import lego_assemble.utils.metric_system;
import lego_assemble.utils.transforms;
import lego_assemble.vendor;

#include <PxPhysicsAPI.h>
#include <array>
#include <cassert>
#include <cstdint>
#include <initializer_list>
#include <memory_resource>
#include <vector>

using namespace lego_assemble;

struct TestHooks {
	struct AssembledEvent {
		ConnSegId csid;
		ConnSegRef csref;
		ConnectionSegment conn_seg;
	};

	std::vector<AssembledEvent> assembled_events;
	std::vector<ConnSegId> disassembled_events;

	void on_assembled(ConnSegId csid, const ConnSegRef &csref,
	                  const ConnectionSegment &conn_seg) {
		assembled_events.push_back(AssembledEvent{
		    .csid = csid,
		    .csref = csref,
		    .conn_seg = conn_seg,
		});
	}

	void on_disassembled(ConnSegId csid) {
		disassembled_events.push_back(csid);
	}
};

using TestGraph = PhysicsLegoGraph<type_list<BrickPart>, TestHooks>;
using TopologyGraph = TestGraph::TopologyGraph;
using ShapeMappingT = TestGraph::ShapeMapping;

static_assert(TopologyGraph::HasAllOnPartAddedHooks);
static_assert(TopologyGraph::HasAllOnPartRemovingHooks);
static_assert(TopologyGraph::HasOnConnectedHook);
static_assert(TopologyGraph::HasOnDisconnectingHook);
static_assert(TopologyGraph::HasOnBundleCreatedHook);
static_assert(TopologyGraph::HasOnBundleRemovingHook);
static_assert(TestGraph::HasOnAssembledHook);
static_assert(TestGraph::HasOnDisassembledHook);

// Ensure PhysX actor key is integrated into the part key set
static_assert(
    TopologyGraph::PartKeys::template contains<physx::PxRigidActor *>);

static_assert(std::is_class_v<ShapeMappingT>);

namespace {

// Minimal PhysX world for exercising PhysicsLegoGraph at runtime.
struct PhysxEnv {
	physx::PxDefaultAllocator allocator{};
	physx::PxDefaultErrorCallback error_callback{};
	physx::PxFoundation *foundation{nullptr};
	physx::PxPhysics *physics{nullptr};
	physx::PxDefaultCpuDispatcher *dispatcher{nullptr};
	physx::PxScene *scene{nullptr};
	physx::PxMaterial *material{nullptr};

	PhysxEnv() {
		foundation =
		    PxCreateFoundation(PX_PHYSICS_VERSION, allocator, error_callback);
		assert(foundation);

		physx::PxTolerancesScale scale;
		physics = PxCreatePhysics(PX_PHYSICS_VERSION, *foundation, scale, true,
		                          nullptr);
		assert(physics);

		physx::PxSceneDesc desc(physics->getTolerancesScale());
		desc.gravity = physx::PxVec3(0.0f, -9.81f, 0.0f);
		dispatcher = physx::PxDefaultCpuDispatcherCreate(1);
		assert(dispatcher);
		desc.cpuDispatcher = dispatcher;
		desc.filterShader = physx::PxDefaultSimulationFilterShader;
		scene = physics->createScene(desc);
		assert(scene);

		material = physics->createMaterial(0.5f, 0.5f, 0.6f);
		assert(material);
	}

	~PhysxEnv() {
		if (scene) {
			scene->release();
			scene = nullptr;
		}
		if (dispatcher) {
			dispatcher->release();
			dispatcher = nullptr;
		}
		if (physics) {
			physics->release();
			physics = nullptr;
		}
		if (foundation) {
			foundation->release();
			foundation = nullptr;
		}
	}
};

// Helper to build a tiny brick-like rigid actor with two shapes:
// one associated with the hole interface, one with the stud interface.
static physx::PxRigidDynamic *create_brick_actor(PhysxEnv &env,
                                                 const physx::PxTransform &pose,
                                                 physx::PxShape *&hole_shape,
                                                 physx::PxShape *&stud_shape) {
	physx::PxBoxGeometry geom(0.02f, 0.02f, 0.02f); // arbitrary small box

	auto *actor = env.physics->createRigidDynamic(pose);
	assert(actor);

	hole_shape = env.physics->createShape(geom, *env.material);
	assert(hole_shape);
	stud_shape = env.physics->createShape(geom, *env.material);
	assert(stud_shape);

	actor->attachShape(*hole_shape);
	actor->attachShape(*stud_shape);

	physx::PxRigidBodyExt::updateMassAndInertia(*actor, 1.0f);
	env.scene->addActor(*actor);

	return actor;
}

// Fixture that wires up a PhysicsLegoGraph with two BrickPart nodes
// backed by PhysX actors and shapes, and a single connection segment.
struct PhysicsGraphFixture {
	PhysxEnv env;
	MetricSystem metrics{1.0, 1.0, 1.0};
	TestHooks hooks{};
	TestGraph graph;

	physx::PxRigidDynamic *actorA{nullptr};
	physx::PxRigidDynamic *actorB{nullptr};
	physx::PxShape *holeShapeA{nullptr};
	physx::PxShape *studShapeA{nullptr};
	physx::PxShape *holeShapeB{nullptr};
	physx::PxShape *studShapeB{nullptr};

	PartId pidA{};
	PartId pidB{};
	ConnSegId csid{};

	PhysicsGraphFixture(bool create_initial_connection = true)
	    : env{}, metrics{1.0, 1.0, 1.0}, hooks{},
	      graph(metrics, env.physics, &hooks, {},
	            ContactExclusionLevel::Shape) {
		// Bind PhysX scene so callbacks are installed.
		assert(graph.bind_physx_scene(env.scene));

		// Two brick actors with per-interface shapes.
		actorA = create_brick_actor(
		    env, physx::PxTransform(physx::PxVec3(0.0f, 0.0f, 0.0f)),
		    holeShapeA, studShapeA);
		actorB = create_brick_actor(
		    env,
		    physx::PxTransform(physx::PxVec3(
		        0.0f, 0.0f,
		        static_cast<float>(BrickHeightPerPlate * PlateUnitHeight))),
		    holeShapeB, studShapeB);

		BrickColor red{255, 0, 0};
		BrickColor green{0, 255, 0};

		// Map interface ids to PhysX shapes for each part.
		std::initializer_list<InterfaceShapePair> ifsA{
		    InterfaceShapePair{BrickPart::HoleId, holeShapeA},
		    InterfaceShapePair{BrickPart::StudId, studShapeA},
		};
		std::initializer_list<InterfaceShapePair> ifsB{
		    InterfaceShapePair{BrickPart::HoleId, holeShapeB},
		    InterfaceShapePair{BrickPart::StudId, studShapeB},
		};

		auto pidA_opt = graph.topology().add_part<BrickPart>(
		    actorA, ifsA, BrickUnit{2}, BrickUnit{4},
		    PlateUnit{BrickHeightPerPlate}, red);
		auto pidB_opt = graph.topology().add_part<BrickPart>(
		    actorB, ifsB, BrickUnit{2}, BrickUnit{4},
		    PlateUnit{BrickHeightPerPlate}, green);
		assert(pidA_opt.has_value());
		assert(pidB_opt.has_value());
		pidA = *pidA_opt;
		pidB = *pidB_opt;

		if (create_initial_connection) {
			// Create one connection segment between stud(A) and hole(B).
			ConnectionSegment cs{};
			auto csid_opt = graph.topology().connect(
			    InterfaceRef{pidA, BrickPart::StudId},
			    InterfaceRef{pidB, BrickPart::HoleId}, cs);
			assert(csid_opt.has_value());
			csid = *csid_opt;

			const auto &bundles = graph.topology().connection_bundles();
			assert(bundles.size() == 1);
			ConnectionEndpoint ep{pidA, pidB};
			auto it = bundles.find(ep);
			assert(it != bundles.end());
		}
	}

	~PhysicsGraphFixture() {
		// Unbind scene (idempotent) before tearing down PhysX.
		(void)graph.unbind_physx_scene();

		if (actorA) {
			actorA->release();
			actorA = nullptr;
		}
		if (actorB) {
			actorB->release();
			actorB = nullptr;
		}
	}
};

// Ensure basic bind/unbind semantics for the PhysX scene.
static void test_bind_unbind_scene() {
	PhysxEnv env;
	MetricSystem metrics{1.0, 1.0, 1.0};
	TestGraph g(metrics, env.physics, nullptr, {},
	            ContactExclusionLevel::Shape);

	assert(g.bind_physx_scene(env.scene));
	// Second bind must fail (already bound).
	assert(!g.bind_physx_scene(env.scene));

	assert(g.unbind_physx_scene());
	// Second unbind must fail (already unbound).
	assert(!g.unbind_physx_scene());
}

// Verify that topology hooks wire up constraints and adjacency correctly.
static void test_topology_and_constraints() {
	PhysicsGraphFixture fx;

	// Topology should see two parts and one connection.
	assert(fx.graph.topology().parts().size() == 2);
	assert(fx.graph.topology().connection_segments().size() == 1);
	assert(fx.graph.topology().connection_bundles().size() == 1);

	// Adjacency in wrappers: stud has outgoing, hole has incoming, both are
	// neighbors of each other.
	using PW = PhysicsPartWrapper<BrickPart>;
	const PW *pA = fx.graph.topology().parts().find_value<PW>(fx.pidA);
	const PW *pB = fx.graph.topology().parts().find_value<PW>(fx.pidB);
	assert(pA && pB);
	assert(pA->outgoings().contains(fx.csid));
	assert(pB->incomings().contains(fx.csid));
	assert(pA->neighbor_parts().contains(fx.pidB));
	assert(pB->neighbor_parts().contains(fx.pidA));

	// Connection bundle stores the segment id
	ConnectionEndpoint ep{fx.pidA, fx.pidB};
	const auto &bundles = fx.graph.topology().connection_bundles();
	auto it = bundles.find(ep);
	assert(it != bundles.end());
	const auto &cbw = it->second;
	assert(cbw.wrapped().conn_seg_ids.contains(fx.csid));
}

// Exercise PhysxBinding::pairFound branches via the PxScene callback.
static void test_filter_callback_pairFound() {
	PhysicsGraphFixture fx;
	// Mirror the offsets used by lego_assemble.physx.patcher to locate
	// the internal Sc::Scene::mFilterCallback pointer.
	constexpr std::size_t offset_NpScene_mScene = 1440;
	constexpr std::size_t offset_NpScene_mScene_mFilterCallback =
	    offset_NpScene_mScene + 1016;
	auto locate_mFilterCallback = [](physx::PxScene *scene) {
		return reinterpret_cast<physx::PxSimulationFilterCallback **>(
		    reinterpret_cast<unsigned char *>(scene) +
		    offset_NpScene_mScene_mFilterCallback);
	};

	physx::PxSimulationFilterCallback **cb_ptr =
	    locate_mFilterCallback(fx.env.scene);
	assert(cb_ptr && *cb_ptr);

	auto *proxy = dynamic_cast<PxSimulationFilterPatch *>(*cb_ptr);
	assert(proxy != nullptr);

	physx::PxFilterData fd{};
	physx::PxPairFlags pairFlags{};

	// Case 1: two rigid actors that are parts but whose shapes do NOT have
	// contact exclusion => eCONTACT_EVENT_POSE must be enabled.
	pairFlags = physx::PxPairFlags{};
	(void)proxy->pairFound(1, {}, fd, fx.actorA, fx.holeShapeA, {}, fd,
	                       fx.actorB, fx.holeShapeB, pairFlags);
	assert(pairFlags.isSet(physx::PxPairFlag::eCONTACT_EVENT_POSE));

	// Case 2: pair matches a contact exclusion entry => filter result eKILL.
	pairFlags = physx::PxPairFlags{};
	auto kill = proxy->pairFound(2, {}, fd, fx.actorA, fx.studShapeA, {}, fd,
	                             fx.actorB, fx.holeShapeB, pairFlags);
	assert(kill == physx::PxFilterFlag::eKILL);

	// Case 3: one actor not tracked as a part => no CONTACT_EVENT_POSE flag.
	physx::PxRigidDynamic *extraActor = fx.env.physics->createRigidDynamic(
	    physx::PxTransform(physx::PxVec3(1.0f, 0.0f, 0.0f)));
	assert(extraActor);
	physx::PxShape *extraShape = fx.env.physics->createShape(
	    physx::PxBoxGeometry(0.01f, 0.01f, 0.01f), *fx.env.material);
	assert(extraShape);
	extraActor->attachShape(*extraShape);

	pairFlags = physx::PxPairFlags{};
	(void)proxy->pairFound(3, {}, fd, extraActor, extraShape, {}, fd, fx.actorB,
	                       fx.holeShapeB, pairFlags);
	assert(!pairFlags.isSet(physx::PxPairFlag::eCONTACT_EVENT_POSE));

	extraActor->release();
}

// Helper to build a simple PxContactPair and header for onContact tests.
struct ContactPayload {
	physx::PxContactPatch patch{};
	physx::PxReal impulses[1]{};
	physx::PxContactPair pair{};
	std::array<std::uint8_t, sizeof(physx::PxContactPairIndex) +
	                             sizeof(physx::PxContactPairPose)>
	    extra{};
	physx::PxContactPairHeader header{};
};

static ContactPayload
make_contact_payload(PhysicsGraphFixture &fx, physx::PxShape *shape0,
                     physx::PxShape *shape1, const physx::PxTransform &pose0,
                     const physx::PxTransform &pose1, bool set_impulse,
                     const physx::PxVec3 &normal) {
	ContactPayload p{};

	// Extra data: [Index(0), Pose]
	auto *idx = reinterpret_cast<physx::PxContactPairIndex *>(p.extra.data());
	idx->type = physx::PxContactPairExtraDataType::eCONTACT_PAIR_INDEX;
	idx->index = 0;

	auto *pose_item = reinterpret_cast<physx::PxContactPairPose *>(
	    p.extra.data() + sizeof(physx::PxContactPairIndex));
	pose_item->type = physx::PxContactPairExtraDataType::eCONTACT_EVENT_POSE;
	pose_item->globalPose[0] = pose0;
	pose_item->globalPose[1] = pose1;

	p.header.actors[0] = fx.actorA;
	p.header.actors[1] = fx.actorB;
	p.header.extraDataStream = p.extra.data();
	p.header.extraDataStreamSize = static_cast<physx::PxU16>(p.extra.size());
	p.header.flags = {};
	p.header.nbPairs = 1;
	p.header.pairs = &p.pair;

	// Single contact patch with optional impulse.
	p.patch.normal = normal;
	p.patch.startContactIndex = 0;
	p.patch.nbContacts = 1;

	p.impulses[0] = set_impulse ? physx::PxReal(10.0f) : physx::PxReal(0.0f);

	p.pair.shapes[0] = shape0;
	p.pair.shapes[1] = shape1;
	p.pair.contactPatches = reinterpret_cast<const physx::PxU8 *>(&p.patch);
	p.pair.contactImpulses = p.impulses;
	p.pair.patchCount = 1;
	p.pair.contactCount = 1;
	p.pair.contactStreamSize = 0;
	p.pair.requiredBufferSize = 0;
	p.pair.flags = {};
	if (set_impulse) {
		p.pair.flags |= physx::PxContactPairFlag::eINTERNAL_HAS_IMPULSES;
	}
	return p;
}

// Exercise PhysxBinding::onContact branches, including both the early-exit
// paths and the successful assembly detection path feeding the hooks + graph.
static void test_event_callback_onContact() {
	// Case 1: header flags mark removed actor => early return.
	{
		PhysicsGraphFixture fx(/*create_initial_connection=*/false);

		// One simulation step to ensure getElapsedTime(scene) returns a non-zero dt.
		fx.env.scene->simulate(1.0f / 60.0f);
		fx.env.scene->fetchResults(true);

		physx::PxSimulationEventCallback *cb_base =
		    fx.env.scene->getSimulationEventCallback();
		assert(cb_base != nullptr);
		auto *proxy = dynamic_cast<PxSimulationEventPatch *>(cb_base);
		assert(proxy != nullptr);

		ContactPayload payload =
		    make_contact_payload(fx, fx.studShapeA, fx.holeShapeB,
		                         physx::PxTransform(physx::PxVec3(0, 0, 0)),
		                         physx::PxTransform(physx::PxVec3(0, 0, 0)),
		                         /*set_impulse=*/true, physx::PxVec3(0, 0, -1));
		payload.header.flags = physx::PxContactPairHeaderFlag::eREMOVED_ACTOR_0;
		proxy->onContact(payload.header, &payload.pair, 1);

		fx.graph.do_post_step();
		assert(fx.hooks.assembled_events.empty());
		assert(fx.graph.topology().connection_segments().size() == 0);
	}

	// Case 2: shapes not mapped to any interface => ignored.
	{
		PhysicsGraphFixture fx(/*create_initial_connection=*/false);

		fx.env.scene->simulate(1.0f / 60.0f);
		fx.env.scene->fetchResults(true);

		physx::PxSimulationEventCallback *cb_base =
		    fx.env.scene->getSimulationEventCallback();
		assert(cb_base != nullptr);
		auto *proxy = dynamic_cast<PxSimulationEventPatch *>(cb_base);
		assert(proxy != nullptr);

		physx::PxShape *unmappedA = nullptr;
		physx::PxShape *unmappedB = nullptr;
		// Create two extra shapes, but do not register them in interface_shapes
		// of the wrapper.
		physx::PxBoxGeometry geom(0.01f, 0.01f, 0.01f);
		unmappedA = fx.env.physics->createShape(geom, *fx.env.material);
		unmappedB = fx.env.physics->createShape(geom, *fx.env.material);
		assert(unmappedA && unmappedB);
		fx.actorA->attachShape(*unmappedA);
		fx.actorB->attachShape(*unmappedB);

		ContactPayload payload =
		    make_contact_payload(fx, unmappedA, unmappedB,
		                         physx::PxTransform(physx::PxVec3(0, 0, 0)),
		                         physx::PxTransform(physx::PxVec3(0, 0, 0)),
		                         /*set_impulse=*/true, physx::PxVec3(0, 0, -1));
		proxy->onContact(payload.header, &payload.pair, 1);

		fx.graph.do_post_step();
		assert(fx.hooks.assembled_events.empty());
		assert(fx.graph.topology().connection_segments().size() == 0);

		// Clean up the extra shapes (actors hold references).
		unmappedA->release();
		unmappedB->release();
	}

	// Case 3: mapped interfaces but type combination is Hole/Hole =>
	// rejected before force threshold.
	{
		PhysicsGraphFixture fx(/*create_initial_connection=*/false);

		fx.env.scene->simulate(1.0f / 60.0f);
		fx.env.scene->fetchResults(true);

		physx::PxSimulationEventCallback *cb_base =
		    fx.env.scene->getSimulationEventCallback();
		assert(cb_base != nullptr);
		auto *proxy = dynamic_cast<PxSimulationEventPatch *>(cb_base);
		assert(proxy != nullptr);

		ContactPayload payload =
		    make_contact_payload(fx, fx.holeShapeA, fx.holeShapeB,
		                         physx::PxTransform(physx::PxVec3(0, 0, 0)),
		                         physx::PxTransform(physx::PxVec3(0, 0, 0)),
		                         /*set_impulse=*/true, physx::PxVec3(0, 0, -1));
		proxy->onContact(payload.header, &payload.pair, 1);

		fx.graph.do_post_step();
		assert(fx.hooks.assembled_events.empty());
		assert(fx.graph.topology().connection_segments().size() == 0);
	}

	// Case 4: full assembly detection path with stud(A) contacting hole(B)
	// and sufficient force to satisfy thresholds.
	{
		PhysicsGraphFixture fx(/*create_initial_connection=*/false);

		fx.env.scene->simulate(1.0f / 60.0f);
		fx.env.scene->fetchResults(true);

		physx::PxSimulationEventCallback *cb_base =
		    fx.env.scene->getSimulationEventCallback();
		assert(cb_base != nullptr);
		auto *proxy = dynamic_cast<PxSimulationEventPatch *>(cb_base);
		assert(proxy != nullptr);

		physx::PxTransform poseStud(physx::PxVec3(0.0f, 0.0f, 0.0f));
		physx::PxTransform poseHole(physx::PxVec3(
		    0.0f, 0.0f,
		    static_cast<float>(BrickHeightPerPlate * PlateUnitHeight)));

		ContactPayload payload = make_contact_payload(
		    fx, fx.studShapeA, fx.holeShapeB, poseStud, poseHole,
		    /*set_impulse=*/true,
		    // Force along -Z in stud frame.
		    physx::PxVec3(0, 0, -1));
		proxy->onContact(payload.header, &payload.pair, 1);

		fx.graph.do_post_step();
		assert(fx.hooks.assembled_events.size() == 1);
		const auto &evt = fx.hooks.assembled_events[0];

		// Exactly one connection segment should now exist in the topology.
		const auto &conn_segs = fx.graph.topology().connection_segments();
		assert(conn_segs.size() == 1);
		const ConnSegRef *stored_csref =
		    conn_segs.template find_key<ConnSegRef>(evt.csid);
		assert(stored_csref != nullptr);
		assert(*stored_csref == evt.csref);

		// Expect interfaces: stud on part A, hole on part B.
		assert((evt.csref.first == InterfaceRef{fx.pidA, BrickPart::StudId}));
		assert((evt.csref.second == InterfaceRef{fx.pidB, BrickPart::HoleId}));

		// ConnectionSegment should represent a perfect aligned stack:
		assert(evt.conn_seg.offset == Eigen::Vector2i(0, 0));

		// After processing once, subsequent processing should be idempotent.
		fx.graph.do_post_step();
		assert(fx.hooks.assembled_events.size() == 1);
		assert(fx.graph.topology().connection_segments().size() == 1);
	}
}

} // namespace

int main() {
	test_bind_unbind_scene();
	test_topology_and_constraints();
	test_filter_callback_pairFound();
	test_event_callback_onContact();
	return 0;
}
