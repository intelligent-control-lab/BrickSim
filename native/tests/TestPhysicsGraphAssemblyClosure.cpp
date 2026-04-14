import std;
import bricksim.core.specs;
import bricksim.core.graph;
import bricksim.core.connections;
import bricksim.physx.assembly;
import bricksim.physx.shape_mapping;
import bricksim.physx.physics_graph;
import bricksim.physx.patcher;
import bricksim.utils.type_list;
import bricksim.utils.metric_system;
import bricksim.utils.transforms;
import bricksim.utils.conversions;
import bricksim.utils.c4_rotation;
import bricksim.vendor;

#include <cassert>
#include <foundation/PxPhysicsVersion.h>

using namespace bricksim;

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

	void on_disassembled(ConnSegId csid,
	                     [[maybe_unused]] const ConnSegRef &csref,
	                     [[maybe_unused]] const ConnectionSegment &conn_seg) {
		disassembled_events.push_back(csid);
	}
};

using TestGraph = PhysicsLegoGraph<type_list<BrickPart>, TestHooks>;
using TopologyGraph = TestGraph::TopologyGraph;

static_assert(TestGraph::HasOnAssembledHook);
static_assert(TestGraph::HasOnDisassembledHook);
static_assert(
    TopologyGraph::PartKeys::template contains<physx::PxRigidDynamic *>);

struct PhysxEnv {
	physx::PxDefaultAllocator allocator{};
	physx::PxDefaultErrorCallback error_callback{};
	physx::PxFoundation *foundation{nullptr};
	physx::PxPhysics *physics{nullptr};
	physx::PxDefaultCpuDispatcher *dispatcher{nullptr};
	physx::PxScene *scene{nullptr};
	physx::PxMaterial *material{nullptr};

	PhysxEnv() {
		foundation = physx::PxCreateFoundation(PX_PHYSICS_VERSION, allocator,
		                                       error_callback);
		assert(foundation);

		physx::PxTolerancesScale scale;
		physics = physx::PxCreatePhysics(PX_PHYSICS_VERSION, *foundation, scale,
		                                 true, nullptr);
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

physx::PxRigidDynamic *create_brick_actor(PhysxEnv &env, const BrickPart &part,
                                          const physx::PxTransform &pose,
                                          physx::PxShape *&hole_shape,
                                          physx::PxShape *&stud_shape) {
	physx::PxBoxGeometry geom(0.02f, 0.02f, 0.02f);

	auto *actor = env.physics->createRigidDynamic(pose);
	assert(actor);

	hole_shape = env.physics->createShape(geom, *env.material);
	assert(hole_shape);
	stud_shape = env.physics->createShape(geom, *env.material);
	assert(stud_shape);

	actor->attachShape(*hole_shape);
	actor->attachShape(*stud_shape);
	actor->setMass(static_cast<physx::PxReal>(part.mass()));
	actor->setCMassLocalPose(physx::PxTransform(as<physx::PxVec3>(part.com())));
	Eigen::Vector3d inertia_diag = part.inertia_tensor().diagonal();
	actor->setMassSpaceInertiaTensor(as<physx::PxVec3>(inertia_diag));
	env.scene->addActor(*actor);
	return actor;
}

struct BrickHandle {
	physx::PxRigidDynamic *actor{nullptr};
	physx::PxShape *hole_shape{nullptr};
	physx::PxShape *stud_shape{nullptr};
	PartId pid{};
	BrickUnit L{};
	BrickUnit W{};
	PlateUnit H{};
	BrickColor color{};

	BrickPart spec() const {
		return BrickPart(L, W, H, color);
	}

	InterfaceRef stud_ref() const {
		return {pid, BrickPart::StudId};
	}

	InterfaceRef hole_ref() const {
		return {pid, BrickPart::HoleId};
	}
};

BrickPart make_brick_spec(BrickUnit L, BrickUnit W,
                          PlateUnit H = PlateUnit{BrickHeightPerPlate}) {
	return BrickPart(L, W, H, BrickColor{0, 0, 0});
}

Transformd make_pose(double x, double y, double z) {
	return {Eigen::Quaterniond::Identity(), Eigen::Vector3d{x, y, z}};
}

ConnectionSegment cs(int x, int y, YawC4 yaw = YawC4::DEG_0) {
	return {
	    .offset = Eigen::Vector2i(x, y),
	    .yaw = yaw,
	};
}

Transformd connected_hole_pose(const BrickPart &stud_part,
                               const Transformd &stud_pose,
                               const BrickPart &hole_part,
                               const ConnectionSegment &conn_seg) {
	return stud_pose * conn_seg.compute_transform(stud_part.stud_interface(),
	                                              hole_part.hole_interface());
}

Transformd connected_stud_pose(const BrickPart &stud_part,
                               const BrickPart &hole_part,
                               const Transformd &hole_pose,
                               const ConnectionSegment &conn_seg) {
	return hole_pose *
	       inverse(conn_seg.compute_transform(stud_part.stud_interface(),
	                                          hole_part.hole_interface()));
}

struct ContactPayload {
	physx::PxContactPatch patch{};
	physx::PxReal impulses[1]{};
	physx::PxContactPair pair{};
	std::array<std::uint8_t, sizeof(physx::PxContactPairIndex) +
	                             sizeof(physx::PxContactPairPose)>
	    extra{};
	physx::PxContactPairHeader header{};
};

ContactPayload make_contact_payload(
    physx::PxRigidDynamic *actor0, physx::PxShape *shape0,
    const physx::PxTransform &pose0, physx::PxRigidDynamic *actor1,
    physx::PxShape *shape1, const physx::PxTransform &pose1,
    bool set_impulse = true,
    const physx::PxVec3 &normal = physx::PxVec3(0.0f, 0.0f, -1.0f)) {
	ContactPayload p{};

	auto *idx = reinterpret_cast<physx::PxContactPairIndex *>(p.extra.data());
	idx->type = physx::PxContactPairExtraDataType::eCONTACT_PAIR_INDEX;
	idx->index = 0;

	auto *pose_item = reinterpret_cast<physx::PxContactPairPose *>(
	    p.extra.data() + sizeof(physx::PxContactPairIndex));
	pose_item->type = physx::PxContactPairExtraDataType::eCONTACT_EVENT_POSE;
	pose_item->globalPose[0] = pose0;
	pose_item->globalPose[1] = pose1;

	p.header.actors[0] = actor0;
	p.header.actors[1] = actor1;
	p.header.extraDataStream = p.extra.data();
	p.header.extraDataStreamSize = static_cast<physx::PxU16>(p.extra.size());
	p.header.flags = {};
	p.header.nbPairs = 1;
	p.header.pairs = &p.pair;

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

struct PhysicsGraphScenarioFixture {
	PhysxEnv env;
	MetricSystem metrics{1.0, 1.0, 1.0};
	TestHooks hooks{};
	TestGraph graph;
	std::deque<BrickHandle> bricks;
	std::uint8_t next_color{1};

	PhysicsGraphScenarioFixture()
	    : env{}, metrics{1.0, 1.0, 1.0}, hooks{},
	      graph(metrics, env.physics, &hooks, {}) {
		graph.breakage_checker().thresholds.Enabled = false;
		assert(graph.bind_physx_scene(env.scene));
	}

	~PhysicsGraphScenarioFixture() {
		(void)graph.unbind_physx_scene();
		for (auto &brick : bricks) {
			if (brick.actor) {
				brick.actor->release();
				brick.actor = nullptr;
			}
		}
	}

	BrickHandle &add_brick(const Transformd &pose, BrickUnit L, BrickUnit W,
	                       PlateUnit H = PlateUnit{BrickHeightPerPlate}) {
		BrickColor color{next_color++, next_color++, next_color++};
		BrickPart part(L, W, H, color);
		physx::PxShape *hole_shape = nullptr;
		physx::PxShape *stud_shape = nullptr;
		auto *actor = create_brick_actor(
		    env, part, as<physx::PxTransform>(pose), hole_shape, stud_shape);
		std::initializer_list<InterfaceShapePair> ifs{
		    InterfaceShapePair{BrickPart::HoleId, hole_shape},
		    InterfaceShapePair{BrickPart::StudId, stud_shape},
		};
		auto pid_opt =
		    graph.topology().add_part<BrickPart>(actor, ifs, L, W, H, color);
		assert(pid_opt.has_value());
		bricks.push_back(BrickHandle{
		    .actor = actor,
		    .hole_shape = hole_shape,
		    .stud_shape = stud_shape,
		    .pid = *pid_opt,
		    .L = L,
		    .W = W,
		    .H = H,
		    .color = color,
		});
		return bricks.back();
	}

	void connect(const BrickHandle &stud, const BrickHandle &hole,
	             const ConnectionSegment &conn_seg) {
		auto csid = graph.topology().connect(stud.stud_ref(), hole.hole_ref(),
		                                     conn_seg);
		assert(csid.has_value());
	}

	void begin_step() {
		graph.do_pre_step();
		env.scene->simulate(1.0f / 60.0f);
		env.scene->fetchResults(true);
	}

	void end_step() {
		graph.do_post_step();
	}

	PxSimulationEventPatch *event_proxy() {
		auto *cb_base = env.scene->getSimulationEventCallback();
		assert(cb_base != nullptr);
		auto *proxy = dynamic_cast<PxSimulationEventPatch *>(cb_base);
		assert(proxy != nullptr);
		return proxy;
	}
};

void inject_contact_for_connection(PhysicsGraphScenarioFixture &fx,
                                   const BrickHandle &stud,
                                   const BrickHandle &hole,
                                   const ConnectionSegment &conn_seg) {
	Transformd stud_pose = SE3d{}.identity();
	Transformd hole_pose = conn_seg.compute_transform(
	    stud.spec().stud_interface(), hole.spec().hole_interface());
	ContactPayload payload = make_contact_payload(
	    stud.actor, stud.stud_shape, as<physx::PxTransform>(stud_pose),
	    hole.actor, hole.hole_shape, as<physx::PxTransform>(hole_pose),
	    /*set_impulse=*/true, physx::PxVec3(0.0f, 0.0f, -1.0f));
	fx.event_proxy()->onContact(payload.header, &payload.pair, 1);
}

void expect_connection(const PhysicsGraphScenarioFixture &fx,
                       const BrickHandle &stud, const BrickHandle &hole,
                       const ConnectionSegment &expected) {
	ConnSegRef csref{stud.stud_ref(), hole.hole_ref()};
	const ConnSegId *csid =
	    fx.graph.topology().connection_segments().find_key<ConnSegId>(csref);
	assert(csid != nullptr);
	const auto *csw =
	    fx.graph.topology().connection_segments().find_value(*csid);
	assert(csw != nullptr);
	assert(csw->wrapped().offset == expected.offset);
	assert(csw->wrapped().yaw == expected.yaw);
}

std::size_t count_assembled_event(const PhysicsGraphScenarioFixture &fx,
                                  const BrickHandle &stud,
                                  const BrickHandle &hole,
                                  const ConnectionSegment &expected) {
	ConnSegRef csref{stud.stud_ref(), hole.hole_ref()};
	return static_cast<std::size_t>(
	    std::count_if(fx.hooks.assembled_events.begin(),
	                  fx.hooks.assembled_events.end(), [&](const auto &evt) {
		                  return evt.csref == csref &&
		                         evt.conn_seg.offset == expected.offset &&
		                         evt.conn_seg.yaw == expected.yaw;
	                  }));
}

void snap_actor_to_connection(const BrickHandle &stud, BrickHandle &hole,
                              const ConnectionSegment &conn_seg) {
	Transformd T_stud = as<Transformd>(stud.actor->getGlobalPose());
	Transformd T_hole =
	    connected_hole_pose(stud.spec(), T_stud, hole.spec(), conn_seg);
	hole.actor->setGlobalPose(as<physx::PxTransform>(T_hole));
	hole.actor->setLinearVelocity(physx::PxVec3(0.0f, 0.0f, 0.0f));
	hole.actor->setAngularVelocity(physx::PxVec3(0.0f, 0.0f, 0.0f));
}

void test_same_step_stud_side_followers_are_kept() {
	PhysicsGraphScenarioFixture fx;
	const auto base_spec = make_brick_spec(BrickUnit{4}, BrickUnit{2});
	const auto small_spec = make_brick_spec(BrickUnit{2}, BrickUnit{2});

	const Transformd T_C = make_pose(0.0, 0.0, 0.0);
	auto &C = fx.add_brick(T_C, BrickUnit{4}, BrickUnit{2});
	auto &A =
	    fx.add_brick(connected_hole_pose(base_spec, T_C, small_spec, cs(0, 0)),
	                 BrickUnit{2}, BrickUnit{2});
	auto &B =
	    fx.add_brick(connected_hole_pose(base_spec, T_C, small_spec, cs(2, 0)),
	                 BrickUnit{2}, BrickUnit{2});
	auto &D =
	    fx.add_brick(make_pose(0.0, 0.25, 0.0), BrickUnit{4}, BrickUnit{2});

	fx.connect(C, A, cs(0, 0));
	fx.connect(C, B, cs(2, 0));

	const std::size_t baseline_segments =
	    fx.graph.topology().connection_segments().size();
	assert(baseline_segments == 2);

	fx.begin_step();
	inject_contact_for_connection(fx, A, D, cs(0, 0));
	inject_contact_for_connection(fx, B, D, cs(-2, 0));
	fx.end_step();

	assert(fx.graph.topology().connection_segments().size() ==
	       baseline_segments + 2);
	assert(fx.hooks.assembled_events.size() == 2);
	expect_connection(fx, A, D, cs(0, 0));
	expect_connection(fx, B, D, cs(-2, 0));
	assert(count_assembled_event(fx, A, D, cs(0, 0)) == 1);
	assert(count_assembled_event(fx, B, D, cs(-2, 0)) == 1);
}

void test_same_step_hole_side_followers_are_kept() {
	PhysicsGraphScenarioFixture fx;
	const auto small_spec = make_brick_spec(BrickUnit{2}, BrickUnit{2});
	const auto cap_spec = make_brick_spec(BrickUnit{4}, BrickUnit{2});

	auto &A =
	    fx.add_brick(make_pose(0.0, 0.0, 0.0), BrickUnit{4}, BrickUnit{2});
	const Transformd T_D = make_pose(0.0, 0.25, 0.0);
	auto &D = fx.add_brick(T_D, BrickUnit{2}, BrickUnit{2});
	Transformd T_F = connected_hole_pose(small_spec, T_D, cap_spec, cs(0, 0));
	auto &F = fx.add_brick(T_F, BrickUnit{4}, BrickUnit{2});
	Transformd T_E = connected_stud_pose(small_spec, cap_spec, T_F, cs(-2, 0));
	auto &E = fx.add_brick(T_E, BrickUnit{2}, BrickUnit{2});

	fx.connect(D, F, cs(0, 0));
	fx.connect(E, F, cs(-2, 0));

	const std::size_t baseline_segments =
	    fx.graph.topology().connection_segments().size();
	assert(baseline_segments == 2);

	fx.begin_step();
	inject_contact_for_connection(fx, A, D, cs(0, 0));
	inject_contact_for_connection(fx, A, E, cs(2, 0));
	fx.end_step();

	assert(fx.graph.topology().connection_segments().size() ==
	       baseline_segments + 2);
	assert(fx.hooks.assembled_events.size() == 2);
	expect_connection(fx, A, D, cs(0, 0));
	expect_connection(fx, A, E, cs(2, 0));
	assert(count_assembled_event(fx, A, D, cs(0, 0)) == 1);
	assert(count_assembled_event(fx, A, E, cs(2, 0)) == 1);
}

void test_same_step_many_to_many_followers_are_kept() {
	PhysicsGraphScenarioFixture fx;
	const auto small_spec = make_brick_spec(BrickUnit{2}, BrickUnit{2});
	const auto wide_spec = make_brick_spec(BrickUnit{4}, BrickUnit{2});
	const auto cap_spec = make_brick_spec(BrickUnit{6}, BrickUnit{2});

	const Transformd T_C = make_pose(0.0, 0.0, 0.0);
	auto &C = fx.add_brick(T_C, BrickUnit{6}, BrickUnit{2});
	auto &A =
	    fx.add_brick(connected_hole_pose(cap_spec, T_C, small_spec, cs(0, 0)),
	                 BrickUnit{2}, BrickUnit{2});
	auto &B =
	    fx.add_brick(connected_hole_pose(cap_spec, T_C, wide_spec, cs(2, 0)),
	                 BrickUnit{4}, BrickUnit{2});

	const Transformd T_D = make_pose(0.0, 0.25, 0.0);
	auto &D = fx.add_brick(T_D, BrickUnit{4}, BrickUnit{2});
	Transformd T_F = connected_hole_pose(wide_spec, T_D, cap_spec, cs(0, 0));
	auto &F = fx.add_brick(T_F, BrickUnit{6}, BrickUnit{2});
	Transformd T_E = connected_stud_pose(small_spec, cap_spec, T_F, cs(-4, 0));
	auto &E = fx.add_brick(T_E, BrickUnit{2}, BrickUnit{2});

	fx.connect(C, A, cs(0, 0));
	fx.connect(C, B, cs(2, 0));
	fx.connect(D, F, cs(0, 0));
	fx.connect(E, F, cs(-4, 0));

	const std::size_t baseline_segments =
	    fx.graph.topology().connection_segments().size();
	assert(baseline_segments == 4);

	fx.begin_step();
	inject_contact_for_connection(fx, A, D, cs(0, 0));
	inject_contact_for_connection(fx, B, D, cs(-2, 0));
	inject_contact_for_connection(fx, B, E, cs(2, 0));
	fx.end_step();

	assert(fx.graph.topology().connection_segments().size() ==
	       baseline_segments + 3);
	assert(fx.hooks.assembled_events.size() == 3);
	expect_connection(fx, A, D, cs(0, 0));
	expect_connection(fx, B, D, cs(-2, 0));
	expect_connection(fx, B, E, cs(2, 0));
	assert(count_assembled_event(fx, A, D, cs(0, 0)) == 1);
	assert(count_assembled_event(fx, B, D, cs(-2, 0)) == 1);
	assert(count_assembled_event(fx, B, E, cs(2, 0)) == 1);
}

void test_post_merge_closure_stud_side_followers_are_added() {
	PhysicsGraphScenarioFixture fx;
	const auto base_spec = make_brick_spec(BrickUnit{4}, BrickUnit{2});
	const auto small_spec = make_brick_spec(BrickUnit{2}, BrickUnit{2});

	const Transformd T_C = make_pose(0.0, 0.0, 0.0);
	auto &C = fx.add_brick(T_C, BrickUnit{4}, BrickUnit{2});
	auto &A =
	    fx.add_brick(connected_hole_pose(base_spec, T_C, small_spec, cs(0, 0)),
	                 BrickUnit{2}, BrickUnit{2});
	auto &B =
	    fx.add_brick(connected_hole_pose(base_spec, T_C, small_spec, cs(2, 0)),
	                 BrickUnit{2}, BrickUnit{2});
	auto &D =
	    fx.add_brick(make_pose(0.0, 0.25, 0.0), BrickUnit{4}, BrickUnit{2});

	fx.connect(C, A, cs(0, 0));
	fx.connect(C, B, cs(2, 0));

	const std::size_t baseline_segments =
	    fx.graph.topology().connection_segments().size();
	assert(baseline_segments == 2);

	fx.begin_step();
	inject_contact_for_connection(fx, A, D, cs(0, 0));
	fx.end_step();

	assert(fx.graph.topology().connection_segments().size() ==
	       baseline_segments + 2);
	assert(fx.hooks.assembled_events.size() == 2);
	expect_connection(fx, A, D, cs(0, 0));
	expect_connection(fx, B, D, cs(-2, 0));
}

void test_duplicate_seed_contacts_do_not_duplicate_closure() {
	PhysicsGraphScenarioFixture fx;
	const auto base_spec = make_brick_spec(BrickUnit{4}, BrickUnit{2});
	const auto small_spec = make_brick_spec(BrickUnit{2}, BrickUnit{2});

	const Transformd T_C = make_pose(0.0, 0.0, 0.0);
	auto &C = fx.add_brick(T_C, BrickUnit{4}, BrickUnit{2});
	auto &A =
	    fx.add_brick(connected_hole_pose(base_spec, T_C, small_spec, cs(0, 0)),
	                 BrickUnit{2}, BrickUnit{2});
	auto &B =
	    fx.add_brick(connected_hole_pose(base_spec, T_C, small_spec, cs(2, 0)),
	                 BrickUnit{2}, BrickUnit{2});
	auto &D =
	    fx.add_brick(make_pose(0.0, 0.25, 0.0), BrickUnit{4}, BrickUnit{2});

	fx.connect(C, A, cs(0, 0));
	fx.connect(C, B, cs(2, 0));

	const std::size_t baseline_segments =
	    fx.graph.topology().connection_segments().size();
	assert(baseline_segments == 2);

	fx.begin_step();
	inject_contact_for_connection(fx, A, D, cs(0, 0));
	inject_contact_for_connection(fx, A, D, cs(0, 0));
	fx.end_step();

	assert(fx.graph.topology().connection_segments().size() ==
	       baseline_segments + 2);
	assert(fx.hooks.assembled_events.size() == 2);
	expect_connection(fx, A, D, cs(0, 0));
	expect_connection(fx, B, D, cs(-2, 0));
	assert(count_assembled_event(fx, A, D, cs(0, 0)) == 1);
	assert(count_assembled_event(fx, B, D, cs(-2, 0)) == 1);
}

void test_post_merge_closure_hole_side_followers_are_added() {
	PhysicsGraphScenarioFixture fx;
	const auto small_spec = make_brick_spec(BrickUnit{2}, BrickUnit{2});
	const auto cap_spec = make_brick_spec(BrickUnit{4}, BrickUnit{2});

	auto &A =
	    fx.add_brick(make_pose(0.0, 0.0, 0.0), BrickUnit{4}, BrickUnit{2});
	const Transformd T_D = make_pose(0.0, 0.25, 0.0);
	auto &D = fx.add_brick(T_D, BrickUnit{2}, BrickUnit{2});
	Transformd T_F = connected_hole_pose(small_spec, T_D, cap_spec, cs(0, 0));
	auto &F = fx.add_brick(T_F, BrickUnit{4}, BrickUnit{2});
	Transformd T_E = connected_stud_pose(small_spec, cap_spec, T_F, cs(-2, 0));
	auto &E = fx.add_brick(T_E, BrickUnit{2}, BrickUnit{2});

	fx.connect(D, F, cs(0, 0));
	fx.connect(E, F, cs(-2, 0));

	const std::size_t baseline_segments =
	    fx.graph.topology().connection_segments().size();
	assert(baseline_segments == 2);

	fx.begin_step();
	inject_contact_for_connection(fx, A, D, cs(0, 0));
	fx.end_step();

	assert(fx.graph.topology().connection_segments().size() ==
	       baseline_segments + 2);
	assert(fx.hooks.assembled_events.size() == 2);
	expect_connection(fx, A, D, cs(0, 0));
	expect_connection(fx, A, E, cs(2, 0));
}

void test_post_merge_closure_many_to_many_followers_are_added() {
	PhysicsGraphScenarioFixture fx;
	const auto small_spec = make_brick_spec(BrickUnit{2}, BrickUnit{2});
	const auto wide_spec = make_brick_spec(BrickUnit{4}, BrickUnit{2});
	const auto cap_spec = make_brick_spec(BrickUnit{6}, BrickUnit{2});

	const Transformd T_C = make_pose(0.0, 0.0, 0.0);
	auto &C = fx.add_brick(T_C, BrickUnit{6}, BrickUnit{2});
	auto &A =
	    fx.add_brick(connected_hole_pose(cap_spec, T_C, small_spec, cs(0, 0)),
	                 BrickUnit{2}, BrickUnit{2});
	auto &B =
	    fx.add_brick(connected_hole_pose(cap_spec, T_C, wide_spec, cs(2, 0)),
	                 BrickUnit{4}, BrickUnit{2});

	const Transformd T_D = make_pose(0.0, 0.25, 0.0);
	auto &D = fx.add_brick(T_D, BrickUnit{4}, BrickUnit{2});
	Transformd T_F = connected_hole_pose(wide_spec, T_D, cap_spec, cs(0, 0));
	auto &F = fx.add_brick(T_F, BrickUnit{6}, BrickUnit{2});
	Transformd T_E = connected_stud_pose(small_spec, cap_spec, T_F, cs(-4, 0));
	auto &E = fx.add_brick(T_E, BrickUnit{2}, BrickUnit{2});

	fx.connect(C, A, cs(0, 0));
	fx.connect(C, B, cs(2, 0));
	fx.connect(D, F, cs(0, 0));
	fx.connect(E, F, cs(-4, 0));

	const std::size_t baseline_segments =
	    fx.graph.topology().connection_segments().size();
	assert(baseline_segments == 4);

	fx.begin_step();
	inject_contact_for_connection(fx, A, D, cs(0, 0));
	fx.end_step();

	assert(fx.graph.topology().connection_segments().size() ==
	       baseline_segments + 3);
	assert(fx.hooks.assembled_events.size() == 3);
	expect_connection(fx, A, D, cs(0, 0));
	expect_connection(fx, B, D, cs(-2, 0));
	expect_connection(fx, B, E, cs(2, 0));
}

void test_late_follower_contact_after_merge_is_recovered() {
	PhysicsGraphScenarioFixture fx;
	const auto base_spec = make_brick_spec(BrickUnit{4}, BrickUnit{2});
	const auto small_spec = make_brick_spec(BrickUnit{2}, BrickUnit{2});

	const Transformd T_C = make_pose(0.0, 0.0, 0.0);
	auto &C = fx.add_brick(T_C, BrickUnit{4}, BrickUnit{2});
	auto &A =
	    fx.add_brick(connected_hole_pose(base_spec, T_C, small_spec, cs(0, 0)),
	                 BrickUnit{2}, BrickUnit{2});
	auto &B =
	    fx.add_brick(connected_hole_pose(base_spec, T_C, small_spec, cs(2, 0)),
	                 BrickUnit{2}, BrickUnit{2});
	auto &D =
	    fx.add_brick(make_pose(0.0, 0.25, 0.0), BrickUnit{4}, BrickUnit{2});

	fx.connect(C, A, cs(0, 0));
	fx.connect(C, B, cs(2, 0));

	const std::size_t baseline_segments =
	    fx.graph.topology().connection_segments().size();
	assert(baseline_segments == 2);

	fx.begin_step();
	inject_contact_for_connection(fx, A, D, cs(0, 0));
	fx.end_step();
	snap_actor_to_connection(A, D, cs(0, 0));

	fx.begin_step();
	inject_contact_for_connection(fx, B, D, cs(-2, 0));
	fx.end_step();

	assert(fx.graph.topology().connection_segments().size() ==
	       baseline_segments + 2);
	assert(fx.hooks.assembled_events.size() == 2);
	expect_connection(fx, A, D, cs(0, 0));
	expect_connection(fx, B, D, cs(-2, 0));
}

int main() {
	test_same_step_stud_side_followers_are_kept();
	test_same_step_hole_side_followers_are_kept();
	test_same_step_many_to_many_followers_are_kept();
	test_post_merge_closure_stud_side_followers_are_added();
	test_duplicate_seed_contacts_do_not_duplicate_closure();
	test_post_merge_closure_hole_side_followers_are_added();
	test_post_merge_closure_many_to_many_followers_are_added();
	test_late_follower_contact_after_merge_is_recovered();
	return 0;
}
