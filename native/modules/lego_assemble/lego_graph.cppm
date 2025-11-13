export module lego_assemble.lego_graph;

import std;
import lego_assemble.brick_specs;
import lego_assemble.physx.scene_patcher;
import lego_assemble.physx.weld_constraint;
import lego_assemble.vendor.carb;
import lego_assemble.vendor.eigen;
import lego_assemble.vendor.physx;
import lego_assemble.utils.hash;
import lego_assemble.utils.conversions;
import lego_assemble.utils.pair;
import lego_assemble.utils.skip_graph;
import lego_assemble.utils.unordered_pair;

namespace lego_assemble {

export class LegoGraph {
  public:
	struct Config {
		double mpu;
		double kpu;
	};
	struct Thresholds {
		// Maximum distance between bricks (m)
		double DistanceTolerance = 0.001;

		// Maximum penetration between bricks (m), penetration can happen due to simulation inaccuracies
		double MaxPenetration = 0.005;

		// Maximum angle between z-axis of bricks (rad)
		double ZAngleTolerance = 5.0 * (std::numbers::pi / 180.0);

		// Minimum clutch power (N)
		double RequiredForce = 1.0;

		// Maximum yaw error (rad)
		double YawTolerance = 5.0 * (std::numbers::pi / 180.0);

		// Maximum position error (m)
		double PositionTolerance = 0.002;
	};
	struct BrickInfo {
		std::array<BrickUnit, 3> dimensions;
		physx::PxShape *body_collider;
		physx::PxShape *top_collider;
	};
	struct ConnInfo {
		physx::PxTransform T_parent_local;
		physx::PxTransform T_child_local;
		std::array<float, 2> overlap_xy;
	};
	struct AssemblyEvent {
		physx::PxRigidActor *parent;
		physx::PxRigidActor *child;
		std::array<BrickUnit, 2> offset_studs;
		BrickOrientation orientation;
		physx::PxTransform T_parent_local;
		physx::PxTransform T_child_local;
		std::array<float, 2> overlap_xy;
	};

	explicit LegoGraph(physx::PxPhysics *px, const Config &config);
	LegoGraph(const LegoGraph &) = delete;
	LegoGraph &operator=(const LegoGraph &) = delete;
	~LegoGraph();

	bool addRigidBody(physx::PxRigidActor *actor, const BrickInfo &info);
	bool removeRigidBody(physx::PxRigidActor *actor);
	bool connect(physx::PxRigidActor *a, physx::PxRigidActor *b,
	             const ConnInfo &info);
	bool disconnect(physx::PxRigidActor *a, physx::PxRigidActor *b);
	std::vector<std::pair<physx::PxRigidActor *, physx::PxRigidActor *>>
	solveLimits();
	std::vector<AssemblyEvent> pollAssemblyEvents();
	void clear();
	void getThresholds(Thresholds &out) const;
	void setThresholds(const Thresholds &in);

  private:
	class Impl;
	std::unique_ptr<Impl> impl_;
};

class LegoGraph::Impl {
  public:
	struct BodyDesc;
	struct ConnDesc;
	using RigidBody = physx::PxRigidActor;
	using Constraint = physx::PxConstraint;
	using Scheduler = SimpleSkipGraphScheduler<
	    BodyDesc *, Constraint *,
	    std::function<Constraint *(BodyDesc *, BodyDesc *)>,
	    std::function<void(Constraint *)>>;
	using Transform = physx::PxTransform;
	using BodyMap = std::unordered_map<RigidBody *, BodyDesc>;
	using ConnMap =
	    std::unordered_map<std::pair<RigidBody *, RigidBody *>, ConnDesc,
	                       PairHash<RigidBody *>, PairEq<RigidBody *>>;

	struct BodyDesc {
		RigidBody *actor;
		std::unordered_set<ConnDesc *> parents;  // this is child
		std::unordered_set<ConnDesc *> children; // this is parent
		BrickInfo info;
	};
	struct ConnDesc {
		Constraint *joint;
		BodyDesc *parent;
		BodyDesc *child;
		Transform tf;
		Transform T_parent_local;
		Transform T_child_local;
		std::array<float, 2> overlap_xy; // in stage units
	};

	class LegoSimulationFilterCallback
	    : public PxSimulationFilterCallbackProxy {
	  public:
		LegoGraph::Impl *impl;
		explicit LegoSimulationFilterCallback(LegoGraph::Impl *i) : impl(i) {}
		LegoSimulationFilterCallback(const LegoSimulationFilterCallback &) =
		    delete;
		LegoSimulationFilterCallback &
		operator=(const LegoSimulationFilterCallback &) = delete;

		virtual physx::PxFilterFlags pairFound(
		    physx::PxU64 pairID, physx::PxFilterObjectAttributes attributes0,
		    physx::PxFilterData filterData0, const physx::PxActor *a0,
		    const physx::PxShape *s0,
		    physx::PxFilterObjectAttributes attributes1,
		    physx::PxFilterData filterData1, const physx::PxActor *a1,
		    const physx::PxShape *s1, physx::PxPairFlags &pairFlags) override {
			auto result = PxSimulationFilterCallbackProxy::pairFound(
			    pairID, attributes0, filterData0, a0, s0, attributes1,
			    filterData1, a1, s1, pairFlags);
			if (impl->contactExclusions.contains({a0, a1})) {
				result = physx::PxFilterFlag::eKILL;
			} else {
				BodyDesc *body0 = nullptr;
				BodyDesc *body1 = nullptr;
				if (impl->resolveActor(a0, body0) &&
				    impl->resolveActor(a1, body1)) {
					if ((s0 == body0->info.top_collider &&
					     s1 == body1->info.body_collider) ||
					    (s0 == body0->info.body_collider &&
					     s1 == body1->info.top_collider)) {
						pairFlags |= physx::PxPairFlag::eCONTACT_EVENT_POSE;
					}
				}
			}
			return result;
		}
	};

	class LegoSimulationEventCallback : public PxSimulationEventCallbackProxy {
	  public:
		LegoGraph::Impl *impl;
		explicit LegoSimulationEventCallback(LegoGraph::Impl *i) : impl(i) {}
		LegoSimulationEventCallback(const LegoSimulationEventCallback &) =
		    delete;
		LegoSimulationEventCallback &
		operator=(const LegoSimulationEventCallback &) = delete;

		virtual void onContact(const physx::PxContactPairHeader &header,
		                       const physx::PxContactPair *pairs,
		                       physx::PxU32 nbPairs) {
			PxSimulationEventCallbackProxy::onContact(header, pairs, nbPairs);

			if (header.flags.isSet(
			        physx::PxContactPairHeaderFlag::eREMOVED_ACTOR_0) ||
			    header.flags.isSet(
			        physx::PxContactPairHeaderFlag::eREMOVED_ACTOR_1)) {
				return;
			}
			BodyDesc *body0 = nullptr;
			BodyDesc *body1 = nullptr;
			if (!impl->resolveActor(header.actors[0], body0) ||
			    !impl->resolveActor(header.actors[1], body1)) {
				return;
			}
			// Lookup global poses
			physx::PxContactPairExtraDataIterator extraIt(
			    header.extraDataStream, header.extraDataStreamSize);
			const physx::PxContactPairPose *eventPose = nullptr;
			bool extraHasNext = extraIt.nextItemSet();
			// Iterate over contact pairs
			for (physx::PxU32 i = 0; i < nbPairs; i++) {
				while (extraHasNext && extraIt.contactPairIndex <= i) {
					eventPose = extraIt.eventPose;
					extraHasNext = extraIt.nextItemSet();
				}
				if (eventPose == nullptr) {
					continue;
				}
				const auto &pair = pairs[i];
				if (!pair.flags.isSet(
				        physx::PxContactPairFlag::eINTERNAL_HAS_IMPULSES)) {
					continue;
				}
				if (!pair.contactPatches) {
					continue;
				}
				const auto &[shape0, shape1] = pair.shapes;
				// body0 should be the lower brick, offering TopCollider
				// body1 should be the upper brick, offering BodyCollider
				bool toSwap = false;
				if (shape0 == body0->info.top_collider &&
				    shape1 == body1->info.body_collider) {
				} else if (shape0 == body0->info.body_collider &&
				           shape1 == body1->info.top_collider) {
					toSwap = true;
				} else {
					continue;
				}
				// Sum up contact impulses
				physx::PxVec3 totalImpulse(0, 0, 0);
				const auto *patches =
				    reinterpret_cast<const physx::PxContactPatch *>(
				        pair.contactPatches);
				for (physx::PxU32 j = 0; j < pair.patchCount; j++) {
					const auto &patch = patches[j];
					physx::PxReal patchImpulse = 0;
					for (physx::PxU32 k = 0; k < patch.nbContacts; k++) {
						patchImpulse +=
						    pair.contactImpulses[patch.startContactIndex + k];
					}
					totalImpulse += patch.normal * patchImpulse;
				}
				const auto &[pose0, pose1] = eventPose->globalPose;
				if (toSwap) {
					impl->processAssemblyContact(body1, pose1, body0, pose0,
					                             -totalImpulse);
				} else {
					impl->processAssemblyContact(body0, pose0, body1, pose1,
					                             totalImpulse);
				}
			}
		}
	};

	physx::PxPhysics *px;
	Config cfg;
	Thresholds thresholds;
	BodyMap bodies;
	ConnMap conns;
	Scheduler scheduler;
	std::unique_ptr<LegoSimulationFilterCallback> simFilter;
	std::unique_ptr<LegoSimulationEventCallback> simCallback;
	std::unordered_set<UnorderedPair<const physx::PxActor *>> contactExclusions;
	std::vector<AssemblyEvent> assemblyEvents;

	explicit Impl(physx::PxPhysics *px_, const Config &cfg_)
	    : px(px_), cfg(cfg_),
	      scheduler(std::bind(&Impl::createAuxConstraint, this,
	                          std::placeholders::_1, std::placeholders::_2),
	                std::bind(&Impl::destroyAuxConstraint, this,
	                          std::placeholders::_1)) {
		simFilter = std::make_unique<LegoSimulationFilterCallback>(this);
		if (!setPxSimulationFilterCallback(simFilter.get())) {
			simFilter = nullptr;
		}
		simCallback = std::make_unique<LegoSimulationEventCallback>(this);
		if (!setPxSimulationEventCallback(simCallback.get())) {
			simCallback = nullptr;
		}
	}

	Impl(const Impl &) = delete;
	Impl &operator=(const Impl &) = delete;
	~Impl() {
		clear();
		if (simCallback) {
			clearPxSimulationEventCallback();
			simCallback = nullptr;
		}
		if (simFilter) {
			clearPxSimulationFilterCallback();
			simFilter = nullptr;
		}
	}

	bool addRigidBody(RigidBody *actor, const BrickInfo &info) {
		if (bodies.contains(actor)) {
			return false;
		}
		BodyDesc desc;
		desc.actor = actor;
		desc.info = info;
		bodies[actor] = desc;
		return true;
	}

	bool removeRigidBody(RigidBody *actor) {
		auto it = bodies.find(actor);
		if (it == bodies.end()) {
			return false;
		}
		BodyDesc &desc = it->second;
		for (ConnDesc *c : desc.parents) {
			if (!c->parent->children.erase(c)) {
				log_error(
				    "Inconsistent state when removing body {:p}, continuing",
				    static_cast<const void *>(actor));
			}
			releaseConn(*c);
			if (!scheduler.disconnect(c->parent, c->child)) {
				log_error("Failed to remove edge between {:p} and {:p} from "
				          "scheduler, continuing",
				          static_cast<const void *>(c->parent->actor),
				          static_cast<const void *>(c->child->actor));
			}
			auto edge_key = std::make_pair(c->parent->actor, c->child->actor);
			if (!conns.erase(edge_key)) {
				log_error("Failed to remove connection [{:p}, {:p}] from map, "
				          "continuing",
				          static_cast<const void *>(c->parent->actor),
				          static_cast<const void *>(c->child->actor));
			}
		}
		for (ConnDesc *c : desc.children) {
			if (!c->child->parents.erase(c)) {
				log_error(
				    "Inconsistent state when removing body {:p}, continuing",
				    static_cast<const void *>(actor));
			}
			releaseConn(*c);
			if (!scheduler.disconnect(c->parent, c->child)) {
				log_error("Failed to remove edge between {:p} and {:p} from "
				          "scheduler, continuing",
				          static_cast<const void *>(c->parent->actor),
				          static_cast<const void *>(c->child->actor));
			}
			auto edge_key = std::make_pair(c->parent->actor, c->child->actor);
			if (!conns.erase(edge_key)) {
				log_error("Failed to remove connection [{:p}, {:p}] from map, "
				          "continuing",
				          static_cast<const void *>(c->parent->actor),
				          static_cast<const void *>(c->child->actor));
			}
		}
		bodies.erase(it);
		return true;
	}

	bool connect(RigidBody *a, RigidBody *b, const ConnInfo &info) {
		if (a == b) {
			return false;
		}
		auto ita = bodies.find(a);
		auto itb = bodies.find(b);
		if (ita == bodies.end() || itb == bodies.end()) {
			return false;
		}
		BodyDesc *da = &ita->second;
		BodyDesc *db = &itb->second;
		if (findConnBidirectional(a, b) != conns.end()) {
			return false;
		}

		ConnDesc c;
		c.parent = da;
		c.child = db;
		c.tf = info.T_parent_local * info.T_child_local.getInverse();
		c.T_parent_local = info.T_parent_local;
		c.T_child_local = info.T_child_local;
		c.overlap_xy = info.overlap_xy;
		setupConn(c);
		auto edge_key = std::make_pair(a, b);
		conns[edge_key] = c;
		da->children.insert(&conns[edge_key]);
		db->parents.insert(&conns[edge_key]);
		if (!scheduler.connect(da, db)) {
			log_error("Failed to add edge between {:p} and {:p} to scheduler, "
			          "continuing",
			          static_cast<const void *>(a),
			          static_cast<const void *>(b));
		}
		b->getScene()->resetFiltering(*b);
		log_info("Created base constraint between {:p} and {:p}",
		         static_cast<const void *>(a), static_cast<const void *>(b));
		return true;
	}

	bool disconnect(RigidBody *a, RigidBody *b) {
		if (a == b) {
			return false;
		}
		auto ita = bodies.find(a);
		auto itb = bodies.find(b);
		if (ita == bodies.end() || itb == bodies.end()) {
			return false;
		}
		auto itc = findConnBidirectional(a, b);
		if (itc == conns.end()) {
			return false;
		}

		ConnDesc &c = itc->second;
		c.parent->children.erase(&c);
		c.child->parents.erase(&c);
		releaseConn(c);
		if (!scheduler.disconnect(c.parent, c.child)) {
			log_error("Failed to remove edge between {:p} and {:p} from "
			          "scheduler, continuing",
			          static_cast<const void *>(c.parent->actor),
			          static_cast<const void *>(c.child->actor));
		}
		c.child->actor->getScene()->resetFiltering(*c.child->actor);
		log_info("Destroyed base constraint between {:p} and {:p}",
		         static_cast<const void *>(c.parent->actor),
		         static_cast<const void *>(c.child->actor));
		conns.erase(itc);
		return true;
	}

	void clear() {
		scheduler.clear();
		for (auto &[_, c] : conns) {
			releaseConn(c);
		}
		conns.clear();
		bodies.clear();
	}

	std::vector<ConnDesc *> solveLimits() {
		std::vector<ConnDesc *> violated;

		// ---- Config (tune as needed) ----
		constexpr double kTolParallel =
		    0.999; // z-normal parallel threshold cos(angle)
		constexpr double kTolZero = 1e-12; // numerical zero
		constexpr double kViolTol = 1e-6;  // tolerance for envelope violation
		constexpr double kKlinScale = 1.0; // linear stiffness scale
		constexpr double kKrotScale = 1.0; // bending stiffness scale
		constexpr float kClutchPerArea =
		    95000.0f; // N per (stage-unit^2); TODO: configure

		// ---- Early out: nothing to do if no base connections ----
		if (conns.empty())
			return violated;

		auto getConstraintBetween = [](RigidBody *A,
		                               RigidBody *B) -> Constraint * {
			if (!A || !B)
				return nullptr;
			physx::PxU32 n = A->getNbConstraints();
			if (n == 0)
				return nullptr;
			std::vector<Constraint *> buf(n);
			A->getConstraints(buf.data(), n);
			for (Constraint *c : buf) {
				RigidBody *a0 = nullptr;
				RigidBody *a1 = nullptr;
				c->getActors(a0, a1);
				if ((a0 == A && a1 == B) || (a0 == B && a1 == A)) {
					return c;
				}
			}
			return nullptr;
		};

		auto canonPair = [](BodyDesc *a, BodyDesc *b) {
			return std::make_pair(std::min(a, b), std::max(a, b));
		};

		// ---- 1) Index nodes (only those participating in base edges) ----
		// Build unique node set from base connections
		std::unordered_map<BodyDesc *, int> nodeIx;
		std::vector<BodyDesc *> nodeRev;
		nodeRev.reserve(conns.size() * 2);
		for (auto &kv : conns) {
			ConnDesc &c = kv.second;
			if (!nodeIx.count(c.parent)) {
				nodeIx[c.parent] = int(nodeRev.size());
				nodeRev.push_back(c.parent);
			}
			if (!nodeIx.count(c.child)) {
				nodeIx[c.child] = int(nodeRev.size());
				nodeRev.push_back(c.child);
			}
		}
		const int n = int(nodeRev.size());

		// ---- 2) Build base-edge list with geometry & stiffness ----
		struct BaseEdge {
			int i, j;             // node indices (parent -> child)
			ConnDesc *conn;       // pointer back to connection
			double a, b;          // half sizes [m] (stage units)
			double N0;            // clutch capacity
			double Kfz, Ktx, Kty; // scalar stiffnesses
			double phi;           // in-plane yaw wrt component basis (rad)
			Eigen::Matrix2d
			    KxyGlob; // rotated 2x2 bending stiffness in component basis
			Eigen::Vector3d z_world; // interface normal (graph-world)
		};
		std::vector<BaseEdge> E;
		E.reserve(conns.size());
		// We'll also build an adjacency for CC detection
		std::vector<std::vector<int>> adj(n);

		// ---- Pick a root per connected component (we'll fill later). For now pick a global root candidate. ----
		BodyDesc *rootBody = nodeRev[0];
		auto Rroot_world =
		    as<Eigen::Matrix3d>(rootBody->actor->getGlobalPose().q);

		// ---- Build graph-world rotations Rgw for every node using your graph transforms ----
		// We do a BFS from the chosen rootBody over the base graph using your lookupGraphTransform_
		std::vector<Eigen::Matrix3d> Rgw(n, Eigen::Matrix3d::Identity());
		{
			// Fill adjacency from base connections for the BFS of rotations
			std::vector<std::vector<int>> g(n);
			for (auto &kv : conns) {
				ConnDesc &c = kv.second;
				int ip = nodeIx[c.parent];
				int ic = nodeIx[c.child];
				g[ip].push_back(ic);
				g[ic].push_back(ip);
			}
			// BFS to compute Rgw via your lookupGraphTransform_
			std::vector<char> vis(n, 0);
			std::deque<int> dq;
			int root = 0;
			Rgw[root] = Rroot_world;
			dq.push_back(root);
			vis[root] = 1;
			while (!dq.empty()) {
				int u = dq.front();
				dq.pop_front();
				for (int v : g[u]) {
					if (vis[v])
						continue;
					physx::PxTransform T_u_v;
					bool ok =
					    lookupGraphTransform(nodeRev[u], nodeRev[v], T_u_v);
					if (!ok) {
						// If disconnected by mistake, keep identity; but this shouldn't happen inside a CC.
						Rgw[v] = Rgw[u];
					} else {
						auto R_uv = as<Eigen::Matrix3d>(T_u_v.q);
						Rgw[v] = Rgw[u] * R_uv; // compose along base graph
					}
					vis[v] = 1;
					dq.push_back(v);
				}
			}
		}

		// Component canonical basis from root (graph-world)
		const Eigen::Vector3d x_star =
		    Rgw[nodeIx[rootBody]] * Eigen::Vector3d(1, 0, 0);
		const Eigen::Vector3d y_star =
		    Rgw[nodeIx[rootBody]] * Eigen::Vector3d(0, 1, 0);
		const Eigen::Vector3d z_star =
		    Rgw[nodeIx[rootBody]] * Eigen::Vector3d(0, 0, 1);

		// Fill BaseEdge array
		for (auto &kv : conns) {
			ConnDesc &c = kv.second;
			int ip = nodeIx[c.parent];
			int ic = nodeIx[c.child];

			// geometry: interpret overlap_xy as FULL extents in x,y (stage units)
			double w = std::max(1e-9f, c.overlap_xy[0]); // length along x
			double h = std::max(1e-9f, c.overlap_xy[1]); // length along y
			double a = 0.5 * w;
			double b = 0.5 * h;
			double area = w * h;

			// capacity (very simple model): clutch per area
			double N0 = double(kClutchPerArea) * area;

			// stiffness proxies (relative are enough)
			// linear normal ~ area
			double Kfz = kKlinScale * area;
			// bending about x (across y-span) uses I_y = 4/3 * a^3 b
			double Ktx = kKrotScale * (4.0 / 3.0) * (a * a * a) * b;
			// bending about y (across x-span) uses I_x = 4/3 * a b^3
			double Kty = kKrotScale * (4.0 / 3.0) * a * (b * b * b);

			// interface frame in parent local: use T_parent_local rotation
			Eigen::Matrix3d R_parent_gw = Rgw[ip];
			auto R_joint_parentLocal = as<Eigen::Matrix3d>(c.T_parent_local.q);
			Eigen::Matrix3d R_joint_world = R_parent_gw * R_joint_parentLocal;

			Eigen::Vector3d z_world =
			    (R_joint_world * Eigen::Vector3d(0, 0, 1)).normalized();
			// Ensure normal points in same general direction as component normal
			double cz = z_world.dot(z_star);
			if (cz < 0.0) {
				z_world = -z_world;
			} // flip to align

			// in-plane yaw phi relative to component (x_star,y_star)
			Eigen::Vector3d x_world = R_joint_world * Eigen::Vector3d(1, 0, 0);
			Eigen::Vector3d x_par = (x_world - (x_world.dot(z_star)) * z_star);
			if (x_par.norm() < 1e-12)
				x_par = x_star;
			else
				x_par.normalize();
			double phi = std::atan2(x_par.dot(y_star), x_par.dot(x_star));

			// build rotated bending stiffness in component basis
			Eigen::Matrix2d Rphi;
			Rphi << std::cos(phi), -std::sin(phi), std::sin(phi), std::cos(phi);
			Eigen::Matrix2d Kloc;
			Kloc.setZero();
			Kloc(0, 0) = Ktx;
			Kloc(1, 1) = Kty;
			Eigen::Matrix2d Kglob = Rphi * Kloc * Rphi.transpose();

			BaseEdge be;
			be.i = ip;
			be.j = ic;
			be.conn = &c;
			be.a = a;
			be.b = b;
			be.N0 = N0;
			be.Kfz = Kfz;
			be.Ktx = Ktx;
			be.Kty = Kty;
			be.phi = phi;
			be.KxyGlob = Kglob;
			be.z_world = z_world;
			E.push_back(std::move(be));

			adj[ip].push_back(ic);
			adj[ic].push_back(ip);
		}
		const int m = int(E.size());

		// ---- 3) Check normals roughly parallel (Level-1 assumption) ----
		for (auto const &be : E) {
			double c = be.z_world.dot(z_star);
			if (c < kTolParallel) {
				// Non-parallel normals: for this single-shot implementation, we conservatively return "no violations computed".
				// In production: fallback to LP as discussed (HiGHS).
				log_warn("solveLimits(): non-parallel joint normals detected; "
				         "fallback not implemented in this one-shot version.");
				return violated;
			}
		}

		// ---- 4) Assemble Laplacians (scalar for Fz, 2x2 block for bending) ----
		// Lz: n x n
		std::vector<Eigen::Triplet<double>> tripsLz;
		tripsLz.reserve(4 * m);
		for (auto const &e : E) {
			int i = e.i, j = e.j;
			double k = e.Kfz;
			tripsLz.emplace_back(i, i, +k);
			tripsLz.emplace_back(j, j, +k);
			tripsLz.emplace_back(i, j, -k);
			tripsLz.emplace_back(j, i, -k);
		}
		Eigen::SparseMatrix<double> Lz(n, n);
		Lz.setFromTriplets(tripsLz.begin(), tripsLz.end());

		// Lbend: (2n) x (2n)
		auto idx2 = [](int v, int comp) {
			return 2 * v + comp;
		}; // comp: 0 for x, 1 for y
		std::vector<Eigen::Triplet<double>> tripsLb;
		tripsLb.reserve(16 * m); // each edge contributes four 2x2 blocks
		for (auto const &e : E) {
			int i = e.i, j = e.j;
			const Eigen::Matrix2d &K = e.KxyGlob;
			// +K at (i,i), (j,j); -K at (i,j), (j,i)
			for (int r = 0; r < 2; ++r)
				for (int c = 0; c < 2; ++c) {
					tripsLb.emplace_back(idx2(i, r), idx2(i, c), +K(r, c));
					tripsLb.emplace_back(idx2(j, r), idx2(j, c), +K(r, c));
					tripsLb.emplace_back(idx2(i, r), idx2(j, c), -K(r, c));
					tripsLb.emplace_back(idx2(j, r), idx2(i, c), -K(r, c));
				}
		}
		Eigen::SparseMatrix<double> Lb(2 * n, 2 * n);
		Lb.setFromTriplets(tripsLb.begin(), tripsLb.end());

		// ---- 5) Choose anchors (one node per CC) and build selection matrix S to remove anchors ----
		// Find CCs:
		std::vector<int> comp(n, -1);
		int ncc = 0;
		for (int s = 0; s < n; ++s)
			if (comp[s] < 0) {
				std::deque<int> dq;
				dq.push_back(s);
				comp[s] = ncc;
				while (!dq.empty()) {
					int u = dq.front();
					dq.pop_front();
					for (int v : adj[u])
						if (comp[v] < 0) {
							comp[v] = ncc;
							dq.push_back(v);
						}
				}
				++ncc;
			}
		// pick first node as anchor for each cc
		std::vector<int> anchorOfCC(ncc, -1);
		for (int i = 0; i < n; ++i)
			if (anchorOfCC[comp[i]] < 0)
				anchorOfCC[comp[i]] = i;

		// Build keep-lists
		std::vector<int> keepNodes;
		keepNodes.reserve(n - ncc);
		std::vector<char> isAnchor(n, 0);
		for (int cci = 0; cci < ncc; ++cci) {
			isAnchor[anchorOfCC[cci]] = 1;
		}
		for (int i = 0; i < n; ++i)
			if (!isAnchor[i])
				keepNodes.push_back(i);

		auto buildSelector = [](const std::vector<int> &keep,
		                        int N) -> Eigen::SparseMatrix<double> {
			std::vector<Eigen::Triplet<double>> t;
			t.reserve(keep.size());
			for (int r = 0; r < (int)keep.size(); ++r)
				t.emplace_back(r, keep[r], 1.0);
			Eigen::SparseMatrix<double> S(keep.size(), N);
			S.setFromTriplets(t.begin(), t.end());
			return S;
		};
		Eigen::SparseMatrix<double> Sz = buildSelector(keepNodes, n);

		// For bending: keep both x/y rows for non-anchor nodes
		std::vector<int> keepB;
		keepB.reserve(2 * (n - ncc));
		for (int i = 0; i < n; ++i)
			if (!isAnchor[i]) {
				keepB.push_back(2 * i + 0);
				keepB.push_back(2 * i + 1);
			}
		auto buildSelector2 = [](const std::vector<int> &keep,
		                         int N) -> Eigen::SparseMatrix<double> {
			std::vector<Eigen::Triplet<double>> t;
			t.reserve(keep.size());
			for (int r = 0; r < (int)keep.size(); ++r)
				t.emplace_back(r, keep[r], 1.0);
			Eigen::SparseMatrix<double> S(keep.size(), N);
			S.setFromTriplets(t.begin(), t.end());
			return S;
		};
		Eigen::SparseMatrix<double> Sb = buildSelector2(keepB, 2 * n);

		// Reduced systems
		Eigen::SparseMatrix<double> Lz_red = Sz * Lz * Sz.transpose();
		Eigen::SparseMatrix<double> Lb_red = Sb * Lb * Sb.transpose();

		// ---- 6) Assemble RHS from aux constraints (skip-graph): bFz (n), bT (2n) ----
		Eigen::VectorXd bFz = Eigen::VectorXd::Zero(n);
		Eigen::VectorXd bT = Eigen::VectorXd::Zero(2 * n);

		// We need the component canonical basis; use the root's world rotation to project world wrenches.
		Eigen::Matrix3d Rcw =
		    Rroot_world.transpose(); // world -> component canonical basis

		// The scheduler can tell us which aux pairs exist; we need to find the actual PxConstraint* for each pair to read forces
		for (auto const &aux : scheduler.aux_edges()) {
			BodyDesc *A = aux.first;
			BodyDesc *B = aux.second;
			// Ignore if either endpoint not in base-node set (we solve per base CC)
			if (!nodeIx.count(A) || !nodeIx.count(B))
				continue;

			Constraint *c = getConstraintBetween(A->actor, B->actor);
			if (!c)
				continue; // robust to transient conditions

			// Read force/torque (PhysX returns wrench on actor0)
			physx::PxVec3 Fw, Mw;
			c->getForce(Fw, Mw);

			// Determine actor0/actor1 mapping to nodes
			RigidBody *a0 = nullptr;
			RigidBody *a1 = nullptr;
			c->getActors(a0, a1);
			int i = nodeIx[A], j = nodeIx[B];
			// but constraint may have actor order swapped:
			int src = -1, dst = -1;
			if (a0 == A->actor && a1 == B->actor) {
				src = i;
				dst = j;
			} else if (a0 == B->actor && a1 == A->actor) {
				src = j;
				dst = i;
			} else {
				// Shouldn't happen; fall back to A->B as src->dst
				src = i;
				dst = j;
			}

			Eigen::Vector3d Fw_e(Fw.x, Fw.y, Fw.z);
			Eigen::Vector3d Mw_e(Mw.x, Mw.y, Mw.z);
			Eigen::Vector3d Fcmp = Rcw * Fw_e;
			Eigen::Vector3d Mcmp = Rcw * Mw_e;

			// Accumulate node divergences (balance): + at src, - at dst
			bFz[src] += Fcmp.z();
			bFz[dst] -= Fcmp.z();
			// Bending demand as 2-vector (x,y)
			bT[2 * src + 0] += Mcmp.x();
			bT[2 * dst + 0] -= Mcmp.x();
			bT[2 * src + 1] += Mcmp.y();
			bT[2 * dst + 1] -= Mcmp.y();
		}

		// Reduced RHS
		Eigen::VectorXd bFz_red = Sz * bFz;
		Eigen::VectorXd bT_red = Sb * bT;

		// ---- 7) Solve reduced SPD systems (single-shot; no caching) ----
		Eigen::SimplicialLDLT<Eigen::SparseMatrix<double>> solverZ, solverB;
		solverZ.compute(Lz_red);
		if (solverZ.info() != Eigen::Success) {
			log_error("solveLimits(): Lz factorization failed");
			return violated;
		}
		Eigen::VectorXd u_red = solverZ.solve(bFz_red);
		if (solverZ.info() != Eigen::Success) {
			log_error("solveLimits(): Lz solve failed");
			return violated;
		}

		Eigen::VectorXd th_red;
		if (Lb_red.nonZeros() > 0) {
			solverB.compute(Lb_red);
			if (solverB.info() != Eigen::Success) {
				log_error("solveLimits(): Lb factorization failed");
				return violated;
			}
			th_red = solverB.solve(bT_red);
			if (solverB.info() != Eigen::Success) {
				log_error("solveLimits(): Lb solve failed");
				return violated;
			}
		} else {
			th_red = Eigen::VectorXd::Zero(2 * (n - ncc));
		}

		// Expand to full potentials (anchors = 0)
		Eigen::VectorXd u = Eigen::VectorXd::Zero(n);
		for (int k = 0; k < (int)keepNodes.size(); ++k)
			u[keepNodes[k]] = u_red[k];

		Eigen::VectorXd th = Eigen::VectorXd::Zero(2 * n);
		for (int k = 0; k < (int)keepB.size(); ++k)
			th[keepB[k]] = th_red[k];

		// ---- 8) Edge flows at alpha=1 and envelope check (base + delta) ----
		double alphaStar = std::numeric_limits<double>::infinity();
		std::vector<double> ratio(E.size(),
		                          std::numeric_limits<double>::infinity());
		std::vector<char> isViolated(E.size(), 0);

		for (std::size_t e = 0; e < E.size(); ++e) {
			const auto &be = E[e];
			const int i = be.i, j = be.j;

			// --- Elastic delta from aux removal (alpha = 1) ---
			const double Fz_delta = be.Kfz * (u[i] - u[j]);

			const Eigen::Vector2d thi(th[2 * i + 0], th[2 * i + 1]);
			const Eigen::Vector2d thj(th[2 * j + 0], th[2 * j + 1]);
			const Eigen::Vector2d dth = thi - thj;
			const Eigen::Vector2d Tglob_delta = be.KxyGlob * dth;

			const double c = std::cos(-be.phi), s = std::sin(-be.phi);
			Eigen::Matrix2d Rm;
			Rm << c, -s, s, c;
			const Eigen::Vector2d Tloc_delta = Rm * Tglob_delta;
			double Tx_delta = Tloc_delta[0];
			double Ty_delta = Tloc_delta[1];

			// --- Measured base-constraint wrench (always) ---
			double Fz_base = 0.0, Tx_base = 0.0, Ty_base = 0.0;
			if (be.conn && be.conn->joint) {
				physx::PxRigidActor *a0 = nullptr, *a1 = nullptr;
				be.conn->joint->getActors(a0, a1);

				physx::PxVec3 Fw(0.0f, 0.0f, 0.0f), Mw(0.0f, 0.0f, 0.0f);
				be.conn->joint->getForce(Fw, Mw);

				// Project world -> component canonical basis
				Eigen::Vector3d Fcmp = Rcw * Eigen::Vector3d(Fw.x, Fw.y, Fw.z);
				Eigen::Vector3d Mcmp = Rcw * Eigen::Vector3d(Mw.x, Mw.y, Mw.z);

				// Orient along edge i->j (match delta's sign convention)
				int sgn = 0;
				if (a0 == nodeRev[i]->actor && a1 == nodeRev[j]->actor)
					sgn = +1;
				else if (a0 == nodeRev[j]->actor && a1 == nodeRev[i]->actor)
					sgn = -1;
				else
					sgn = +1; // fallback

				// Rotate bending to edge-local
				const double cph = std::cos(-be.phi), sph = std::sin(-be.phi);
				const double Tx_loc_base = cph * Mcmp.x() - sph * Mcmp.y();
				const double Ty_loc_base = sph * Mcmp.x() + cph * Mcmp.y();

				Fz_base = sgn * Fcmp.z();
				Tx_base = sgn * Tx_loc_base;
				Ty_base = sgn * Ty_loc_base;
			}

			// --- Base-only equivalent load to test ---
			const double Fz_star = Fz_base + Fz_delta;
			const double Tx_star = Tx_base + Tx_delta;
			const double Ty_star = Ty_base + Ty_delta;

			// Pressure consumption (per-edge)
			const double use = std::max(Fz_star, 0.0) +
			                   (3.0 / be.b) * std::abs(Tx_star) +
			                   (3.0 / be.a) * std::abs(Ty_star);

			if (use > kTolZero) {
				const double r = be.N0 / use;
				ratio[e] = r;
				alphaStar = std::min(alphaStar, r);
				if (r < 1.0 - kViolTol) {
					isViolated[e] = 1;
				}
			}
		}

		// ---- 9) Report violated base joints ----
		for (std::size_t e = 0; e < E.size(); ++e) {
			if (isViolated[e]) {
				violated.push_back(E[e].conn);
			}
		}

		// (Optional) Log min safety factor
		if (alphaStar < std::numeric_limits<double>::infinity()) {
			log_info("solveLimits(): min safety factor alpha* = {:.6f} "
			         "({}/{}) violated)",
			         alphaStar, violated.size(), E.size());
		} else {
			log_info("solveLimits(): no demand detected; all safe.");
		}

		return violated;
	}

	void setupConn(ConnDesc &c) {
		c.joint = createConstraint(c.parent, c.child, c.tf);
		contactExclusions.insert({c.parent->actor, c.child->actor});
	}

	void releaseConn(ConnDesc &c) {
		c.joint->release();
		contactExclusions.erase({c.parent->actor, c.child->actor});
	}

	ConnMap::iterator findConnBidirectional(RigidBody *a, RigidBody *b) {
		auto it = conns.find(std::make_pair(a, b));
		if (it != conns.end()) {
			return it;
		}
		return conns.find(std::make_pair(b, a));
	}

	bool lookupGraphTransform(const BodyDesc *a, const BodyDesc *b,
	                          Transform &T_a_b) const {
		if (a == nullptr || b == nullptr) {
			return false;
		}
		if (a == b) {
			T_a_b = Transform(physx::PxIdentity);
			return true;
		}

		struct Node {
			const BodyDesc *v;
			Transform T_a_v;
		};

		std::deque<Node> q;
		std::unordered_set<const BodyDesc *> visited;
		q.push_back({a, Transform(physx::PxIdentity)});
		visited.insert(a);

		while (!q.empty()) {
			Node cur = q.front();
			q.pop_front();

			if (cur.v == b) {
				T_a_b = cur.T_a_v;
				return true;
			}

			// Traverse to children (current is parent): use forward tf
			for (const ConnDesc *c : cur.v->children) {
				if (!c || !c->child)
					continue;
				const BodyDesc *nxt = c->child;
				if (visited.find(nxt) != visited.end())
					continue;
				auto T_a_nxt = cur.T_a_v * c->tf;
				visited.insert(nxt);
				q.push_back({nxt, T_a_nxt});
			}

			// Traverse to parents (current is child): use inverse tf
			for (const ConnDesc *c : cur.v->parents) {
				if (!c || !c->parent)
					continue;
				const BodyDesc *nxt = c->parent;
				if (visited.find(nxt) != visited.end())
					continue;
				auto T_nxt = cur.T_a_v * c->tf.getInverse();
				visited.insert(nxt);
				q.push_back({nxt, T_nxt});
			}
		}

		// Not connected
		return false;
	}

	Constraint *createAuxConstraint(BodyDesc *a, BodyDesc *b) {
		Transform T_a_b;
		if (!lookupGraphTransform(a, b, T_a_b)) {
			log_fatal("Cannot find graph transform from {:p} to {:p}",
			          static_cast<const void *>(a),
			          static_cast<const void *>(b));
			return nullptr;
		}
		log_info("Creating aux constraint between {:p} and {:p}",
		         static_cast<const void *>(a), static_cast<const void *>(b));
		return createConstraint(a, b, T_a_b);
	}

	void destroyAuxConstraint(Constraint *j) {
		if (!j) {
			log_fatal("Cannot destroy null constraint");
			return;
		}
		log_info("Destroying aux constraint {:p}",
		         static_cast<const void *>(j));
		j->release();
	}

	Constraint *createConstraint(BodyDesc *a, BodyDesc *b, Transform T_a_b) {
		// PxConstraint shader uses body frames (COM frames) bA2w/bB2w.
		// Our T_a_b (from Python) is defined between actor-local origins (bottom centers).
		// Convert to COM-local frames so the weld aligns the intended anchor points.
		//
		// parentLocal (A_com -> B_com) = (A_com -> A_orig) * (A_orig -> B_orig) * (B_orig -> B_com)
		// childLocal is identity so cB2w = bB2w (B_com).

		Transform A_o2com(physx::PxIdentity);
		Transform B_o2com(physx::PxIdentity);

		if (auto *rbA = a->actor->is<physx::PxRigidBody>()) {
			A_o2com = rbA->getCMassLocalPose();
		}
		if (auto *rbB = b->actor->is<physx::PxRigidBody>()) {
			B_o2com = rbB->getCMassLocalPose();
		}

		Transform A_com2o = A_o2com.getInverse();
		Transform parentLocal = A_com2o * T_a_b * B_o2com;
		Transform childLocal(physx::PxIdentity);

		return createWeldConstraint(
		    *px, a->actor, b->actor,
		    {.parentLocal = parentLocal, .childLocal = childLocal});
	}

	bool resolveActor(const physx::PxActor *actor, BodyDesc *&out) {
		if (actor->getType() != physx::PxActorType::eRIGID_DYNAMIC) {
			return false;
		}
		auto it = bodies.find(
		    const_cast<RigidBody *>(static_cast<const RigidBody *>(actor)));
		if (it == bodies.end()) {
			return false;
		}
		out = &it->second;
		return true;
	}

	void processAssemblyContact(BodyDesc *body0, const Transform &pose0_px,
	                            BodyDesc *body1, const Transform &pose1_px,
	                            physx::PxVec3 impulse_px) {
		using Eigen::Matrix2d;
		using Eigen::Matrix3d;
		using Eigen::Rotation2Dd;
		using Eigen::Vector2d;
		using Eigen::Vector3d;
		using std::abs;
		using std::atan2;
		using std::cos;
		using std::round;
		using std::sin;
		using std::numbers::pi;

		double dt = getElapsedTime(body0->actor->getScene());
		auto dim0 = as<Vector3d>(body0->info.dimensions);
		auto dim1 = as<Vector3d>(body1->info.dimensions);
		auto [R0, t0] = as_pose_rt<eigen_tag, double>(pose0_px);
		auto [R1, t1] = as_pose_rt<eigen_tag, double>(pose1_px);
		auto impulse = as<Vector3d>(impulse_px); // From 2nd to 1st

		auto rel_R = R0.transpose() * R1;
		auto rel_t = R0.transpose() * (t1 - t0);

		// Calculate relative distance
		auto brick0_height = dim0(2) * (PlateHeight / cfg.mpu);
		auto rel_dist = rel_t(2) - (brick0_height + (StudHeight / cfg.mpu));

		// Calculate projected force along brick0's z-axis
		auto fz = impulse.dot(R0.col(2)) / dt;

		// Calculate snapped yaw & yaw error
		auto rel_yaw = atan2(rel_R(1, 0), rel_R(0, 0));
		auto yaw_snap = round(rel_yaw / (pi / 2)) * (pi / 2);
		auto R_snap = Rotation2Dd(yaw_snap).toRotationMatrix();
		auto yaw_err = rel_yaw - yaw_snap;

		// Calculate snapped position in grid coordinates & position error
		auto grid_scale = BrickLength / cfg.mpu;
		auto p0 =
		    rel_t.head<2>() / grid_scale +
		    (dim0.head<2>() - rel_R.topLeftCorner<2, 2>() * dim1.head<2>()) / 2;
		auto p0_snap = p0.array().round().matrix();
		auto p1_snap =
		    (p0_snap + R_snap * dim1.head<2>()).array().round().matrix();
		auto p_err = (p0 - p0_snap).norm() * grid_scale;

		// Calculate overlap
		auto ov_start1 = Vector2d::Zero();
		auto ov_end1 = dim0.head<2>();
		auto ov_start2 = p0_snap.cwiseMin(p1_snap);
		auto ov_end2 = p0_snap.cwiseMax(p1_snap);
		auto ov_start = ov_start1.cwiseMax(ov_start2);
		auto ov_end = ov_end1.cwiseMin(ov_end2);
		auto ov_mid = ((ov_start + ov_end) / 2).eval();
		auto overlap = (ov_end - ov_start).cwiseMax(Vector2d::Zero()).eval();

		// Calculate joint local transforms
		Matrix3d T_parent_local_R = Matrix3d::Identity();
		Vector3d T_parent_local_t;
		T_parent_local_t.head<2>() = (ov_mid - dim0.head<2>() / 2) * grid_scale;
		T_parent_local_t(2) = brick0_height;

		Matrix2d R_snap_T = R_snap.transpose();
		Matrix3d T_child_local_R = Matrix3d::Identity();
		T_child_local_R.topLeftCorner<2, 2>() = R_snap_T;
		Vector3d T_child_local_t = Vector3d::Zero();
		T_child_local_t.head<2>() =
		    R_snap_T * (ov_mid - (ov_start2 + ov_end2) / 2) * grid_scale;

		// Filtering
		bool accept = (rel_dist <= (thresholds.DistanceTolerance / cfg.mpu)) &&
		              (rel_dist >= -(thresholds.MaxPenetration / cfg.mpu)) &&
		              (rel_R(2, 2) > cos(thresholds.ZAngleTolerance)) &&
		              (fz <= -(thresholds.RequiredForce / cfg.mpu / cfg.kpu)) &&
		              (abs(yaw_err) <= thresholds.YawTolerance) &&
		              (p_err <= (thresholds.PositionTolerance / cfg.mpu)) &&
		              ((overlap(0) > 0) && (overlap(1) > 0));
		if (!accept) {
			return;
		}
		if (findConnBidirectional(body0->actor, body1->actor) != conns.end()) {
			return;
		}

		BrickOrientation orientation = static_cast<BrickOrientation>(
		    static_cast<std::int8_t>(round(rel_yaw / (pi / 2))) &
		    std::int8_t(3));
		assemblyEvents.push_back({
		    .parent = body0->actor,
		    .child = body1->actor,
		    .offset_studs = as_array<BrickUnit, 2>(p0_snap),
		    .orientation = orientation,
		    .T_parent_local =
		        as<physx::PxTransform>(T_parent_local_R, T_parent_local_t),
		    .T_child_local =
		        as<physx::PxTransform>(T_child_local_R, T_child_local_t),
		    .overlap_xy = {static_cast<float>(overlap(0) * grid_scale),
		                   static_cast<float>(overlap(1) * grid_scale)},
		});
	}
};

LegoGraph::LegoGraph(physx::PxPhysics *px, const Config &config)
    : impl_(std::make_unique<Impl>(px, config)) {}
LegoGraph::~LegoGraph() = default;

bool LegoGraph::addRigidBody(physx::PxRigidActor *actor,
                             const BrickInfo &info) {
	return impl_->addRigidBody(actor, info);
}
bool LegoGraph::removeRigidBody(physx::PxRigidActor *actor) {
	return impl_->removeRigidBody(actor);
}
bool LegoGraph::connect(physx::PxRigidActor *a, physx::PxRigidActor *b,
                        const ConnInfo &info) {
	return impl_->connect(a, b, info);
}
bool LegoGraph::disconnect(physx::PxRigidActor *a, physx::PxRigidActor *b) {
	return impl_->disconnect(a, b);
}
std::vector<std::pair<physx::PxRigidActor *, physx::PxRigidActor *>>
LegoGraph::solveLimits() {
	auto violated = impl_->solveLimits();
	std::vector<std::pair<physx::PxRigidActor *, physx::PxRigidActor *>> res;
	res.reserve(violated.size());
	for (auto *c : violated) {
		res.emplace_back(c->parent->actor, c->child->actor);
	}
	return res;
}
std::vector<LegoGraph::AssemblyEvent> LegoGraph::pollAssemblyEvents() {
	auto result = std::move(impl_->assemblyEvents);
	impl_->assemblyEvents.clear();
	return result;
}
void LegoGraph::clear() {
	impl_->clear();
}
void LegoGraph::getThresholds(Thresholds &out) const {
	out = impl_->thresholds;
}
void LegoGraph::setThresholds(const Thresholds &in) {
	impl_->thresholds = in;
}

} // namespace lego_assemble
