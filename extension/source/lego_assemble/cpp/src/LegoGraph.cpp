#include "LegoGraph.h"

#include "LegoWeldConstraint.h"
#include "ScenePatcher.h"
#include "SkipGraph.h"

#include <deque>
#include <unordered_map>
#include <unordered_set>

#include <Eigen/Dense>
#include <Eigen/Sparse>

#include <carb/logging/Log.h>

#include <PxRigidBody.h>

namespace lego_assemble {

class LegoGraph::Impl {
  public:
	struct BodyDesc;
	struct ConnDesc;
	using RigidBody = physx::PxRigidActor;
	using Constraint = physx::PxConstraint;
	using Shape = physx::PxShape;
	using Scheduler = SimpleSkipGraphScheduler<
	    BodyDesc *, Constraint *,
	    std::function<Constraint *(BodyDesc *, BodyDesc *)>,
	    std::function<void(Constraint *)>>;
	using Transform = physx::PxTransform;
	using BodyMap = std::unordered_map<RigidBody *, BodyDesc>;
	using ConnMap =
	    std::unordered_map<std::pair<RigidBody *, RigidBody *>, ConnDesc,
	                       PairHash<RigidBody *, std::hash<RigidBody *>>,
	                       PairEq<RigidBody *, std::equal_to<RigidBody *>>>;

	struct BodyDesc {
		RigidBody *actor;
		std::unordered_set<ConnDesc *> parents;  // this is child
		std::unordered_set<ConnDesc *> children; // this is parent
		Shape *body_collider;
		Shape *top_collider;
	};
	struct ConnDesc {
		Constraint *joint;
		BodyDesc *parent;
		BodyDesc *child;
		Transform tf;
		Transform T_parent_local;
		Transform T_child_local;
		float overlap_xy[2]; // in stage units
	};

	explicit Impl(physx::PxPhysics *px)
	    : px_(px),
	      scheduler_(std::bind(&Impl::createAuxConstraint_, this,
	                           std::placeholders::_1, std::placeholders::_2),
	                 std::bind(&Impl::destroyAuxConstraint_, this,
	                           std::placeholders::_1)) {}

	Impl(const Impl &) = delete;
	Impl &operator=(const Impl &) = delete;
	~Impl() {
		clear();
	}

	bool addRigidBody(RigidBody *actor, const BrickInfo &info) {
		if (bodies_.contains(actor)) {
			return false;
		}
		BodyDesc desc;
		desc.actor = actor;
		desc.body_collider = info.body_collider;
		desc.top_collider = info.top_collider;
		bodies_[actor] = desc;
		return true;
	}

	bool removeRigidBody(RigidBody *actor) {
		auto it = bodies_.find(actor);
		if (it == bodies_.end()) {
			return false;
		}
		BodyDesc &desc = it->second;
		for (ConnDesc *c : desc.parents) {
			if (!c->parent->children.erase(c)) {
				CARB_LOG_ERROR(
				    "Inconsistent state when removing body %p, continuing",
				    actor);
			}
			releaseConn_(*c);
			if (!scheduler_.disconnect(c->parent, c->child)) {
				CARB_LOG_ERROR("Failed to remove edge between %p and %p from "
				               "scheduler, continuing",
				               c->parent->actor, c->child->actor);
			}
			auto edge_key = std::make_pair(c->parent->actor, c->child->actor);
			if (!conns_.erase(edge_key)) {
				CARB_LOG_ERROR(
				    "Failed to remove connection [%p, %p] from map, continuing",
				    c->parent->actor, c->child->actor);
			}
		}
		for (ConnDesc *c : desc.children) {
			if (!c->child->parents.erase(c)) {
				CARB_LOG_ERROR(
				    "Inconsistent state when removing body %p, continuing",
				    actor);
			}
			releaseConn_(*c);
			if (!scheduler_.disconnect(c->parent, c->child)) {
				CARB_LOG_ERROR("Failed to remove edge between %p and %p from "
				               "scheduler, continuing",
				               c->parent->actor, c->child->actor);
			}
			auto edge_key = std::make_pair(c->parent->actor, c->child->actor);
			if (!conns_.erase(edge_key)) {
				CARB_LOG_ERROR(
				    "Failed to remove connection [%p, %p] from map, continuing",
				    c->parent->actor, c->child->actor);
			}
		}
		bodies_.erase(it);
		return true;
	}

	bool connect(RigidBody *a, RigidBody *b, const ConnInfo &info) {
		if (a == b) {
			return false;
		}
		auto ita = bodies_.find(a);
		auto itb = bodies_.find(b);
		if (ita == bodies_.end() || itb == bodies_.end()) {
			return false;
		}
		BodyDesc *da = &ita->second;
		BodyDesc *db = &itb->second;
		if (find_conn_bidirectional_(a, b) != conns_.end()) {
			return false;
		}

		ConnDesc c;
		c.parent = da;
		c.child = db;
		c.tf = info.T_parent_local * info.T_child_local.getInverse();
		c.T_parent_local = info.T_parent_local;
		c.T_child_local = info.T_child_local;
		c.overlap_xy[0] = info.overlap_xy[0];
		c.overlap_xy[1] = info.overlap_xy[1];
		setupConn_(c);
		auto edge_key = std::make_pair(a, b);
		conns_[edge_key] = c;
		da->children.insert(&conns_[edge_key]);
		db->parents.insert(&conns_[edge_key]);
		if (!scheduler_.connect(da, db)) {
			CARB_LOG_ERROR(
			    "Failed to add edge between %p and %p to scheduler, continuing",
			    a, b);
		}
		b->getScene()->resetFiltering(*b);
		CARB_LOG_INFO("Created base constraint between %p and %p", a, b);
		return true;
	}

	bool disconnect(RigidBody *a, RigidBody *b) {
		if (a == b) {
			return false;
		}
		auto ita = bodies_.find(a);
		auto itb = bodies_.find(b);
		if (ita == bodies_.end() || itb == bodies_.end()) {
			return false;
		}
		auto itc = find_conn_bidirectional_(a, b);
		if (itc == conns_.end()) {
			return false;
		}

		ConnDesc &c = itc->second;
		c.parent->children.erase(&c);
		c.child->parents.erase(&c);
		releaseConn_(c);
		if (!scheduler_.disconnect(c.parent, c.child)) {
			CARB_LOG_ERROR("Failed to remove edge between %p and %p from "
			               "scheduler, continuing",
			               c.parent->actor, c.child->actor);
		}
		c.child->actor->getScene()->resetFiltering(*c.child->actor);
		CARB_LOG_INFO("Destroyed base constraint between %p and %p",
		              c.parent->actor, c.child->actor);
		conns_.erase(itc);
		return true;
	}

	void clear() {
		scheduler_.clear();
		for (auto &[_, c] : conns_) {
			releaseConn_(c);
		}
		conns_.clear();
		bodies_.clear();
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
		if (conns_.empty())
			return violated;

		// ---- Helpers ----
		auto pxQuatToEigen = [](const physx::PxQuat &q) -> Eigen::Matrix3d {
			physx::PxMat33 R(q);
			Eigen::Matrix3d M;
			// PxMat33 stores columns column0..2
			M.col(0) = Eigen::Vector3d(R.column0.x, R.column0.y, R.column0.z);
			M.col(1) = Eigen::Vector3d(R.column1.x, R.column1.y, R.column1.z);
			M.col(2) = Eigen::Vector3d(R.column2.x, R.column2.y, R.column2.z);
			return M;
		};

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
		nodeRev.reserve(conns_.size() * 2);
		for (auto &kv : conns_) {
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
		E.reserve(conns_.size());
		// We'll also build an adjacency for CC detection
		std::vector<std::vector<int>> adj(n);

		// ---- Pick a root per connected component (we'll fill later). For now pick a global root candidate. ----
		BodyDesc *rootBody = nodeRev[0];
		Eigen::Matrix3d Rroot_world =
		    pxQuatToEigen(rootBody->actor->getGlobalPose().q);

		// ---- Build graph-world rotations Rgw for every node using your graph transforms ----
		// We do a BFS from the chosen rootBody over the base graph using your lookupGraphTransform_
		std::vector<Eigen::Matrix3d> Rgw(n, Eigen::Matrix3d::Identity());
		{
			// Fill adjacency from base connections for the BFS of rotations
			std::vector<std::vector<int>> g(n);
			for (auto &kv : conns_) {
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
					    lookupGraphTransform_(nodeRev[u], nodeRev[v], T_u_v);
					if (!ok) {
						// If disconnected by mistake, keep identity; but this shouldn't happen inside a CC.
						Rgw[v] = Rgw[u];
					} else {
						Eigen::Matrix3d R_uv = pxQuatToEigen(T_u_v.q);
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
		for (auto &kv : conns_) {
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
			Eigen::Matrix3d R_joint_parentLocal =
			    pxQuatToEigen(c.T_parent_local.q);
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
				CARB_LOG_WARN(
				    "solveLimits(): non-parallel joint normals detected; "
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
		for (auto const &aux : scheduler_.aux_edges()) {
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
			CARB_LOG_ERROR("solveLimits(): Lz factorization failed");
			return violated;
		}
		Eigen::VectorXd u_red = solverZ.solve(bFz_red);
		if (solverZ.info() != Eigen::Success) {
			CARB_LOG_ERROR("solveLimits(): Lz solve failed");
			return violated;
		}

		Eigen::VectorXd th_red;
		if (Lb_red.nonZeros() > 0) {
			solverB.compute(Lb_red);
			if (solverB.info() != Eigen::Success) {
				CARB_LOG_ERROR("solveLimits(): Lb factorization failed");
				return violated;
			}
			th_red = solverB.solve(bT_red);
			if (solverB.info() != Eigen::Success) {
				CARB_LOG_ERROR("solveLimits(): Lb solve failed");
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

		// ---- 8) Edge flows at alpha=1 and envelope check ----
		double alphaStar = std::numeric_limits<double>::infinity();
		std::vector<double> ratio(E.size(),
		                          std::numeric_limits<double>::infinity());
		std::vector<char> isViolated(E.size(), 0);

		for (size_t e = 0; e < E.size(); ++e) {
			auto const &be = E[e];
			int i = be.i, j = be.j;

			// Fz (scalar)
			double Fz = be.Kfz * (u[i] - u[j]);

			// Bending vector in component basis
			Eigen::Vector2d thi(th[2 * i + 0], th[2 * i + 1]);
			Eigen::Vector2d thj(th[2 * j + 0], th[2 * j + 1]);
			Eigen::Vector2d dth = thi - thj; // potential difference
			Eigen::Vector2d Tglob = be.KxyGlob * dth;

			// Rotate back to edge-local axes to apply (3/b)|Tx| + (3/a)|Ty|
			double c = std::cos(-be.phi), s = std::sin(-be.phi);
			Eigen::Matrix2d Rm;
			Rm << c, -s, s, c;
			Eigen::Vector2d Tloc = Rm * Tglob;
			double Tx_loc = Tloc[0], Ty_loc = Tloc[1];

			// Pressure consumption (per-edge)
			double use = std::max(Fz, 0.0) + (3.0 / be.b) * std::abs(Tx_loc) +
			             (3.0 / be.a) * std::abs(Ty_loc);

			if (use > kTolZero) {
				double r = be.N0 / use;
				ratio[e] = r;
				if (r < 1.0 - kViolTol) {
					isViolated[e] = 1;
					alphaStar = std::min(alphaStar, r);
				} else {
					alphaStar = std::min(alphaStar, r);
				}
			}
		}

		// ---- 9) Report violated base joints ----
		for (size_t e = 0; e < E.size(); ++e) {
			if (isViolated[e]) {
				violated.push_back(E[e].conn);
			}
		}

		// (Optional) Log min safety factor
		if (alphaStar < std::numeric_limits<double>::infinity()) {
			CARB_LOG_INFO("solveLimits(): min safety factor alpha* = %.6f "
			              "(%zu/%zu violated)",
			              alphaStar, violated.size(), E.size());
		} else {
			CARB_LOG_INFO("solveLimits(): no demand detected; all safe.");
		}

		return violated;
	}

  private:
	physx::PxPhysics *px_;
	BodyMap bodies_;
	ConnMap conns_;
	Scheduler scheduler_;

	void setupConn_(ConnDesc &c) {
		c.joint = createConstraint_(c.parent, c.child, c.tf);
		if (!addContactExclusion(c.parent->actor, nullptr, c.child->actor,
		                         nullptr)) {
			CARB_LOG_ERROR("Failed to add contact exclusion between %p and %p",
			               c.parent->actor, c.child->actor);
		}
	}

	void releaseConn_(ConnDesc &c) {
		c.joint->release();
		if (!removeContactExclusion(c.parent->actor, nullptr, c.child->actor,
		                            nullptr)) {
			CARB_LOG_ERROR(
			    "Failed to remove contact exclusion between %p and %p",
			    c.parent->actor, c.child->actor);
		}
	}

	ConnMap::iterator find_conn_bidirectional_(RigidBody *a, RigidBody *b) {
		auto it = conns_.find(std::make_pair(a, b));
		if (it != conns_.end()) {
			return it;
		}
		return conns_.find(std::make_pair(b, a));
	}

	bool lookupGraphTransform_(const BodyDesc *a, const BodyDesc *b,
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

	Constraint *createAuxConstraint_(BodyDesc *a, BodyDesc *b) {
		Transform T_a_b;
		if (!lookupGraphTransform_(a, b, T_a_b)) {
			CARB_LOG_FATAL("Cannot find graph transform from %p to %p", a, b);
			return nullptr;
		}
		CARB_LOG_INFO("Creating aux constraint between %p and %p", a, b);
		return createConstraint_(a, b, T_a_b);
	}

	void destroyAuxConstraint_(Constraint *j) {
		if (!j) {
			CARB_LOG_FATAL("Cannot destroy null constraint");
			return;
		}
		CARB_LOG_INFO("Destroying aux constraint %p", j);
		j->release();
	}

	Constraint *createConstraint_(BodyDesc *a, BodyDesc *b, Transform T_a_b) {
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

		return CreateLegoWeld(
		    *px_, a->actor, b->actor,
		    {.parentLocal = parentLocal, .childLocal = childLocal});
	}
};

LegoGraph::LegoGraph(physx::PxPhysics *px)
    : impl_(std::make_unique<Impl>(px)) {}
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
void LegoGraph::clear() {
	impl_->clear();
}

}; // namespace lego_assemble
