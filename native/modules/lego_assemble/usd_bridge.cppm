export module lego_assemble.usd_bridge;

import std;
import lego_assemble.brick_naming;
import lego_assemble.usd.tokens;
import lego_assemble.brick_specs;
import lego_assemble.lego_graph;
import lego_assemble.utils.pair;
import lego_assemble.utils.conversions;
import lego_assemble.utils.sdf;
import lego_assemble.vendor.carb;
import lego_assemble.vendor.physx;
import lego_assemble.vendor.pxr;
import lego_assemble.vendor.omni;

namespace lego_assemble {

export class LegoUsdBridge {
  public:
	explicit LegoUsdBridge(pxr::UsdStageRefPtr stage, physx::PxPhysics *px,
	                       omni::physx::IPhysx *omni_px);
	LegoUsdBridge(const LegoUsdBridge &) = delete;
	LegoUsdBridge &operator=(const LegoUsdBridge &) = delete;
	~LegoUsdBridge();

	void onRigidCreated(physx::PxRigidActor *actor,
	                    const pxr::SdfPath &primPath);
	void onRigidDestroyed(physx::PxRigidActor *actor,
	                      const pxr::SdfPath &primPath);
	void enqueuePrimChange(const pxr::SdfPath &primPath);
	void onPreStep();
	void onPostStep();
	LegoGraph &getGraph();

  private:
	struct ConnDesc {
		pxr::SdfPath parent_path;
		pxr::SdfPath child_path;
		std::uint64_t creation_time;
	};

	pxr::UsdStageRefPtr stage_;
	omni::physx::IPhysx *omni_px_;
	std::uint64_t current_time;

	pxr::SdfPathTable<physx::PxRigidActor *> bodies_;
	pxr::SdfPathTable<ConnDesc> conns_;
	std::unordered_map<physx::PxRigidActor *, pxr::SdfPath> body_rev_;
	std::unordered_map<std::pair<physx::PxRigidActor *, physx::PxRigidActor *>,
	                   pxr::SdfPath, PairHash<physx::PxRigidActor *>>
	    conn_rev_;
	LegoGraph graph_;
	std::vector<pxr::SdfPath> pendingChanges_;
	std::mutex mutex_;
};

static bool getBrickInfo(omni::physx::IPhysx *omni_px, const pxr::UsdPrim &prim,
                         LegoGraph::BrickInfo &out) {
	if (!prim.IsActive())
		return false;

	pxr::GfVec3i dimensions;
	if (!prim.GetAttribute(LegoTokens->brick_dimensions).Get(&dimensions))
		return false;

	auto body_collider_prim = prim.GetChild(LegoTokens->BodyCollider);
	if (!body_collider_prim || !body_collider_prim.IsActive())
		return false;
	auto *body_collider = static_cast<physx::PxShape *>(omni_px->getPhysXPtr(
	    body_collider_prim.GetPath(), omni::physx::ePTShape));
	if (!body_collider)
		return false;

	auto top_collider_prim = prim.GetChild(LegoTokens->TopCollider);
	if (!top_collider_prim || !top_collider_prim.IsActive())
		return false;
	auto *top_collider = static_cast<physx::PxShape *>(omni_px->getPhysXPtr(
	    top_collider_prim.GetPath(), omni::physx::ePTShape));
	if (!top_collider)
		return false;

	out.dimensions = as_array<BrickUnit, 3>(dimensions);
	out.body_collider = body_collider;
	out.top_collider = top_collider;
	return true;
}

struct ConnInfo {
	pxr::SdfPath parent;
	pxr::SdfPath child;
	physx::PxTransform T_parent_local;
	physx::PxTransform T_child_local;
	std::array<float, 2> overlap_xy;
};

static bool getConnInfo(const pxr::UsdPrim &prim, ConnInfo &out) {
	if (!prim.IsActive())
		return false;
	auto attr_enabled = prim.GetAttribute(LegoTokens->conn_enabled);
	if (!attr_enabled.IsValid())
		return false;
	bool enabled;
	if (!attr_enabled.Get<bool>(&enabled))
		return false;
	if (!enabled)
		return false;

	pxr::SdfPathVector t0, t1;
	prim.GetRelationship(LegoTokens->conn_body0).GetTargets(&t0);
	prim.GetRelationship(LegoTokens->conn_body1).GetTargets(&t1);
	if (t0.empty() || t1.empty())
		return false;
	out.parent = t0.front();
	out.child = t1.front();

	pxr::GfVec3f pos0(0);
	pxr::GfQuatf rot0(1, 0, 0, 0);
	pxr::GfVec3f pos1(0);
	pxr::GfQuatf rot1(1, 0, 0, 0);
	prim.GetAttribute(LegoTokens->conn_pos0).Get(&pos0);
	prim.GetAttribute(LegoTokens->conn_rot0).Get(&rot0);
	prim.GetAttribute(LegoTokens->conn_pos1).Get(&pos1);
	prim.GetAttribute(LegoTokens->conn_rot1).Get(&rot1);
	out.T_parent_local = as<physx::PxTransform>(rot0, pos0);
	out.T_child_local = as<physx::PxTransform>(rot1, pos1);

	pxr::GfVec2f overlap_xy(0, 0);
	prim.GetAttribute(LegoTokens->conn_overlap_xy).Get(&overlap_xy);
	out.overlap_xy = as_array<float, 2>(overlap_xy);
	return true;
}

LegoUsdBridge::LegoUsdBridge(pxr::UsdStageRefPtr stage, physx::PxPhysics *px,
                             omni::physx::IPhysx *omni_px)
    : omni_px_(omni_px), stage_(std::move(stage)),
      graph_(px, {.mpu = pxr::UsdGeomGetStageMetersPerUnit(stage_),
                  .kpu = pxr::UsdPhysicsGetStageKilogramsPerUnit(stage_)}),
      current_time(0) {

	std::lock_guard lock(mutex_);

	struct QueuedConn {
		pxr::SdfPath path;
		ConnInfo info;
	};

	std::vector<QueuedConn> queued_conns;
	for (const pxr::UsdPrim &prim : stage_->Traverse()) {
		ConnInfo conn_info;
		LegoGraph::BrickInfo brick_info;
		if (getConnInfo(prim, conn_info)) {
			queued_conns.push_back({prim.GetPath(), conn_info});
		} else if (getBrickInfo(omni_px_, prim, brick_info)) {
			auto path = prim.GetPath();
			auto *rb = static_cast<physx::PxRigidActor *>(
			    omni_px_->getPhysXPtr(path, omni::physx::ePTActor));
			if (rb) {
				bodies_[path] = rb;
				body_rev_[rb] = path;
				if (!graph_.addRigidBody(rb, brick_info)) {
					log_error("Failed to add rigid body for prim {}",
					          path.GetText());
				}
			} else {
				log_warn("Failed to get rigid body for brick prim {}",
				         path.GetText());
			}
		}
	}

	for (const auto &conn : queued_conns) {
		auto parent_it = bodies_.find(conn.info.parent);
		auto child_it = bodies_.find(conn.info.child);
		if (parent_it != bodies_.end() && child_it != bodies_.end()) {
			auto *rb_parent = parent_it->second;
			auto *rb_child = child_it->second;
			auto *ps = rb_parent->getScene();
			if (ps == rb_child->getScene()) {
				ps->lockWrite();
				if (graph_.connect(rb_parent, rb_child,
				                   {.T_parent_local = conn.info.T_parent_local,
				                    .T_child_local = conn.info.T_child_local,
				                    .overlap_xy = conn.info.overlap_xy})) {
					conns_[conn.path] = {
					    .parent_path = conn.info.parent,
					    .child_path = conn.info.child,
					    .creation_time = current_time,
					};
					conn_rev_[{rb_parent, rb_child}] = conn.path;
				} else {
					log_warn("Failed to connect bodies for conn prim {}",
					         conn.path.GetText());
				}
				ps->unlockWrite();
			} else {
				log_warn("Bodies for conn prim {} are in different scenes",
				         conn.path.GetText());
			}
		} else {
			log_warn("Cannot create connection for conn prim {}: one "
			         "or both bodies do not exist",
			         conn.path.GetText());
		}
	}
}

LegoUsdBridge::~LegoUsdBridge() {}

void LegoUsdBridge::onRigidCreated(physx::PxRigidActor *actor,
                                   const pxr::SdfPath &primPath) {
	pxr::UsdPrim prim = stage_->GetPrimAtPath(primPath);
	LegoGraph::BrickInfo info;
	if (!prim || !getBrickInfo(omni_px_, prim, info)) {
		return;
	}

	std::lock_guard lock(mutex_);
	if (bodies_.find(primPath) != bodies_.end()) {
		log_warn("Rigid body for prim {} already exists", primPath.GetText());
		return;
	}
	bodies_[primPath] = actor;
	body_rev_[actor] = primPath;
	if (!graph_.addRigidBody(actor, info)) {
		log_error("Failed to add rigid body for prim {}", primPath.GetText());
	}
}

void LegoUsdBridge::onRigidDestroyed(physx::PxRigidActor *actor,
                                     const pxr::SdfPath &primPath) {
	std::lock_guard lock(mutex_);
	auto it = bodies_.find(primPath);
	if (it == bodies_.end()) {
		return;
	}
	auto *rb = it->second;
	auto *ps = rb->getScene();
	// Need to lock because might remove connected edges (if any)
	ps->lockWrite();
	if (!graph_.removeRigidBody(rb)) {
		log_error("Failed to remove rigid body for prim {}",
		          primPath.GetText());
	}
	ps->unlockWrite();
	bodies_.erase(it);
	body_rev_.erase(actor);
}

void LegoUsdBridge::enqueuePrimChange(const pxr::SdfPath &primPath) {
	std::lock_guard lock(mutex_);
	pendingChanges_.push_back(primPath);
}

void LegoUsdBridge::onPreStep() {
	std::lock_guard lock(mutex_);
	for (const auto &path : pendingChanges_) {
		auto conn_it = conns_.find(path);
		bool conn_exists = (conn_it != conns_.end());

		bool prim_valid = false;
		ConnInfo prim_info;
		pxr::UsdPrim prim = stage_->GetPrimAtPath(path);
		if (prim && getConnInfo(prim, prim_info)) {
			prim_valid = true;
		}

		if (conn_exists && !prim_valid) {
			// Connection removed
			const auto &conn = conn_it->second;
			auto parent_it = bodies_.find(conn.parent_path);
			auto child_it = bodies_.find(conn.child_path);
			if (parent_it == bodies_.end() || child_it == bodies_.end()) {
				// This is a dangling connection
				// Underlying connection has already been removed
			} else {
				auto *rb_parent = parent_it->second;
				auto *rb_child = child_it->second;
				if (!conn_rev_.erase({rb_parent, rb_child})) {
					log_warn(
					    "Failed to remove reverse mapping for conn prim {}",
					    path.GetText());
				}
				auto *ps = rb_parent->getScene();
				if (ps == rb_child->getScene()) {
					ps->lockWrite();
					if (!graph_.disconnect(rb_parent, rb_child)) {
						log_error(
						    "Failed to disconnect bodies for conn prim {}",
						    path.GetText());
					}
					ps->unlockWrite();
				} else {
					log_warn("Bodies for conn prim {} are in different scenes",
					         path.GetText());
				}
			}
			conns_.erase(conn_it);

		} else if (!conn_exists && prim_valid) {
			// Connection added
			auto parent_it = bodies_.find(prim_info.parent);
			auto child_it = bodies_.find(prim_info.child);
			if (parent_it != bodies_.end() && child_it != bodies_.end()) {
				auto *rb_parent = parent_it->second;
				auto *rb_child = child_it->second;
				auto *ps = rb_parent->getScene();
				if (ps == rb_child->getScene()) {
					ps->lockWrite();
					if (graph_.connect(
					        rb_parent, rb_child,
					        {.T_parent_local = prim_info.T_parent_local,
					         .T_child_local = prim_info.T_child_local,
					         .overlap_xy = prim_info.overlap_xy})) {
						conns_[path] = {
						    .parent_path = prim_info.parent,
						    .child_path = prim_info.child,
						    .creation_time = current_time,
						};
						conn_rev_[{rb_parent, rb_child}] = path;
					} else {
						log_warn("Failed to connect bodies for conn prim {}",
						         path.GetText());
					}
					ps->unlockWrite();
				} else {
					log_warn("Bodies for conn prim {} are in different scenes",
					         path.GetText());
				}
			} else {
				log_warn("Cannot create connection for conn prim {}: "
				         "one or both bodies do not exist",
				         path.GetText());
			}
		}
	}
	pendingChanges_.clear();
}

void LegoUsdBridge::onPostStep() {
	constexpr static std::uint64_t kConnDebounceTime = 10;

	auto layer = stage_->GetEditTarget().GetLayer();
	pxr::SdfChangeBlock _changes;
	std::lock_guard lock(mutex_);

	auto broken_conns = graph_.solveLimits();
	for (const auto &[a, b] : broken_conns) {
		auto it = conn_rev_.find({a, b});
		if (it == conn_rev_.end()) {
			log_warn("Cannot find prim for broken connection [{:p}, {:p}]",
			         static_cast<const void *>(a),
			         static_cast<const void *>(b));
			continue;
		}
		auto path = it->second;
		auto &conn = conns_[path];
		if (conn.creation_time > 0 &&
		    current_time < conn.creation_time + kConnDebounceTime) {
			// Ignore if just created, connections created at the beginning are not debounced
			continue;
		}
		auto attr = layer->GetAttributeAtPath(
		    path.AppendProperty(LegoTokens->conn_enabled));
		if (!attr) {
			log_warn("Cannot find attribute for broken connection prim {}",
			         path.GetText());
			continue;
		}
		if (!attr->SetDefaultValue(pxr::VtValue(false))) {
			log_warn("Failed to disable broken connection prim {}",
			         path.GetText());
		} else {
			log_info("Disabled broken connection prim {}", path.GetText());
		}
	}

	auto assembly_events = graph_.pollAssemblyEvents();
	for (const auto &event : assembly_events) {
		auto parent_it = body_rev_.find(event.parent);
		if (parent_it == body_rev_.end()) {
			log_warn("Cannot find prim for assembled body {:p}",
			         static_cast<const void *>(event.parent));
			continue;
		}
		auto child_it = body_rev_.find(event.child);
		if (child_it == body_rev_.end()) {
			log_warn("Cannot find prim for assembled body {:p}",
			         static_cast<const void *>(event.child));
			continue;
		}
		auto parent_path = parent_it->second;
		auto child_path = child_it->second;
		auto conn_path = safeConnPathForBricks(parent_path, child_path);

		auto prim = pxr::SdfCreatePrimInLayer(layer, conn_path);
		prim->SetSpecifier(pxr::SdfSpecifierDef);
		prim->SetTypeName(pxr::UsdGeomTokens->Xform);

		SetRelationship(prim, LegoTokens->conn_body0, parent_path);
		SetRelationship(prim, LegoTokens->conn_body1, child_path);
		SetAttr<pxr::GfVec2i>(prim, LegoTokens->conn_offset_studs,
		                      event.offset_studs);
		SetAttr<int>(prim, LegoTokens->conn_yaw_index, event.orientation);
		SetAttr<pxr::GfVec3f>(prim, LegoTokens->conn_pos0,
		                      event.T_parent_local.p);
		SetAttr<pxr::GfQuatf>(prim, LegoTokens->conn_rot0,
		                      event.T_parent_local.q);
		SetAttr<pxr::GfVec3f>(prim, LegoTokens->conn_pos1,
		                      event.T_child_local.p);
		SetAttr<pxr::GfQuatf>(prim, LegoTokens->conn_rot1,
		                      event.T_child_local.q);
		SetAttr<pxr::GfVec2f>(prim, LegoTokens->conn_overlap_xy,
		                      event.overlap_xy);
		SetAttr<bool>(prim, LegoTokens->conn_enabled, true);
	}

	current_time++;
}

LegoGraph &LegoUsdBridge::getGraph() {
	return graph_;
}

} // namespace lego_assemble
