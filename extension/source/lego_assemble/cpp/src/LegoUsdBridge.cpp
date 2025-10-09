#include "LegoUsdBridge.h"
#include "LegoTokens.h"

#include <PxScene.h>

#include <pxr/usd/usd/attribute.h>
#include <pxr/usd/usd/prim.h>
#include <pxr/usd/usd/primRange.h>
#include <pxr/usd/usd/relationship.h>

namespace lego_assemble {

struct BrickInfo {
	pxr::SdfPath top_collider;
	pxr::SdfPath body_collider;
};

static bool getBrickInfo(const pxr::UsdPrim &prim, BrickInfo &out) {
	if (!prim.IsActive())
		return false;
	if (!prim.GetAttribute(LegoTokens->brick_dimensions).IsValid())
		return false;
	auto body_collider_prim = prim.GetChild(LegoTokens->BodyCollider);
	auto top_collider_prim = prim.GetChild(LegoTokens->TopCollider);
	if (!body_collider_prim || !body_collider_prim.IsActive())
		return false;
	if (!top_collider_prim || !top_collider_prim.IsActive())
		return false;
	out.body_collider = body_collider_prim.GetPath();
	out.top_collider = top_collider_prim.GetPath();
	return true;
}

struct ConnInfo {
	pxr::SdfPath parent;
	pxr::SdfPath child;
	physx::PxTransform T_parent_local;
	physx::PxTransform T_child_local;
	float overlap_xy[2];
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
	out.T_parent_local = physx::PxTransform(
	    physx::PxVec3(pos0[0], pos0[1], pos0[2]),
	    physx::PxQuat(rot0.GetImaginary()[0], rot0.GetImaginary()[1],
	                  rot0.GetImaginary()[2], rot0.GetReal()));
	out.T_child_local = physx::PxTransform(
	    physx::PxVec3(pos1[0], pos1[1], pos1[2]),
	    physx::PxQuat(rot1.GetImaginary()[0], rot1.GetImaginary()[1],
	                  rot1.GetImaginary()[2], rot1.GetReal()));

	pxr::GfVec2f overlap_xy(0, 0);
	prim.GetAttribute(LegoTokens->conn_overlap_xy).Get(&overlap_xy);
	out.overlap_xy[0] = overlap_xy[0];
	out.overlap_xy[1] = overlap_xy[1];
	return true;
}

LegoUsdBridge::LegoUsdBridge(pxr::UsdStageRefPtr stage, physx::PxPhysics *px,
                             omni::physx::IPhysx *omni_px)
    : omni_px_(omni_px), stage_(std::move(stage)), graph_(px), current_time(0) {

	std::lock_guard lock(mutex_);

	struct QueuedConn {
		pxr::SdfPath path;
		ConnInfo info;
	};

	std::vector<QueuedConn> queued_conns;
	for (const pxr::UsdPrim &prim : stage_->Traverse()) {
		ConnInfo conn_info;
		BrickInfo brick_info;
		if (getConnInfo(prim, conn_info)) {
			queued_conns.push_back({prim.GetPath(), conn_info});
		} else if (getBrickInfo(prim, brick_info)) {
			auto path = prim.GetPath();
			auto *rb = static_cast<physx::PxRigidActor *>(
			    omni_px_->getPhysXPtr(path, omni::physx::ePTActor));
			if (rb) {
				auto *body_collider =
				    static_cast<physx::PxShape *>(omni_px_->getPhysXPtr(
				        brick_info.body_collider, omni::physx::ePTShape));
				auto *top_collider =
				    static_cast<physx::PxShape *>(omni_px_->getPhysXPtr(
				        brick_info.top_collider, omni::physx::ePTShape));
				if (body_collider && top_collider) {
					bodies_[path] = rb;
					if (!graph_.addRigidBody(rb,
					                         {
					                             .body_collider = body_collider,
					                             .top_collider = top_collider,
					                         })) {
						CARB_LOG_ERROR("Failed to add rigid body for prim %s",
						               path.GetText());
					}
				} else {
					CARB_LOG_WARN("Failed to get colliders for brick prim %s",
					              path.GetText());
				}
			} else {
				CARB_LOG_WARN("Failed to get rigid body for brick prim %s",
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
				                    .overlap_xy = {conn.info.overlap_xy[0],
				                                   conn.info.overlap_xy[1]}})) {
					conns_[conn.path] = {
					    .parent_path = conn.info.parent,
					    .child_path = conn.info.child,
					    .creation_time = current_time,
					};
					conn_rev_[{rb_parent, rb_child}] = conn.path;
				} else {
					CARB_LOG_WARN("Failed to connect bodies for conn prim %s",
					              conn.path.GetText());
				}
				ps->unlockWrite();
			} else {
				CARB_LOG_WARN("Bodies for conn prim %s are in different scenes",
				              conn.path.GetText());
			}
		} else {
			CARB_LOG_WARN("Cannot create connection for conn prim %s: one "
			              "or both bodies do not exist",
			              conn.path.GetText());
		}
	}
}

LegoUsdBridge::~LegoUsdBridge() {}

void LegoUsdBridge::onRigidCreated(physx::PxRigidActor *actor,
                                   const pxr::SdfPath &primPath) {
	pxr::UsdPrim prim = stage_->GetPrimAtPath(primPath);
	BrickInfo info;
	if (!prim || !getBrickInfo(prim, info)) {
		return;
	}

	auto *body_collider = static_cast<physx::PxShape *>(
	    omni_px_->getPhysXPtr(info.body_collider, omni::physx::ePTShape));
	auto *top_collider = static_cast<physx::PxShape *>(
	    omni_px_->getPhysXPtr(info.top_collider, omni::physx::ePTShape));
	if (!body_collider || !top_collider) {
		CARB_LOG_WARN("Failed to get colliders for brick prim %s",
		              primPath.GetText());
		return;
	}

	std::lock_guard lock(mutex_);
	if (bodies_.find(primPath) != bodies_.end()) {
		CARB_LOG_WARN("Rigid body for prim %s already exists",
		              primPath.GetText());
		return;
	}
	bodies_[primPath] = actor;
	if (!graph_.addRigidBody(actor, {
	                                    .body_collider = body_collider,
	                                    .top_collider = top_collider,
	                                })) {
		CARB_LOG_ERROR("Failed to add rigid body for prim %s",
		               primPath.GetText());
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
		CARB_LOG_ERROR("Failed to remove rigid body for prim %s",
		               primPath.GetText());
	}
	ps->unlockWrite();
	bodies_.erase(it);
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
					CARB_LOG_WARN(
					    "Failed to remove reverse mapping for conn prim %s",
					    path.GetText());
				}
				auto *ps = rb_parent->getScene();
				if (ps == rb_child->getScene()) {
					ps->lockWrite();
					if (!graph_.disconnect(rb_parent, rb_child)) {
						CARB_LOG_ERROR(
						    "Failed to disconnect bodies for conn prim %s",
						    path.GetText());
					}
					ps->unlockWrite();
				} else {
					CARB_LOG_WARN(
					    "Bodies for conn prim %s are in different scenes",
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
					         .overlap_xy = {prim_info.overlap_xy[0],
					                        prim_info.overlap_xy[1]}})) {
						conns_[path] = {
						    .parent_path = prim_info.parent,
						    .child_path = prim_info.child,
						    .creation_time = current_time,
						};
						conn_rev_[{rb_parent, rb_child}] = path;
					} else {
						CARB_LOG_WARN(
						    "Failed to connect bodies for conn prim %s",
						    path.GetText());
					}
					ps->unlockWrite();
				} else {
					CARB_LOG_WARN(
					    "Bodies for conn prim %s are in different scenes",
					    path.GetText());
				}
			} else {
				CARB_LOG_WARN("Cannot create connection for conn prim %s: "
				              "one or both bodies do not exist",
				              path.GetText());
			}
		}
	}
	pendingChanges_.clear();
}

void LegoUsdBridge::onPostStep() {
	constexpr static std::uint64_t kConnDebounceTime = 10;

	pxr::SdfChangeBlock _changes;
	std::lock_guard lock(mutex_);
	auto broken_conns = graph_.solveLimits();
	auto layer = stage_->GetEditTarget().GetLayer();

	for (const auto &[a, b] : broken_conns) {
		auto it = conn_rev_.find({a, b});
		if (it == conn_rev_.end()) {
			CARB_LOG_WARN("Cannot find prim for broken connection [%p, %p]", a,
			              b);
			continue;
		}
		auto path = it->second;
		auto &conn = conns_[path];
		if (conn.creation_time > 0 && current_time < conn.creation_time + kConnDebounceTime) {
			// Ignore if just created, connections created at the beginning are not debounced
			continue;
		}
		auto attr = layer->GetAttributeAtPath(
		    path.AppendProperty(LegoTokens->conn_enabled));
		if (!attr) {
			CARB_LOG_WARN("Cannot find attribute for broken connection prim %s",
			              path.GetText());
			continue;
		}
		if (!attr->SetDefaultValue(pxr::VtValue(false))) {
			CARB_LOG_WARN("Failed to disable broken connection prim %s",
			              path.GetText());
		} else {
			CARB_LOG_INFO("Disabled broken connection prim %s", path.GetText());
		}
	}

	current_time++;
}

} // namespace lego_assemble
