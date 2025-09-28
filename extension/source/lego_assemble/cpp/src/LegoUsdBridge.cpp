#include "LegoUsdBridge.h"
#include "LegoTokens.h"

#include <PxScene.h>

#include <pxr/usd/usd/attribute.h>
#include <pxr/usd/usd/prim.h>
#include <pxr/usd/usd/primRange.h>
#include <pxr/usd/usd/relationship.h>

namespace lego_assemble {

LegoUsdBridge::LegoUsdBridge(pxr::UsdStageRefPtr stage, physx::PxPhysics *px,
                             omni::physx::IPhysx *omni_px)
    : omni_px_(omni_px), stage_(std::move(stage)), graph_(px) {
	loadFromStage_();
}

LegoUsdBridge::~LegoUsdBridge() {}

void LegoUsdBridge::onRigidCreated(physx::PxRigidActor *actor,
                                   const pxr::SdfPath &primPath) {
	pxr::UsdPrim prim = stage_->GetPrimAtPath(primPath);
	if (!prim || !isBrickPrim_(prim)) {
		return;
	}

	std::lock_guard lock(mutex_);
	if (bodies_.find(primPath) != bodies_.end()) {
		CARB_LOG_WARN("Rigid body for prim %s already exists",
		              primPath.GetText());
		return;
	}
	bodies_[primPath] = actor;
	if (!graph_.addRigidBody(actor)) {
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

void LegoUsdBridge::processPrimChanges() {
	std::lock_guard lock(mutex_);
	for (const auto &path : pendingChanges_) {
		auto conn_it = conns_.find(path);
		bool conn_exists = (conn_it != conns_.end());

		bool prim_valid = false;
		ConnInfo prim_info;
		pxr::UsdPrim prim = stage_->GetPrimAtPath(path);
		if (prim && getConnInfo_(prim, prim_info)) {
			prim_valid = true;
		}

		if (conn_exists && !prim_valid) {
			// Connection removed
			const auto &[parent_path, child_path] = conn_it->second;
			auto parent_it = bodies_.find(parent_path);
			auto child_it = bodies_.find(child_path);
			if (parent_it == bodies_.end() || child_it == bodies_.end()) {
				// This is a dangling connection
				// Underlying connection has already been removed
			} else {
				auto *rb_parent = parent_it->second;
				auto *rb_child = child_it->second;
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
					if (graph_.connect(rb_parent, rb_child, prim_info.tf)) {
						conns_[path] =
						    std::make_pair(prim_info.parent, prim_info.child);
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

void LegoUsdBridge::loadFromStage_() {
	std::lock_guard lock(mutex_);

	struct QueuedConn {
		pxr::SdfPath path;
		ConnInfo info;
	};

	std::vector<QueuedConn> queued_conns;
	for (const pxr::UsdPrim &prim : stage_->Traverse()) {
		ConnInfo info;
		if (getConnInfo_(prim, info)) {
			queued_conns.push_back({prim.GetPath(), info});
		} else if (isBrickPrim_(prim)) {
			auto path = prim.GetPath();
			auto *rb = static_cast<physx::PxRigidActor *>(
			    omni_px_->getPhysXPtr(path, omni::physx::ePTActor));
			if (rb) {
				bodies_[path] = rb;
				if (!graph_.addRigidBody(rb)) {
					CARB_LOG_ERROR("Failed to add rigid body for prim %s",
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
				if (graph_.connect(rb_parent, rb_child, conn.info.tf)) {
					conns_[conn.path] =
					    std::make_pair(conn.info.parent, conn.info.child);
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

bool LegoUsdBridge::isBrickPrim_(const pxr::UsdPrim &prim) const {
	return prim.IsActive() &&
	       prim.GetAttribute(LegoTokens->brick_dimensions).IsValid();
}

bool LegoUsdBridge::getConnInfo_(const pxr::UsdPrim &prim,
                                 ConnInfo &out) const {
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

	pxr::GfVec3f pos(0);
	pxr::GfQuatf rot(1, 0, 0, 0);
	prim.GetAttribute(LegoTokens->conn_pos).Get(&pos);
	prim.GetAttribute(LegoTokens->conn_rot).Get(&rot);
	out.tf = physx::PxTransform(
	    physx::PxVec3(pos[0], pos[1], pos[2]),
	    physx::PxQuat(rot.GetImaginary()[0], rot.GetImaginary()[1],
	                  rot.GetImaginary()[2], rot.GetReal()));
	return true;
}

} // namespace lego_assemble
