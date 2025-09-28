#include "LegoJointManager.h"
#include "LegoGraph.h"
#include "ScenePatcher.h"
#include "tokens.h"

#include <PxPhysicsAPI.h>

#include <carb/InterfaceUtils.h>
#include <carb/events/EventsUtils.h>
#include <carb/logging/Log.h>

#include <pxr/usd/sdf/path.h>
#include <pxr/usd/sdf/pathTable.h>
#include <pxr/usd/usd/attribute.h>
#include <pxr/usd/usd/prim.h>
#include <pxr/usd/usd/primRange.h>
#include <pxr/usd/usd/relationship.h>
#include <pxr/usd/usdUtils/stageCache.h>

#include <omni/kit/IStageUpdate.h>
#include <omni/physx/IPhysx.h>

namespace lego_assemble {

class LegoState {
  public:
	LegoState(pxr::UsdStageRefPtr stage, physx::PxPhysics *px,
	          omni::physx::IPhysx *omni_px)
	    : omni_px_(omni_px), stage_(std::move(stage)), graph_(px) {
		loadFromStage_();
	}

	~LegoState() {}

	void onRigidCreated(physx::PxRigidActor *actor,
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

	void onRigidDestroyed(physx::PxRigidActor *actor,
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

	void enqueuePrimChange(const pxr::SdfPath &primPath) {
		std::lock_guard lock(mutex_);
		pendingChanges_.push_back(primPath);
	}

	void processPrimChanges() {
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
							conns_[path] = std::make_pair(prim_info.parent,
							                              prim_info.child);
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

  private:
	struct ConnInfo {
		pxr::SdfPath parent;
		pxr::SdfPath child;
		physx::PxTransform tf;
	};

	omni::physx::IPhysx *omni_px_;
	pxr::UsdStageRefPtr stage_;

	pxr::SdfPathTable<physx::PxRigidActor *> bodies_;
	pxr::SdfPathTable<std::pair<pxr::SdfPath, pxr::SdfPath>> conns_;
	LegoGraph graph_;
	std::vector<pxr::SdfPath> pendingChanges_;
	std::mutex mutex_;

	void loadFromStage_() {
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
						CARB_LOG_WARN(
						    "Failed to connect bodies for conn prim %s",
						    conn.path.GetText());
					}
					ps->unlockWrite();
				} else {
					CARB_LOG_WARN(
					    "Bodies for conn prim %s are in different scenes",
					    conn.path.GetText());
				}
			} else {
				CARB_LOG_WARN("Cannot create connection for conn prim %s: one "
				              "or both bodies do not exist",
				              conn.path.GetText());
			}
		}
	}

	bool isBrickPrim_(const pxr::UsdPrim &prim) const {
		return prim.IsActive() &&
		       prim.GetAttribute(LegoTokens->brick_dimensions).IsValid();
	}

	bool getConnInfo_(const pxr::UsdPrim &prim, ConnInfo &out) const {
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
};

static omni::physx::IPhysx *g_physx = nullptr;
static std::unique_ptr<LegoState> g_state;
static long int g_stageId = -1;

static omni::physx::SubscriptionId g_physxObjSub =
    omni::physx::kInvalidSubscriptionId;
static omni::physx::SubscriptionId g_physxObjFullSub =
    omni::physx::kInvalidSubscriptionId;
static omni::physx::SubscriptionId g_preStepSub =
    omni::physx::kInvalidSubscriptionId;
static omni::kit::StageUpdatePtr g_stageUpdate;
static omni::kit::StageUpdateNode *g_stageUpdateNode = nullptr;

bool initLegoJointManager() {
	g_physx = carb::getCachedInterface<omni::physx::IPhysx>();
	if (!g_physx) {
		CARB_LOG_ERROR("IPhysx unavailable");
		return false;
	}

	// ==== Subscribe to PhysX object creation/destruction
	omni::physx::IPhysicsObjectChangeCallback physxObjCb;
	physxObjCb.objectCreationNotifyFn =
	    [](const pxr::SdfPath &sdfPath,
	       omni::physx::usdparser::ObjectId objectId,
	       omni::physx::PhysXType type, void *userData) {
		    if (!g_state)
			    return;
		    if (type == omni::physx::PhysXType::ePTActor) {
			    auto *actor = static_cast<physx::PxRigidActor *>(
			        g_physx->getPhysXPtrFast(objectId));
			    if (actor) {
				    g_state->onRigidCreated(actor, sdfPath);
			    }
		    }
	    };
	physxObjCb.objectDestructionNotifyFn =
	    [](const pxr::SdfPath &sdfPath,
	       omni::physx::usdparser::ObjectId objectId,
	       omni::physx::PhysXType type, void *userData) {
		    if (!g_state)
			    return;
		    if (type == omni::physx::PhysXType::ePTActor) {
			    auto *actor = static_cast<physx::PxRigidActor *>(
			        g_physx->getPhysXPtrFast(objectId));
			    if (actor) {
				    g_state->onRigidDestroyed(actor, sdfPath);
			    }
		    }
	    };
	physxObjCb.allObjectsDestructionNotifyFn = [](void *userData) {
		// Called on release_physics_objects()
		if (g_state) {
			g_state = nullptr;
			CARB_LOG_INFO("Lego state destroyed");
		}
	};
	physxObjCb.stopCallbackWhenSimStopped = true;
	g_physxObjSub = g_physx->subscribeObjectChangeNotifications(physxObjCb);

	// ==== Subscribe to ALL PhysX object creation/destruction
	omni::physx::IPhysicsObjectChangeCallback physxObjFullCb;
	physxObjFullCb.objectCreationNotifyFn =
	    [](const pxr::SdfPath &sdfPath,
	       omni::physx::usdparser::ObjectId objectId,
	       omni::physx::PhysXType type, void *userData) {
		    if (type == omni::physx::PhysXType::ePTScene) {
			    auto *scene = static_cast<physx::PxScene *>(
			        g_physx->getPhysXPtrFast(objectId));
			    if (scene) {
				    if (patchPxScene(scene)) {
					    CARB_LOG_INFO("PxScene %p patched successfully", scene);
				    } else {
					    CARB_LOG_ERROR("Failed to patch PxScene %p", scene);
				    }
			    }
		    }
	    };
	physxObjFullCb.stopCallbackWhenSimStopped = false;
	g_physxObjFullSub =
	    g_physx->subscribeObjectChangeNotifications(physxObjFullCb);

	// ==== Subscribe to PhysX pre-step event
	if (g_preStepSub == omni::physx::kInvalidSubscriptionId) {
		g_preStepSub = g_physx->subscribePhysicsOnStepEvents(
		    true, 0,
		    [](float elapsedTime, void *userData) {
			    if (!g_state) {
				    auto *px = static_cast<physx::PxPhysics *>(
				        g_physx->getPhysXPtr({}, omni::physx::ePTPhysics));
				    if (!px) {
					    CARB_LOG_FATAL("ePTPhysics is nullptr");
					    return;
				    }
				    auto stageRef = pxr::UsdUtilsStageCache::Get().Find(
				        pxr::UsdStageCache::Id::FromLongInt(g_stageId));
				    if (!stageRef) {
					    CARB_LOG_FATAL(
					        "Failed to get stage from stage cache (pre-step)");
					    return;
				    }
				    g_state = std::make_unique<LegoState>(std::move(stageRef),
				                                          px, g_physx);
				    CARB_LOG_INFO("Lego state created (pre-step)");
			    }
			    g_state->processPrimChanges();
		    },
		    nullptr);
	}

	// ==== Subscribe to stage update events (stage attach/detach + USD edits)
	auto *suIface = carb::getCachedInterface<omni::kit::IStageUpdate>();
	if (!suIface) {
		CARB_LOG_ERROR("IStageUpdate unavailable");
		return false;
	}
	g_stageUpdate = suIface->getStageUpdate();
	if (!g_stageUpdate) {
		CARB_LOG_ERROR("StageUpdate is nullptr");
		return false;
	}
	omni::kit::StageUpdateNodeDesc desc = {nullptr};
	desc.displayName = "lego_assemble";
	desc.userData = nullptr;
	desc.order = 200; // After almost all other nodes, before debug (1000)
	desc.onAttach = [](long stageId, double metersPerUnit, void *userData) {
		// Called when USD attach
		g_stageId = stageId;
	};
	desc.onDetach = [](void *userData) {
		// Called when USD detach
		g_stageId = -1;
		if (g_state) {
			g_state = nullptr;
			CARB_LOG_INFO("Lego state destroyed (onDetach)");
		}
	};
	desc.onStop = [](void *userData) {
		// Called when simulation stops
		if (g_state) {
			g_state = nullptr;
			CARB_LOG_INFO("Lego state destroyed (onStop)");
		}
	};
	desc.onPrimAdd = [](const pxr::SdfPath &primPath, void *userData) {
		if (g_state) {
			g_state->enqueuePrimChange(primPath);
		}
	};
	desc.onPrimOrPropertyChange = [](const pxr::SdfPath &primOrPropertyPath,
	                                 void *userData) {
		if (g_state) {
			g_state->enqueuePrimChange(primOrPropertyPath);
		}
	};
	desc.onPrimRemove = [](const pxr::SdfPath &primPath, void *userData) {
		if (g_state) {
			g_state->enqueuePrimChange(primPath);
		}
	};
	g_stageUpdateNode = g_stageUpdate->createStageUpdateNode(desc);

	return true;
}

bool deinitLegoJointManager() {
	bool success = true;

	// ==== Unsubscribe from stage update events
	if (g_stageUpdate && g_stageUpdateNode) {
		g_stageUpdate->destroyStageUpdateNode(g_stageUpdateNode);
	} else {
		CARB_LOG_WARN("Cannot destroy StageUpdateNode");
		success = false;
	}
	g_stageUpdateNode = nullptr;
	g_stageUpdate = nullptr;

	if (g_physx) {
		// ==== Unsubscribe from PhysX pre-step event
		if (g_preStepSub != omni::physx::kInvalidSubscriptionId) {
			g_physx->unsubscribePhysicsOnStepEvents(g_preStepSub);
			g_preStepSub = omni::physx::kInvalidSubscriptionId;
		} else {
			CARB_LOG_WARN("Pre-step event not subscribed");
			success = false;
		}

		// ==== Unsubscribe from PhysX object creation/destruction
		if (g_physxObjSub != omni::physx::kInvalidSubscriptionId) {
			g_physx->unsubscribeObjectChangeNotifications(g_physxObjSub);
			g_physxObjSub = omni::physx::kInvalidSubscriptionId;
		} else {
			CARB_LOG_WARN("PhysX object change not subscribed");
			success = false;
		}

		// ==== Unsubscribe from ALL PhysX object creation/destruction
		if (g_physxObjFullSub != omni::physx::kInvalidSubscriptionId) {
			g_physx->unsubscribeObjectChangeNotifications(g_physxObjFullSub);
			g_physxObjFullSub = omni::physx::kInvalidSubscriptionId;
		} else {
			CARB_LOG_WARN("PhysX full object change not subscribed");
			success = false;
		}
	} else {
		CARB_LOG_ERROR("IPhysx unavailable");
		success = false;
	}

	g_state = nullptr;
	g_physx = nullptr;
	return success;
}

} // namespace lego_assemble
