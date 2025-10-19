#pragma once

#include "BrickSpecs.h"

#include <memory>
#include <numbers>
#include <vector>

#include <PxRigidActor.h>
#include <PxShape.h>
#include <foundation/PxTransform.h>

namespace lego_assemble {

class LegoGraph {
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

}; // namespace lego_assemble
