export module lego_assemble.physx.weld_constraint;

import std;
import lego_assemble.vendor;

using namespace physx;

namespace lego_assemble {

export struct WeldConstraintData {
	physx::PxTransform parentLocal; // joint frame in parent actor local space
	physx::PxTransform childLocal;  // joint frame in child actor local space
};

static void applyNeighborhood(PxTransform32 &a, PxTransform32 &b) {
	if (a.q.dot(b.q) < 0.0f)
		b.q = -b.q;
}

static void computeJacobianAxes(PxVec3 row[3], const PxQuat &qa,
                                const PxQuat &qb) {
	float wa = qa.w, wb = qb.w;
	PxVec3 va(qa.x, qa.y, qa.z), vb(qb.x, qb.y, qb.z);

	PxVec3 c = vb * wa + va * wb;
	float d0 = wa * wb;
	float d1 = va.dot(vb);
	float d = d0 - d1;

	row[0] = (va * vb.x + vb * va.x + PxVec3(d, c.z, -c.y)) * 0.5f;
	row[1] = (va * vb.y + vb * va.y + PxVec3(-c.z, d, c.x)) * 0.5f;
	row[2] = (va * vb.z + vb * va.z + PxVec3(c.y, -c.x, d)) * 0.5f;

	if ((d0 + d1) == 0.0f) {
		row[0].x += 1e-6f;
		row[1].y += 1e-6f;
		row[2].z += 1e-6f;
	}
}

static void fillLinear(Px1DConstraint &c, const PxVec3 &axis, const PxVec3 &ra,
                       const PxVec3 &rb, float posErr,
                       PxConstraintSolveHint::Enum hint) {
	c.solveHint = PxU16(hint);
	c.linear0 = axis;
	c.angular0 = ra.cross(axis);
	c.linear1 = axis;
	c.angular1 = rb.cross(axis);
	c.geometricError = posErr;
}

static void fillAngular(Px1DConstraint &c, const PxVec3 &axis, float posErr,
                        PxConstraintSolveHint::Enum hint) {
	c.solveHint = PxU16(hint);
	c.linear0 = PxVec3(0.0f);
	c.angular0 = axis;
	c.linear1 = PxVec3(0.0f);
	c.angular1 = axis;
	c.geometricError = posErr;
	c.flags = PxU16(c.flags | Px1DConstraintFlag::eANGULAR_CONSTRAINT);
}

static PxU32 prepareLockedAxes(Px1DConstraint *rows, const PxQuat &qA,
                               const PxQuat &qB, const PxVec3 &cB2cAp,
                               PxVec3 ra, PxVec3 rb, PxVec3 &raOut,
                               PxVec3 &rbOut) {
	float x = qA.x, y = qA.y, z = qA.z, w = qA.w;
	float x2 = x + x, y2 = y + y, z2 = z + z;
	float xx = x * x2, yy = y * y2, zz = z * z2;
	float xy = x * y2, xz = x * z2, yz = y * z2;
	float xw = x * (w + w), yw = y * (w + w), zw = z * (w + w);
	PxVec3 axis0(1.0f - yy - zz, xy + zw, xz - yw);
	PxVec3 axis1(xy - zw, 1.0f - xx - zz, yz + xw);
	PxVec3 axis2(xz + yw, yz - xw, 1.0f - xx - yy);

	PxVec3 err(0.0f);
	err -= axis0 * cB2cAp.x;
	err -= axis1 * cB2cAp.y;
	err -= axis2 * cB2cAp.z;
	ra += err;

	// 3 linear rows
	fillLinear(rows[0], axis0, ra, rb, -cB2cAp.x,
	           PxConstraintSolveHint::eEQUALITY);
	fillLinear(rows[1], axis1, ra, rb, -cB2cAp.y,
	           PxConstraintSolveHint::eEQUALITY);
	fillLinear(rows[2], axis2, ra, rb, -cB2cAp.z,
	           PxConstraintSolveHint::eEQUALITY);

	// 3 angular rows
	PxVec3 jac[3];
	computeJacobianAxes(jac, qA, qB);
	PxQuat qB2qA = qA.getConjugate() * qB;
	fillAngular(rows[3], jac[0], -qB2qA.x, PxConstraintSolveHint::eEQUALITY);
	fillAngular(rows[4], jac[1], -qB2qA.y, PxConstraintSolveHint::eEQUALITY);
	fillAngular(rows[5], jac[2], -qB2qA.z, PxConstraintSolveHint::eEQUALITY);

	raOut = ra;
	rbOut = rb;
	return 6;
}

static PxU32
WeldSolverPrep(Px1DConstraint *constraints, PxVec3p &body0WorldOffset,
               [[maybe_unused]] PxU32 maxConstraints,
               [[maybe_unused]] PxConstraintInvMassScale &invMassScale,
               const void *constantBlock, const PxTransform &bA2w,
               const PxTransform &bB2w, [[maybe_unused]] bool useExtendedLimits,
               PxVec3p &cA2wOut, PxVec3p &cB2wOut) {
	const WeldConstraintData &data =
	    *reinterpret_cast<const WeldConstraintData *>(constantBlock);

	PxTransform32 cA2w, cB2w;
	cA2w = PxTransform32(bA2w.transform(data.parentLocal));
	cB2w = PxTransform32(bB2w.transform(data.childLocal));
	applyNeighborhood(cA2w, cB2w);

	PxVec3 ra = cB2w.p - bA2w.p;
	body0WorldOffset = ra; // PhysX requires this offset
	PxVec3 rb = cB2w.p - bB2w.p;

	PxVec3 raAdj, rbAdj;
	PxU32 count =
	    prepareLockedAxes(constraints, cA2w.q, cB2w.q,
	                      cA2w.transformInv(cB2w.p), ra, rb, raAdj, rbAdj);
	cA2wOut = raAdj + bA2w.p;
	cB2wOut = rbAdj + bB2w.p;
	return count;
}

static PxConstraintShaderTable gWeldShaders = {
    .solverPrep = &WeldSolverPrep,
    .visualize = nullptr,
    .flag = PxConstraintFlag::eCOLLISION_ENABLED,
};

class WeldConstraintConnector : public PxConstraintConnector {
  public:
	WeldConstraintData data{};
	PxConstraint *constraint{nullptr};

	void *prepareData() override {
		return &data;
	}
	void onConstraintRelease() override {
		constraint = nullptr;
		delete this;
	}
	void onComShift(PxU32) override {}
	void onOriginShift(const PxVec3 &) override {}
	void *getExternalReference(PxU32 &typeID) override {
		typeID = PxConstraintExtIDs::eJOINT;
		return nullptr;
	}
	PxBase *getSerializable() override {
		return nullptr;
	}
	bool updatePvdProperties(physx::pvdsdk::PvdDataStream &,
	                         const PxConstraint *,
	                         PxPvdUpdateType::Enum) const override {
		return false;
	}
	void updateOmniPvdProperties() const override {}
	void connectToConstraint(PxConstraint *c) override {
		constraint = c;
	}
	PxConstraintSolverPrep getPrep() const override {
		return &WeldSolverPrep;
	}
	const void *getConstantBlock() const override {
		return &data;
	}
};

export PxConstraint *createWeldConstraint(PxPhysics &physics, PxRigidActor *a,
                                          PxRigidActor *b,
                                          WeldConstraintData data) {
	// TODO: allocate in PhysX's memory pool
	auto *conn = new WeldConstraintConnector();
	try {
		conn->data = std::move(data);
		PxConstraint *c = physics.createConstraint(a, b, *conn, gWeldShaders,
		                                           sizeof(WeldConstraintData));
		conn->connectToConstraint(c);
		return c;
	} catch (...) {
		delete conn;
		throw;
	}
}

} // namespace lego_assemble
