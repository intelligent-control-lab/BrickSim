#include "LegoGraph.h"

#include "LegoWeldConstraint.h"
#include "ScenePatcher.h"
#include "SkipGraph.h"

#include <deque>
#include <unordered_map>
#include <unordered_set>

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
		Transform tf; // T_parent_child
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

	bool addRigidBody(RigidBody *actor, physx::PxShape *body_collider,
	                  physx::PxShape *top_collider) {
		if (bodies_.contains(actor)) {
			return false;
		}
		BodyDesc desc;
		desc.actor = actor;
		desc.body_collider = body_collider;
		desc.top_collider = top_collider;
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

	bool connect(RigidBody *a, RigidBody *b, const Transform &T_a_b) {
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
		c.tf = T_a_b;
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
                             physx::PxShape *body_collider,
                             physx::PxShape *top_collider) {
	return impl_->addRigidBody(actor, body_collider, top_collider);
}
bool LegoGraph::removeRigidBody(physx::PxRigidActor *actor) {
	return impl_->removeRigidBody(actor);
}
bool LegoGraph::connect(physx::PxRigidActor *a, physx::PxRigidActor *b,
                        const physx::PxTransform &T_a_b) {
	return impl_->connect(a, b, T_a_b);
}
bool LegoGraph::disconnect(physx::PxRigidActor *a, physx::PxRigidActor *b) {
	return impl_->disconnect(a, b);
}
void LegoGraph::clear() {
	impl_->clear();
}

}; // namespace lego_assemble
