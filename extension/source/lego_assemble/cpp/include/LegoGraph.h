#include "LegoWeldConstraint.h"
#include "SkipGraph.h"

#include <deque>
#include <unordered_map>
#include <unordered_set>

#include <carb/logging/Log.h>

#include <PxRigidActor.h>
#include <foundation/PxTransform.h>

namespace lego_assemble {

class LegoGraph {

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
	                       PairHash<RigidBody *, std::hash<RigidBody *>>,
	                       PairEq<RigidBody *, std::equal_to<RigidBody *>>>;

	struct BodyDesc {
		RigidBody *actor;
		std::unordered_set<ConnDesc *> parents;  // this is child
		std::unordered_set<ConnDesc *> children; // this is parent
	};
	struct ConnDesc {
		Constraint *joint;
		BodyDesc *parent;
		BodyDesc *child;
		Transform tf; // T_parent_child
	};

	LegoGraph(physx::PxPhysics *px)
	    : px_(px),
	      scheduler_(std::bind(&LegoGraph::createAuxConstraint_, this,
	                           std::placeholders::_1, std::placeholders::_2),
	                 std::bind(&LegoGraph::destroyAuxConstraint_, this,
	                           std::placeholders::_1)) {}

	LegoGraph(const LegoGraph &) = delete;

	~LegoGraph() {}

	bool addRigidBody(RigidBody *actor) {
		if (bodies_.contains(actor)) {
			return false;
		}
		BodyDesc desc;
		desc.actor = actor;
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
			c->parent->children.erase(c);
			c->joint->release();
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
			c->child->parents.erase(c);
			c->joint->release();
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
		c.joint = CreateLegoWeld(
		    *px_, a, b,
		    {.parentLocal = T_a_b, .childLocal = Transform(physx::PxIdentity)});
		auto edge_key = std::make_pair(a, b);
		conns_[edge_key] = c;
		da->children.insert(&conns_[edge_key]);
		db->parents.insert(&conns_[edge_key]);
		CARB_LOG_INFO("Created base constraint between %p and %p", a, b);
		if (!scheduler_.connect(da, db)) {
			CARB_LOG_ERROR(
			    "Failed to add edge between %p and %p to scheduler, continuing",
			    a, b);
		}
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
		c.joint->release();
		conns_.erase(itc);
		CARB_LOG_INFO("Destroyed base constraint between %p and %p",
		              c.parent->actor, c.child->actor);
		if (!scheduler_.disconnect(c.parent, c.child)) {
			CARB_LOG_ERROR("Failed to remove edge between %p and %p from "
			               "scheduler, continuing",
			               c.parent->actor, c.child->actor);
		}
		return true;
	}

  private:
	physx::PxPhysics *px_;
	BodyMap bodies_;
	ConnMap conns_;
	Scheduler scheduler_;

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
		return CreateLegoWeld(
		    *px_, a->actor, b->actor,
		    {.parentLocal = T_a_b, .childLocal = Transform(physx::PxIdentity)});
	}

	void destroyAuxConstraint_(Constraint *j) {
		if (!j) {
			CARB_LOG_FATAL("Cannot destroy null constraint");
			return;
		}
		CARB_LOG_INFO("Destroying aux constraint %p", j);
		j->release();
	}
};

}; // namespace lego_assemble
