import std;
import lego_assemble.core.graph;
import lego_assemble.core.specs;
import lego_assemble.core.connections;
import lego_assemble.utils.transforms;
import lego_assemble.utils.type_list;
import lego_assemble.utils.poly_store;

#include <cassert>

using namespace lego_assemble;

namespace {

// Helpers to build interface specs quickly
static Transformd Ixf() {
	return SE3d{}.identity();
}

static InterfaceSpec mk_stud(InterfaceId id, BrickUnit L, BrickUnit W,
                             Transformd pose = Ixf()) {
	return InterfaceSpec{
	    .id = id, .type = InterfaceType::Stud, .L = L, .W = W, .pose = pose};
}
static InterfaceSpec mk_hole(InterfaceId id, BrickUnit L, BrickUnit W,
                             Transformd pose = Ixf()) {
	return InterfaceSpec{
	    .id = id, .type = InterfaceType::Hole, .L = L, .W = W, .pose = pose};
}

struct Hooks;

using G =
    LegoGraph<PartList<CustomPart>, SimplePartWrapper, type_list<>, type_list<>,
              type_list<>, SimpleWrapper<ConnectionSegment>, type_list<>,
              type_list<>, type_list<>, SimpleWrapper<ConnectionBundle>,
              Hooks>; // single-type graph (CustomPart)

struct Hooks {
	using PW = SimplePartWrapper<CustomPart>;
	using CSW = SimpleWrapper<ConnectionSegment>;
	using CBW = SimpleWrapper<ConnectionBundle>;

	G *g = nullptr;

	// Counters and observations
	int added_calls = 0;
	int removing_calls = 0;
	int connected_calls = 0;
	int disconnecting_calls = 0;

	int bundle_created_calls = 0;
	int bundle_removing_calls = 0;

	ConnectionEndpoint last_created_ep{0, 0};
	ConnectionEndpoint last_removed_ep{0, 0};
	std::size_t created_bundle_size = 0;
	std::size_t removing_bundle_size = 0;

	PartId last_added_pid = PartId{0};
	bool added_pw_matches_store = false;

	bool removing_alive_in_store = false;
	bool removing_outgoing_contains = false;
	bool removing_neighbors_contains = false;

	ConnSegId last_csid{};
	ConnSegRef last_csref{};
	bool connect_bundle_has_csid = false;
	bool disconnect_bundle_has_csid = false;

	template <class P>
	void on_part_added(PartId pid, SimplePartWrapper<P> &pw) {
		++added_calls;
		last_added_pid = pid;
		added_pw_matches_store =
		    (&pw == g->parts().template find_value<SimplePartWrapper<P>>(pid));
	}

	template <class P>
	void on_part_removing(PartId pid, SimplePartWrapper<P> &pw) {
		++removing_calls;
		removing_alive_in_store = g->parts().contains(pid);
		removing_outgoing_contains = (pw.outgoings().size() > 0);
		removing_neighbors_contains = (pw.neighbor_parts().size() > 0);
	}

	void on_connected(ConnSegId csid, const ConnSegRef &csref,
	                  const InterfaceSpec &, const InterfaceSpec &, CSW &,
	                  CBW &cbw) {
		++connected_calls;
		last_csid = csid;
		last_csref = csref;
		connect_bundle_has_csid = cbw.wrapped().conn_seg_ids.contains(csid);
	}

	void on_disconnecting(ConnSegId csid, const ConnSegRef &, CSW &, CBW &cbw) {
		++disconnecting_calls;
		disconnect_bundle_has_csid = cbw.wrapped().conn_seg_ids.contains(csid);
	}

	void on_bundle_created(const ConnectionEndpoint &ep, CBW &cbw) {
		++bundle_created_calls;
		last_created_ep = ep;
		created_bundle_size = cbw.wrapped().conn_seg_ids.size();
	}

	void on_bundle_removing(const ConnectionEndpoint &ep, CBW &cbw) {
		++bundle_removing_calls;
		last_removed_ep = ep;
		removing_bundle_size = cbw.wrapped().conn_seg_ids.size();
	}
};
static_assert(G::HasAllOnPartAddedHooks);
static_assert(G::HasAllOnPartRemovingHooks);
static_assert(G::HasOnConnectedHook);
static_assert(G::HasOnDisconnectingHook);
static_assert(G::HasOnBundleCreatedHook);
static_assert(G::HasOnBundleRemovingHook);

// Build a tiny graph with two parts (pid 0 and 1) and a third isolated
template <class Graph> static void build_three_parts(Graph &g) {
	// Part 0: two studs and two holes
	auto ifs0 = std::initializer_list<InterfaceSpec>{
	    mk_stud(10, 2, 2), mk_stud(12, 2, 2), mk_hole(20, 2, 2),
	    mk_hole(22, 2, 2)};
	[[maybe_unused]] auto ok0 =
	    g.template add_part<CustomPart>(0.1, BrickColor{255, 0, 0}, ifs0);
	assert(ok0);

	// Part 1
	auto ifs1 = std::initializer_list<InterfaceSpec>{
	    mk_stud(11, 2, 2), mk_stud(13, 2, 2), mk_hole(21, 2, 2),
	    mk_hole(23, 2, 2)};
	[[maybe_unused]] auto ok1 =
	    g.template add_part<CustomPart>(0.2, BrickColor{0, 255, 0}, ifs1);
	assert(ok1);

	// Part 2: isolated
	auto ifs2 = std::initializer_list<InterfaceSpec>{mk_stud(30, 1, 1),
	                                                 mk_hole(31, 1, 1)};
	[[maybe_unused]] auto ok2 =
	    g.template add_part<CustomPart>(0.3, BrickColor{0, 0, 255}, ifs2);
	assert(ok2);
}

// Convenience alias
static InterfaceRef IR(PartId pid, InterfaceId iid) {
	return {pid, iid};
}

// ------------------------------ tests ------------------------------

static void test_added_hook() {
	Hooks hooks;
	G g{&hooks};
	hooks.g = &g;
	// Add parts one by one to verify per-call counting
	auto ifs0 = std::initializer_list<InterfaceSpec>{
	    mk_stud(10, 2, 2), mk_stud(12, 2, 2), mk_hole(20, 2, 2),
	    mk_hole(22, 2, 2)};
	assert(g.add_part<CustomPart>(0.1, BrickColor{255, 0, 0}, ifs0));
	assert(hooks.added_calls == 1);

	auto ifs1 = std::initializer_list<InterfaceSpec>{
	    mk_stud(11, 2, 2), mk_stud(13, 2, 2), mk_hole(21, 2, 2),
	    mk_hole(23, 2, 2)};
	assert(g.add_part<CustomPart>(0.2, BrickColor{0, 255, 0}, ifs1));
	assert(hooks.added_calls == 2);

	auto ifs2 = std::initializer_list<InterfaceSpec>{mk_stud(30, 1, 1),
	                                                 mk_hole(31, 1, 1)};
	assert(g.add_part<CustomPart>(0.3, BrickColor{0, 0, 255}, ifs2));
	assert(hooks.added_calls == 3);
	assert(hooks.last_added_pid == 2);
	assert(hooks.added_pw_matches_store);
}

static void test_connected_hook() {
	Hooks hooks;
	G g{&hooks};
	hooks.g = &g;
	build_three_parts(g);
	ConnectionSegment cs{}; // identity transform
	ConnSegRef ab_ref{IR(0, 10), IR(1, 21)};
	assert(g.connect(ab_ref.first, ab_ref.second, cs));
	assert(hooks.connected_calls == 1);
	const ConnSegId *stored_csid =
	    g.connection_segments().find_key<ConnSegId>(ab_ref);
	assert(stored_csid && *stored_csid == hooks.last_csid);
	assert(hooks.connect_bundle_has_csid);
}

static void test_bundle_created_hook() {
	Hooks hooks;
	G g{&hooks};
	hooks.g = &g;
	build_three_parts(g);
	ConnectionSegment cs{};
	// first connection between part 0 and 1 creates a bundle
	assert(g.connect(IR(0, 10), IR(1, 21), cs));
	assert(hooks.bundle_created_calls == 1);
	assert((hooks.last_created_ep == ConnectionEndpoint{0, 1}));
	assert(hooks.created_bundle_size == 1);

	// second connection between the same pair should NOT create a new bundle
	assert(g.connect(IR(0, 12), IR(1, 23), cs));
	assert(hooks.bundle_created_calls == 1);
}

static void test_removing_hook() {
	Hooks hooks;
	G g{&hooks};
	hooks.g = &g;
	build_three_parts(g);
	ConnectionSegment cs{};
	assert(g.connect(IR(0, 10), IR(1, 21), cs));
	assert(g.remove_part(PartId{0}));
	assert(hooks.removing_calls >= 1);
	assert(hooks.removing_alive_in_store);
	assert(hooks.removing_outgoing_contains);
	assert(hooks.removing_neighbors_contains);
}

static void test_disconnecting_hook() {
	Hooks hooks;
	G g{&hooks};
	hooks.g = &g;
	build_three_parts(g);
	ConnectionSegment cs{};
	ConnSegRef ab_ref{IR(0, 10), IR(1, 21)};
	assert(g.connect(ab_ref.first, ab_ref.second, cs));
	assert(g.disconnect(ab_ref));
	assert(hooks.disconnecting_calls == 1);
	assert(hooks.disconnect_bundle_has_csid);
}

static void test_bundle_removing_on_disconnect_last() {
	Hooks hooks;
	G g{&hooks};
	hooks.g = &g;
	build_three_parts(g);
	ConnectionSegment cs{};
	// create two segments between 0 and 1 so bundle has size 2
	assert(g.connect(IR(0, 10), IR(1, 21), cs));
	assert(g.connect(IR(0, 12), IR(1, 23), cs));

	// remove one segment: bundle should remain, no removing callback yet
	assert(g.disconnect(ConnSegRef{IR(0, 10), IR(1, 21)}));
	assert(hooks.bundle_removing_calls == 0);

	// remove the last segment: now bundle is removed and callback fires
	assert(g.disconnect(ConnSegRef{IR(0, 12), IR(1, 23)}));
	assert(hooks.bundle_removing_calls == 1);
	assert((hooks.last_removed_ep == ConnectionEndpoint{0, 1}));
	// New convention: on_bundle_removing is called BEFORE erasing the last
	// segment from the bundle when disconnecting explicitly, so size is 1.
	assert(hooks.removing_bundle_size == 1);
}

static void test_bundle_removing_on_remove_part() {
	Hooks hooks;
	G g{&hooks};
	hooks.g = &g;
	build_three_parts(g);
	ConnectionSegment cs{};
	// create two segments between 0 and 1 so bundle has size 2
	assert(g.connect(IR(0, 10), IR(1, 21), cs));
	assert(g.connect(IR(0, 12), IR(1, 23), cs));

	// removing part 0 should trigger bundle_removing once for ep {0,1}
	assert(g.remove_part(PartId{0}));
	assert(hooks.bundle_removing_calls == 1);
	assert((hooks.last_removed_ep == ConnectionEndpoint{0, 1}));
	// At removal-by-part time, the bundle still holds its segments
	assert(hooks.removing_bundle_size == 2);
}

} // namespace

int main() {
	test_added_hook();
	test_connected_hook();
	test_bundle_created_hook();
	test_removing_hook();
	test_disconnecting_hook();
	test_bundle_removing_on_disconnect_last();
	test_bundle_removing_on_remove_part();
	return 0;
}
