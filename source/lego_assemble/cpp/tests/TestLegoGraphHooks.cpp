import std;
import lego_assemble.core.graph;
import lego_assemble.core.specs;
import lego_assemble.core.connections;
import lego_assemble.utils.transforms;

#include <cassert>

using namespace lego_assemble;

namespace {

// Helpers to build interface specs quickly
static Transformd Ixf() { return SE3d{}.identity(); }

static InterfaceSpec mk_stud(InterfaceId id, BrickUnit L, BrickUnit W,
                             Transformd pose = Ixf()) {
    return InterfaceSpec{.id = id,
                         .type = InterfaceType::Stud,
                         .L = L,
                         .W = W,
                         .pose = pose};
}
static InterfaceSpec mk_hole(InterfaceId id, BrickUnit L, BrickUnit W,
                             Transformd pose = Ixf()) {
    return InterfaceSpec{.id = id,
                         .type = InterfaceType::Hole,
                         .L = L,
                         .W = W,
                         .pose = pose};
}

using G = LegoGraph<PartList<CustomPart>>; // single-type graph (CustomPart)

// Build a tiny graph with two parts (pid 0 and 1) and a third isolated
template <class Graph>
static void build_three_parts(Graph &g) {
    // Part 0: two studs and two holes
    auto ifs0 = std::initializer_list<InterfaceSpec>{mk_stud(10, 2, 2),
                                                     mk_stud(12, 2, 2),
                                                     mk_hole(20, 2, 2),
                                                     mk_hole(22, 2, 2)};
    [[maybe_unused]] auto ok0 =
        g.template add_part<CustomPart>(std::tuple<>{}, 0.1, BrickColor{255, 0, 0},
                                        ifs0);
    assert(ok0);

    // Part 1
    auto ifs1 = std::initializer_list<InterfaceSpec>{mk_stud(11, 2, 2),
                                                     mk_stud(13, 2, 2),
                                                     mk_hole(21, 2, 2),
                                                     mk_hole(23, 2, 2)};
    [[maybe_unused]] auto ok1 =
        g.template add_part<CustomPart>(std::tuple<>{}, 0.2, BrickColor{0, 255, 0},
                                        ifs1);
    assert(ok1);

    // Part 2: isolated
    auto ifs2 = std::initializer_list<InterfaceSpec>{mk_stud(30, 1, 1),
                                                     mk_hole(31, 1, 1)};
    [[maybe_unused]] auto ok2 =
        g.template add_part<CustomPart>(std::tuple<>{}, 0.3, BrickColor{0, 0, 255},
                                        ifs2);
    assert(ok2);
}

// Convenience alias
static InterfaceRef IR(PartId pid, InterfaceId iid) { return {pid, iid}; }

// ---------------- Hook-capturing test subclass ----------------
struct HookGraph final : public G {
    using PW = SimplePartWrapper<CustomPart>;
    using CSW = SimpleWrapper<ConnectionSegment>;

    // Allow base to access private on_* hooks
    friend G;

  public:
    // Re-expose selected base APIs for the tests while preventing upcast
    using G::add_part;
    using G::remove_part;
    using G::connect;
    using G::disconnect;

    // Counters and observations
    int added_calls = 0;
    int removing_calls = 0;
    int connected_calls = 0;
    int disconnecting_calls = 0;

    PartId last_added_pid = PartId{0};
    bool added_pw_matches_store = false;

    bool removing_alive_in_store = false;
    bool removing_outgoing_contains = false;
    bool removing_neighbors_contains = false;

    ConnSegId last_csid{};
    ConnSegRef last_csref{};
    bool connect_bundle_has_csid = false;
    bool disconnect_bundle_has_csid = false;

  private: // hooks must be private, base is a friend
    template <class P>
    void on_part_added(PartId pid, SimplePartWrapper<P> &pw) {
        ++added_calls;
        last_added_pid = pid;
        added_pw_matches_store =
            (&pw == this->parts_.template get<SimplePartWrapper<P>>(pid));
    }

    template <class P>
    void on_part_removing(PartId pid, SimplePartWrapper<P> &pw) {
        ++removing_calls;
        removing_alive_in_store = this->parts_.alive(pid);
        removing_outgoing_contains = (pw.outgoings().size() > 0);
        removing_neighbors_contains = (pw.neighbor_parts().size() > 0);
    }

    void on_connected(ConnSegId csid, const ConnSegRef &csref,
                      const InterfaceSpec &, const InterfaceSpec &,
                      CSW &, ConnectionBundle &bundle) {
        ++connected_calls;
        last_csid = csid;
        last_csref = csref;
        connect_bundle_has_csid = bundle.conn_seg_ids.contains(csid);
    }

    void on_disconnecting(ConnSegId csid, const ConnSegRef &,
                          CSW &, ConnectionBundle &bundle) {
        ++disconnecting_calls;
        disconnect_bundle_has_csid = bundle.conn_seg_ids.contains(csid);
    }
};

// Ensure the detection variables in LegoGraph see our private hooks
static_assert(G::HasOnPartAdded<HookGraph, CustomPart>);
static_assert(G::HasOnPartRemoving<HookGraph, CustomPart>);
static_assert(G::HasOnConnected<HookGraph>);
static_assert(G::HasOnDisconnecting<HookGraph>);

// ------------------------------ tests ------------------------------

static void test_added_hook() {
    HookGraph hg;
    // Add parts one by one to verify per-call counting
    auto ifs0 = std::initializer_list<InterfaceSpec>{mk_stud(10, 2, 2),
                                                     mk_stud(12, 2, 2),
                                                     mk_hole(20, 2, 2),
                                                     mk_hole(22, 2, 2)};
    assert(hg.add_part<CustomPart>(std::tuple<>{}, 0.1, BrickColor{255, 0, 0}, ifs0));
    assert(hg.added_calls == 1);

    auto ifs1 = std::initializer_list<InterfaceSpec>{mk_stud(11, 2, 2),
                                                     mk_stud(13, 2, 2),
                                                     mk_hole(21, 2, 2),
                                                     mk_hole(23, 2, 2)};
    assert(hg.add_part<CustomPart>(std::tuple<>{}, 0.2, BrickColor{0, 255, 0}, ifs1));
    assert(hg.added_calls == 2);

    auto ifs2 = std::initializer_list<InterfaceSpec>{mk_stud(30, 1, 1),
                                                     mk_hole(31, 1, 1)};
    assert(hg.add_part<CustomPart>(std::tuple<>{}, 0.3, BrickColor{0, 0, 255}, ifs2));
    assert(hg.added_calls == 3);
    assert(hg.last_added_pid == 2);
    assert(hg.added_pw_matches_store);
}

static void test_connected_hook() {
    HookGraph hg;
    build_three_parts(hg);
    ConnectionSegment cs{}; // identity transform
    ConnSegRef ab_ref{IR(0, 10), IR(1, 21)};
    assert(hg.connect(ab_ref.first, ab_ref.second, std::tuple<>{}, cs));
    assert(hg.connected_calls == 1);
    const ConnSegId *stored_csid =
        hg.connection_segments().project<ConnSegRef, ConnSegId>(ab_ref);
    assert(stored_csid && *stored_csid == hg.last_csid);
    assert(hg.connect_bundle_has_csid);
}

static void test_removing_hook() {
    HookGraph hg;
    build_three_parts(hg);
    ConnectionSegment cs{};
    assert(hg.connect(IR(0, 10), IR(1, 21), std::tuple<>{}, cs));
    assert(hg.remove_part(PartId{0}));
    assert(hg.removing_calls >= 1);
    assert(hg.removing_alive_in_store);
    assert(hg.removing_outgoing_contains);
    assert(hg.removing_neighbors_contains);
}

static void test_disconnecting_hook() {
    HookGraph hg;
    build_three_parts(hg);
    ConnectionSegment cs{};
    ConnSegRef ab_ref{IR(0, 10), IR(1, 21)};
    assert(hg.connect(ab_ref.first, ab_ref.second, std::tuple<>{}, cs));
    assert(hg.disconnect(ab_ref));
    assert(hg.disconnecting_calls == 1);
    assert(hg.disconnect_bundle_has_csid);
}

} // namespace

int main() {
    test_added_hook();
    test_connected_hook();
    test_removing_hook();
    test_disconnecting_hook();
    return 0;
}
