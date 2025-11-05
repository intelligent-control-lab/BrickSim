import std;
import lego_assemble.core.graph;
import lego_assemble.core.specs;
import lego_assemble.utils.transforms;
import lego_assemble.core.connections;

#include <cassert>

using namespace lego_assemble;

namespace {

// Fail if anyone touches the default PMR.
struct FailResource : std::pmr::memory_resource {
    void *do_allocate(std::size_t, std::size_t) override {
        assert(false && "Default PMR used (should route through graph resource)!");
        std::unreachable();
    }
    void do_deallocate(void *, std::size_t, std::size_t) override {}
    bool do_is_equal(const std::pmr::memory_resource &other) const noexcept override {
        return this == &other;
    }
};

struct DefaultResourceGuard {
    std::pmr::memory_resource *prev{};
    explicit DefaultResourceGuard(std::pmr::memory_resource *r) : prev(std::pmr::set_default_resource(r)) {}
    ~DefaultResourceGuard() { std::pmr::set_default_resource(prev); }
};

// Simple helpers
static InterfaceSpec stud(InterfaceId id, BrickUnit L, BrickUnit W) {
    return {.id = id, .type = InterfaceType::Stud, .L = L, .W = W, .pose = SE3d{}.identity()};
}
static InterfaceSpec hole(InterfaceId id, BrickUnit L, BrickUnit W) {
    return {.id = id, .type = InterfaceType::Hole, .L = L, .W = W, .pose = SE3d{}.identity()};
}

} // namespace

// This test should FAIL on current code if conn_bundles_ and/or dynamic_graph_ do not use graph's PMR.
int main() {
    FailResource fail;
    DefaultResourceGuard guard(&fail); // abort if default PMR touched

    // Important: construct our arena with a non-default upstream so creating the
    // arena itself doesn't touch the default PMR (which is FailResource).
    std::pmr::unsynchronized_pool_resource arena{
        std::pmr::pool_options{}, std::pmr::new_delete_resource()};
    using G = LegoGraph<PartList<CustomPart>>;
    G g(&arena);

    // Part A (pid 0): one stud and one hole
    std::initializer_list<InterfaceSpec> aifs{stud(10, 2, 2), hole(20, 2, 2)};
    assert(g.add_part<CustomPart>(std::tuple<>{}, 0.1, BrickColor{1, 2, 3}, aifs));
    // Part B (pid 1)
    std::initializer_list<InterfaceSpec> bifs{stud(11, 2, 2), hole(21, 2, 2)};
    assert(g.add_part<CustomPart>(std::tuple<>{}, 0.2, BrickColor{4, 5, 6}, bifs));

    ConnectionSegment cs{}; // default offset (0,0)
    // If any allocation falls back to default PMR (fail), this will abort.
    assert(g.connect(InterfaceRef{0, 10}, InterfaceRef{1, 21}, std::tuple<>{}, cs));
    return 0;
}
