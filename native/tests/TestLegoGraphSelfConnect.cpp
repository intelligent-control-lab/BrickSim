import std;
import lego_assemble.core.graph;
import lego_assemble.core.specs;
import lego_assemble.utils.transforms;
import lego_assemble.core.connections;

#include <cassert>

using namespace lego_assemble;

static InterfaceSpec stud(InterfaceId id) {
	return {.id = id,
	        .type = InterfaceType::Stud,
	        .L = 2,
	        .W = 2,
	        .pose = SE3d{}.identity()};
}
static InterfaceSpec hole(InterfaceId id) {
	return {.id = id,
	        .type = InterfaceType::Hole,
	        .L = 2,
	        .W = 2,
	        .pose = SE3d{}.identity()};
}

int main() {
	using G = LegoGraph<PartList<CustomPart>>;
	G g;

	// One part with both stud and hole
	std::initializer_list<InterfaceSpec> ifs{stud(10), hole(20)};
	assert(
	    g.add_part<CustomPart>(std::tuple<>{}, 0.1, BrickColor{0, 0, 0}, ifs));

	// Self-connection must be rejected gracefully (no value), not crash
	ConnectionSegment cs{};
	auto ok =
	    g.connect(InterfaceRef{0, 10}, InterfaceRef{0, 20}, std::tuple<>{}, cs);
	assert(!ok && "Self-connection should be rejected");
	return 0;
}
