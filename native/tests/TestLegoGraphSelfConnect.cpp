import std;
import bricksim.core.graph;
import bricksim.core.specs;
import bricksim.utils.transforms;
import bricksim.core.connections;

#include <cassert>

using namespace bricksim;

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
	assert(g.add_part<CustomPart>(0.1, BrickColor{0, 0, 0}, ifs));

	// Self-connection must be rejected gracefully (no value), not crash
	ConnectionSegment cs{};
	auto ok = g.connect(InterfaceRef{0, 10}, InterfaceRef{0, 20}, cs);
	assert(!ok && "Self-connection should be rejected");
	return 0;
}
