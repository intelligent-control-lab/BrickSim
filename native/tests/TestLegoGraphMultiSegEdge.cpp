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

	// A and B each with two studs and two holes
	std::initializer_list<InterfaceSpec> aifs{stud(10), stud(12), hole(20),
	                                          hole(22)};
	std::initializer_list<InterfaceSpec> bifs{stud(11), stud(13), hole(21),
	                                          hole(23)};
	assert(g.add_part<CustomPart>(0.1, BrickColor{1, 2, 3}, aifs));
	assert(g.add_part<CustomPart>(0.2, BrickColor{4, 5, 6}, bifs));

	ConnectionSegment cs{}; // default offset(0,0), yaw 0
	// First connection succeeds
	assert(g.connect(InterfaceRef{0, 10}, InterfaceRef{1, 21}, cs));
	// Second connection with matching transform between same endpoint must also succeed
	// If connect() unconditionally calls dynamic_graph_.add_edge() again, this will crash.
	assert(g.connect(InterfaceRef{0, 12}, InterfaceRef{1, 23}, cs));
	return 0;
}
