import std;
import lego_assemble.core.specs;

#include <cassert>

using namespace lego_assemble;

void test_brick() {
	BrickPart brick{2, 4, 3, {255, 0, 0}};
	std::size_t iface_count = 0;
	for_each_interface(brick, [&](const auto &iface) {
		++iface_count;
		if (iface.id == 0) {
			assert(iface.type == InterfaceType::Hole);
		} else if (iface.id == 1) {
			assert(iface.type == InterfaceType::Stud);
		} else {
			assert(false && "Unexpected interface id");
		}
	});
	assert(iface_count == 2);
}

void test_custom_part() {
	CustomPart part{
	    0.5,
	    {0, 255, 0},
	    {
	        {.id = 10,
	         .type = InterfaceType::Stud,
	         .L = 2,
	         .W = 2,
	         .pose = {{1.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 1.0}}},
	        {.id = 20,
	         .type = InterfaceType::Hole,
	         .L = 1,
	         .W = 1,
	         .pose = {{1.0, 0.0, 0.0, 0.0}, {1.0, 1.0, 0.0}}},
	    },
	};
	std::size_t iface_count = 0;
	for_each_interface(part, [&](const auto &iface) {
		++iface_count;
		if (iface.id == 10) {
			assert(iface.type == InterfaceType::Stud);
		} else if (iface.id == 20) {
			assert(iface.type == InterfaceType::Hole);
		} else {
			assert(false && "Unexpected interface id");
		}
	});
	assert(iface_count == 2);
}

int main() {
	test_brick();
}
