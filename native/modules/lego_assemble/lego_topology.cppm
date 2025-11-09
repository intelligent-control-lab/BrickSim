export module lego_assemble.lego_topology;

import std;
import lego_assemble.brick_specs;

namespace lego_assemble {

export struct LegoTopology {
	using BrickID = std::uint32_t;
	struct Brick {
		BrickID id;
		std::array<BrickUnit, 3> dimensions;
		BrickColor color;
	};
	struct Connection {
		BrickID parent;
		BrickID child;
		std::array<BrickUnit, 2> offset;
		BrickOrientation orientation;
	};
	struct PoseHint {
		BrickID brick;
		std::array<double, 3> pos; // In meters
		std::array<double, 4> rot; // Quaternion wxyz
	};

	std::vector<Brick> bricks;
	std::vector<Connection> connections;
	std::vector<PoseHint> pose_hints;
};

} // namespace lego_assemble
