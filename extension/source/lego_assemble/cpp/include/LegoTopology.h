#pragma once

#include "BrickSpecs.h"

#include <pxr/usd/sdf/path.h>
#include <pxr/usd/usd/stage.h>

namespace lego_assemble {

struct LegoTopology {
	using BrickID = uint32_t;
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

LegoTopology exportLegoTopology(const pxr::UsdStageRefPtr &stage,
                                const pxr::SdfPath &rootPath);

} // namespace lego_assemble
