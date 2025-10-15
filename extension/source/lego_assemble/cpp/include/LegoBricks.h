#pragma once

#include <array>
#include <cstdint>

#include <pxr/usd/sdf/path.h>
#include <pxr/usd/usd/stage.h>

namespace lego_assemble {

using BrickUnit = int32_t;

constexpr double BrickLength = 0.008;        // 8 mm per stud
constexpr double PlateHeight = 0.0032;       // 3.2 mm per plate
constexpr BrickUnit BrickHeightPerPlate = 3; // 1 brick height = 3 plates
constexpr double StudDiameter = 0.0048;      // 4.8 mm stud diameter
constexpr double StudHeight = 0.0017;        // 1.7 mm stud height

constexpr std::array<double, 3>
brickDimensionsToMeters(const std::array<BrickUnit, 3> &dimensions) {
	return {dimensions[0] * BrickLength, dimensions[1] * BrickLength,
	        dimensions[2] * PlateHeight};
}

constexpr double brickMassInKg(const std::array<BrickUnit, 3> &dimensions) {
	// Experimental formula to fit mass of arbitrary bricks
	int L = dimensions[0];
	int W = dimensions[1];
	int H = dimensions[2];
	int bricks = H / 3;
	int rem = H % 3;
	double m_brick = 0.1523 * (L * W) + 0.2498 * (L + W) - 0.2485;
	double m_plate = 0.10937 * (L * W) + 0.05671 * (L + W) - 0.02207;
	return (bricks * m_brick + rem * m_plate) / 1000;
}

using BrickColor = std::array<uint8_t, 3>; // RGB

void createBrick(const pxr::UsdStageRefPtr &stage, const pxr::SdfPath &path,
                 const std::array<BrickUnit, 3> &dimensions,
                 const BrickColor &color);

} // namespace lego_assemble
