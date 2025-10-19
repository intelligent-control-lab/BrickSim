#pragma once

#include "BrickSpecs.h"

#include <pxr/usd/usd/stage.h>

namespace lego_assemble {

void createBrickPrim(const pxr::UsdStageRefPtr &stage, const pxr::SdfPath &path,
                     const std::array<BrickUnit, 3> &dimensions,
                     const BrickColor &color);

}
