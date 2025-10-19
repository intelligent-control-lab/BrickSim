#pragma once

#include "LegoBricks.h"

#include <pxr/usd/usd/stage.h>

namespace lego_assemble {

void createBrick(const pxr::UsdStageRefPtr &stage, const pxr::SdfPath &path,
                 const std::array<BrickUnit, 3> &dimensions,
                 const BrickColor &color);

}
