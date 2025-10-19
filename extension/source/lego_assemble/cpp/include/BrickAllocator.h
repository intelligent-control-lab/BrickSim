#pragma once

#include "BrickNaming.h"
#include "BrickSpecs.h"

#include <pxr/usd/usd/stage.h>

namespace lego_assemble {

std::tuple<BrickId, pxr::SdfPath>
allocateBrick(const pxr::UsdStageRefPtr &stage,
              const std::array<BrickUnit, 3> &dimensions,
              const BrickColor &color, EnvId env_id);

bool deallocateBrick(const pxr::UsdStageRefPtr &stage, EnvId env_id,
                     BrickId brick_id);

void deallocateAllBricksInEnv(const pxr::UsdStageRefPtr &stage, EnvId env_id);

}; // namespace lego_assemble
