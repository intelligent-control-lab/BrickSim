export module bricksim.usd.specs;

import std;
import bricksim.core.specs;
import bricksim.vendor;

namespace bricksim {

export using InterfaceColliderPair = std::pair<InterfaceId, pxr::SdfPath>;

export using InterfaceCollidersVector = std::vector<InterfaceColliderPair>;

} // namespace bricksim
