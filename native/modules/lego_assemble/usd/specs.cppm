export module lego_assemble.usd.specs;

import std;
import lego_assemble.core.specs;
import lego_assemble.vendor.pxr;

namespace lego_assemble {

export using InterfaceColliderPair = std::pair<InterfaceId, pxr::SdfPath>;

export using InterfaceCollidersVector = std::vector<InterfaceColliderPair>;

} // namespace lego_assemble
