export module bricksim.utils.memory;

import std;
import bricksim.vendor;

namespace bricksim {

export template <class Ref, class V = void>
using aligned_generator =
    std::generator<Ref, V, Eigen::aligned_allocator<std::byte>>;

}
