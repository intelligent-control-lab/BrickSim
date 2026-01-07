export module lego_assemble.utils.memory;

import std;
import lego_assemble.vendor;

namespace lego_assemble {

export template <class Ref, class V = void>
using aligned_generator =
    std::generator<Ref, V, Eigen::aligned_allocator<std::byte>>;

}
