export module lego_assemble.utils.ranges;

import std;

namespace lego_assemble {

export template <typename R, typename T>
concept range_of = std::ranges::range<R> &&
                   std::convertible_to<std::ranges::range_value_t<R>, T>;

}
