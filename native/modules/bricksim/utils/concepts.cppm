export module bricksim.utils.concepts;

import std;

namespace bricksim {

export template <typename R, typename T>
concept range_of = std::ranges::range<R> &&
                   std::convertible_to<std::ranges::range_value_t<R>, T>;

template <class T> struct is_optional : std::false_type {};

template <class U> struct is_optional<std::optional<U>> : std::true_type {};

export template <class T>
constexpr bool is_optional_v = is_optional<std::remove_cvref_t<T>>::value;

export template <class Opt>
using optional_value_t = typename std::remove_cvref_t<Opt>::value_type;

} // namespace bricksim
