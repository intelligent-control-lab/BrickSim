export module bricksim.utils.hash;

import std;

namespace bricksim {

export template <class H, class V>
concept hash_function =
    std::regular_invocable<const std::remove_reference_t<H> &,
                           const std::remove_cvref_t<V> &> &&
    std::convertible_to<std::invoke_result_t<const std::remove_reference_t<H> &,
                                             const std::remove_cvref_t<V> &>,
                        std::size_t>;

export constexpr void hash_combine(std::size_t &seed, std::size_t v) noexcept {
	seed ^= v + 0x9e3779b97f4a7c15ull + (seed << 6) + (seed >> 2);
}

} // namespace bricksim
