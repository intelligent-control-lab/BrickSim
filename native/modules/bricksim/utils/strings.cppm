export module bricksim.utils.strings;

import std;

namespace bricksim {

export template <class Int>
constexpr std::optional<Int> parse_int(std::string_view sv) {
	Int value{};
	const char *first = sv.data();
	const char *last = sv.data() + sv.size();
	auto [ptr, ec] = std::from_chars(first, last, value, 10);
	if (ec == std::errc{} && ptr == last) {
		return value; // parsed all characters successfully
	}
	return std::nullopt;
}

export constexpr bool eat_prefix(std::string_view &sv,
                                 std::string_view prefix) {
	if (!sv.starts_with(prefix))
		return false;
	sv.remove_prefix(prefix.size());
	return true;
}

} // namespace bricksim
