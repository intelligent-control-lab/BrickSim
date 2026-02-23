export module bricksim.utils.sdf_path_map;

import std;
import bricksim.vendor;

namespace bricksim {

export template <class M>
concept SdfPathOrderedMap = std::same_as<typename M::key_type, pxr::SdfPath> &&
                            requires(M &m, const pxr::SdfPath &p) {
	                            {
		                            m.lower_bound(p)
	                            }; // std::map / std::multimap style
	                            { m.end() };
                            };

export template <SdfPathOrderedMap M>
auto subtree_range(M &map, const pxr::SdfPath &prefix) {
	using It = decltype(map.lower_bound(prefix));

	// Special case: empty prefix => whole map
	if (prefix.IsEmpty()) {
		return std::ranges::subrange(map.begin(), map.end());
	}

	It first = map.lower_bound(prefix);
	It last = first;

	// All paths are lexicographically ordered; all with the prefix are
	// contiguous, so we can stop at the first one that doesn't match.
	while (last != map.end() && last->first.HasPrefix(prefix)) {
		++last;
	}

	return std::ranges::subrange(first, last);
}

} // namespace bricksim
