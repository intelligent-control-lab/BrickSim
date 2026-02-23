export module bricksim.utils.pair;

import std;
import bricksim.utils.hash;

namespace bricksim {

export template <class T, class HashT = std::hash<T>, class U = T,
                 class HashU = HashT>
    requires hash_function<HashT, T> && hash_function<HashU, U>
struct PairHash {
	std::size_t operator()(const std::pair<T, U> &p) const
	    noexcept(noexcept(HashT{}(p.first)) && noexcept(HashU{}(p.second))) {
		std::size_t h1 = HashT{}(p.first);
		std::size_t h2 = HashU{}(p.second);
		hash_combine(h2, h1);
		return h2;
	}
};

export template <class T, class EqT = std::equal_to<>, class U = T,
                 class EqU = EqT>
    requires std::equivalence_relation<EqT, const T &, const T &> &&
             std::equivalence_relation<EqU, const U &, const U &>
struct PairEq {
	bool operator()(const std::pair<T, U> &a, const std::pair<T, U> &b) const
	    noexcept(noexcept(EqT{}(a.first, b.first)) &&
	             noexcept(EqU{}(a.second, b.second))) {
		return EqT{}(a.first, b.first) && EqU{}(a.second, b.second);
	}
};

} // namespace bricksim
