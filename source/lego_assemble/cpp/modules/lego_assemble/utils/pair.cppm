export module lego_assemble.utils.pair;

import std;
import lego_assemble.utils.hash;

namespace lego_assemble {

export template <class V, class Hash = std::hash<V>>
    requires hash_function<Hash, V>
struct PairHash {
	std::size_t operator()(const std::pair<V, V> &p) const
	    noexcept(noexcept(Hash{}(p.first)) && noexcept(Hash{}(p.second))) {
		std::size_t h1 = Hash{}(p.first);
		std::size_t h2 = Hash{}(p.second);
		hash_combine(h2, h1);
		return h2;
	}
};

export template <class V, class Eq = std::equal_to<>>
    requires std::equivalence_relation<Eq, const V &, const V &>
struct PairEq {
	bool operator()(const std::pair<V, V> &a, const std::pair<V, V> &b) const
	    noexcept(noexcept(Eq{}(a.first, b.first)) &&
	             noexcept(Eq{}(a.second, b.second))) {
		return Eq{}(a.first, b.first) && Eq{}(a.second, b.second);
	}
};

} // namespace lego_assemble
