export module bricksim.utils.unordered_pair;

import std;
import bricksim.utils.hash;
import bricksim.vendor;

namespace bricksim {

export template <class V, class Less = std::less<>, class Eq = std::equal_to<>>
    requires std::movable<V> && std::default_initializable<Less> &&
             std::strict_weak_order<Less, const V &, const V &> &&
             std::default_initializable<Eq> &&
             std::equivalence_relation<Eq, const V &, const V &>
struct UnorderedPair {
	using value_type = V;

	V first;
	V second;

	constexpr UnorderedPair(const V &a, const V &b) : first(a), second(b) {
		if (Less{}(second, first))
			std::swap(first, second);
	}

	template <class A, class B>
	    requires std::constructible_from<V, A &&> &&
	                 std::constructible_from<V, B &&>
	constexpr UnorderedPair(A &&a, B &&b)
	    : first(std::forward<A>(a)), second(std::forward<B>(b)) {
		if (Less{}(second, first))
			std::swap(first, second);
	}

	friend constexpr bool operator==(
	    const UnorderedPair &x,
	    const UnorderedPair &y) noexcept(noexcept(Eq{}(x.first, y.first)) &&
	                                     noexcept(Eq{}(x.second, y.second))) {
		return Eq{}(x.first, y.first) && Eq{}(x.second, y.second);
	}

	constexpr bool contains(const V &x) const
	    noexcept(noexcept(Eq{}(first, x)) && noexcept(Eq{}(second, x))) {
		return Eq{}(first, x) || Eq{}(second, x);
	}

	template <class HashV = std::hash<V>>
	    requires hash_function<HashV, V>
	struct Hasher {
		std::size_t operator()(const UnorderedPair &p) const
		    noexcept(noexcept(HashV{}(p.first)) &&
		             noexcept(HashV{}(p.second))) {
			std::size_t h1 = HashV{}(p.first);
			std::size_t h2 = HashV{}(p.second);
			hash_combine(h2, h1);
			return h2;
		}
	};
};

export template <class A, class B>
UnorderedPair(A, B) -> UnorderedPair<
    std::common_type_t<std::remove_cvref_t<A>, std::remove_cvref_t<B>>>;

} // namespace bricksim

namespace std {
export template <class V, class Less, class Eq>
    requires std::same_as<Eq, std::equal_to<>> &&
             bricksim::hash_function<std::hash<V>, V>
struct hash<bricksim::UnorderedPair<V, Less, Eq>> {
	using HashV = std::hash<V>;
	using Pair = bricksim::UnorderedPair<V, Less, Eq>;
	size_t operator()(const Pair &p) const
	    noexcept(noexcept(HashV{}(p.first)) && noexcept(HashV{}(p.second))) {
		return typename Pair::template Hasher<HashV>{}(p);
	}
};
} // namespace std

namespace nlohmann {
export template <class V>
struct adl_serializer<bricksim::UnorderedPair<V>> {
	using UnorderedPair = bricksim::UnorderedPair<V>;
	static UnorderedPair from_json(const nlohmann::ordered_json &j) {
		if (!j.is_array() || j.size() != 2) {
			throw std::runtime_error("UnorderedPair: expected array of size 2");
		}
		return UnorderedPair{j.at(0).get<V>(), j.at(1).get<V>()};
	}
	static void to_json(nlohmann::ordered_json &j, const UnorderedPair &p) {
		j = nlohmann::ordered_json::array({p.first, p.second});
	}
};
} // namespace nlohmann
