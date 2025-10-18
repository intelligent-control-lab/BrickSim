#pragma once

#include <concepts>
#include <functional>
#include <utility>

namespace lego_assemble {

template <class Cmp, class V>
concept LessLike = requires(Cmp cmp, const V &a, const V &b) {
	{ cmp(a, b) } -> std::convertible_to<bool>;
	{ cmp(b, a) } -> std::convertible_to<bool>;
};

// ---------- Utility: hash for pair using underlying Hash ----------
template <class V, class Hash> struct PairHash {
	std::size_t operator()(const std::pair<V, V> &p) const noexcept {
		std::size_t h1 = Hash{}(p.first);
		std::size_t h2 = Hash{}(p.second);
		// A standard hash-combine (boost-like)
		h2 ^= h1 + 0x9e3779b97f4a7c15ULL + (h2 << 6) + (h2 >> 2);
		return h2;
	}
};

template <class V, class Eq> struct PairEq {
	bool operator()(const std::pair<V, V> &a,
	                const std::pair<V, V> &b) const noexcept {
		return Eq{}(a.first, b.first) && Eq{}(a.second, b.second);
	}
};

// -------- Hash combine (boost-like) --------
inline constexpr std::size_t kHashMagic = 0x9e3779b97f4a7c15ull;
inline void hash_combine(std::size_t &seed, std::size_t v) noexcept {
	seed ^= v + kHashMagic + (seed << 6) + (seed >> 2);
}

// -------- UnorderedPair --------
template <class V, class Less = std::less<V>, class Hash = std::hash<V>,
          class Eq = std::equal_to<V>>
    requires LessLike<Less, V>
struct UnorderedPair {
	V first{};
	V second{};

	// Canonicalizing constructor: ensures first <= second according to Less
	constexpr UnorderedPair(const V &a, const V &b, Less less = {})
	    : first(a), second(b) {
		if (less(second, first))
			std::swap(first, second);
	}

	// Perfect-forwarding constructor
	template <class A, class B>
	    requires std::constructible_from<V, A &&> &&
	                 std::constructible_from<V, B &&>
	constexpr UnorderedPair(A &&a, B &&b, Less less = {})
	    : first(std::forward<A>(a)), second(std::forward<B>(b)) {
		if (less(second, first))
			std::swap(first, second);
	}

	// Equality (order-insensitive because we store in canonical order)
	friend constexpr bool operator==(const UnorderedPair &x,
	                                 const UnorderedPair &y) noexcept {
		return Eq{}(x.first, y.first) && Eq{}(x.second, y.second);
	}

	// Optional convenience: does the pair contain x?
	constexpr bool contains(const V &x) const noexcept {
		return Eq{}(first, x) || Eq{}(second, x);
	}
};

// CTAD for UnorderedPair(T,T)
template <class V> UnorderedPair(V, V) -> UnorderedPair<std::decay_t<V>>;

}; // namespace lego_assemble

// std::hash specialization
namespace std {
template <class V, class Less, class Hash, class Eq>
struct hash<lego_assemble::UnorderedPair<V, Less, Hash, Eq>> {
	size_t operator()(const lego_assemble::UnorderedPair<V, Less, Hash, Eq> &p)
	    const noexcept {
		size_t h1 = Hash{}(p.first);
		size_t h2 = Hash{}(p.second);
		lego_assemble::hash_combine(h2, h1);
		return h2;
	}
};
} // namespace std
