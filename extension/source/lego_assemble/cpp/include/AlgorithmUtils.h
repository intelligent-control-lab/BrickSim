#pragma once

#include <concepts>
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

}; // namespace lego_assemble
