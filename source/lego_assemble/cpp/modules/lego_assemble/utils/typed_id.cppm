export module lego_assemble.utils.typed_id;

import std;

namespace lego_assemble {

export template <class Tag, std::integral Rep> struct TypedId {
	using rep_type = Rep;

	Rep v{};

	constexpr TypedId() noexcept = default;
	explicit constexpr TypedId(Rep x) noexcept : v(x) {}

	constexpr Rep value() const noexcept {
		return v;
	}
	explicit constexpr operator Rep() const noexcept {
		return v;
	}
	friend constexpr auto operator<=>(TypedId, TypedId) = default;
};
static_assert(std::equality_comparable<TypedId<struct A, int>>);
static_assert(std::totally_ordered<TypedId<struct A, int>>);

} // namespace lego_assemble

export namespace std {
template <class Tag, std::unsigned_integral Rep>
struct hash<lego_assemble::TypedId<Tag, Rep>> {
	size_t
	operator()(const lego_assemble::TypedId<Tag, Rep> &x) const noexcept {
		return std::hash<Rep>{}(x.value());
	}
};
template <class Tag, std::unsigned_integral Rep>
struct formatter<lego_assemble::TypedId<Tag, Rep>> : formatter<Rep> {
	auto format(const lego_assemble::TypedId<Tag, Rep> &x,
	            format_context &ctx) const {
		return formatter<Rep>::format(x.value(), ctx);
	}
};
} // namespace std
