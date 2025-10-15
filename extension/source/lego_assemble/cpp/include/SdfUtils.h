#pragma once

#include <carb/logging/Log.h>

#include <array>
#include <ranges>
#include <string>
#include <tuple>
#include <type_traits>

#include <pxr/usd/sdf/attributeSpec.h>
#include <pxr/usd/sdf/schema.h>

namespace lego_assemble {

template <class T>
concept GfVecLike = requires {
	typename T::ScalarType;
	{ T::dimension } -> std::convertible_to<int>;
};

template <class R, class Scalar>
concept RangeOf = std::ranges::sized_range<R> &&
                  std::convertible_to<std::ranges::range_value_t<R>, Scalar>;

// Try to detect a fixed compile-time size (std::array, std::span<T,N>, etc.)
template <class R>
concept HasTupleSize =
    requires { std::tuple_size<std::remove_reference_t<R>>::value; };

// --- Convert any sized range to a GfVec-like type ----------------------------

template <GfVecLike GfVecT, class R>
    requires RangeOf<R, typename GfVecT::ScalarType>
GfVecT ToGfVec(R &&r) {
	using Scalar = typename GfVecT::ScalarType;
	constexpr std::size_t N = static_cast<std::size_t>(GfVecT::dimension);

	if constexpr (HasTupleSize<R>) {
		static_assert(
		    std::tuple_size_v<std::remove_reference_t<R>> == N,
		    "Dimension mismatch between source container and GfVec type");
	}

	const auto sz = std::ranges::size(r);
	if (sz != N) {
		CARB_LOG_FATAL("ToGfVec: expected %zu elements, got %zu.", N,
		               static_cast<std::size_t>(sz));
	}

	GfVecT out{};
	auto it = std::ranges::begin(r);
	for (std::size_t i = 0; i < N && it != std::ranges::end(r); ++i, ++it) {
		out[i] = static_cast<Scalar>(*it);
	}
	return out;
}

// --- Base NewAttr: exact USD type already provided ---------------------------

template <class T>
pxr::SdfAttributeSpecHandle
NewAttr(const pxr::SdfPrimSpecHandle &owner, const std::string &name,
        const T &defaultValue, const pxr::TfToken &role = {},
        pxr::SdfVariability variability = pxr::SdfVariabilityVarying,
        bool custom = false) {
	pxr::VtValue vt(defaultValue);
	auto typeName = pxr::SdfSchema::GetInstance().FindType(vt, role);
	if (!typeName) {
		CARB_LOG_FATAL("NewAttr: No USD type registered for provided value.");
		return {};
	}
	auto attr =
	    pxr::SdfAttributeSpec::New(owner, name, typeName, variability, custom);
	if (attr && !attr->SetDefaultValue(std::move(vt))) {
		CARB_LOG_FATAL("NewAttr: Failed to set default for '%s'.",
		               name.c_str());
	}
	return attr;
}

// --- Convenience: pass any sized range; we convert to T (a GfVec) for you ----

template <GfVecLike T, class R>
    requires RangeOf<R, typename T::ScalarType>
pxr::SdfAttributeSpecHandle
NewAttr(const pxr::SdfPrimSpecHandle &owner, const std::string &name,
        R &&rangeLike, const pxr::TfToken &role = {},
        pxr::SdfVariability variability = pxr::SdfVariabilityVarying,
        bool custom = false) {
	return NewAttr<T>(owner, name, ToGfVec<T>(std::forward<R>(rangeLike)), role,
	                  variability, custom);
}

// Optional: make braced lists `{...}` unambiguous (initializer_list is a range,
// but this overload keeps diagnostics nicer and avoids rare overload ties).
template <GfVecLike T, class U>
pxr::SdfAttributeSpecHandle
NewAttr(const pxr::SdfPrimSpecHandle &owner, const std::string &name,
        std::initializer_list<U> ilist, const pxr::TfToken &role = {},
        pxr::SdfVariability variability = pxr::SdfVariabilityVarying,
        bool custom = false) {
	return NewAttr<T>(owner, name, ToGfVec<T>(ilist), role, variability,
	                  custom);
}

// SetInfo for SdfSpec
template <class T>
void SetInfo(const pxr::SdfSpecHandle &spec, const pxr::TfToken &key,
             const T &value) {
	spec->SetInfo(key, pxr::VtValue(value));
}

// XformOp names
inline const pxr::TfToken xformOpTranslate("xformOp:translate");
inline const pxr::TfToken xformOpOrient("xformOp:orient");
inline const pxr::TfToken xformOpScale("xformOp:scale");

} // namespace lego_assemble
