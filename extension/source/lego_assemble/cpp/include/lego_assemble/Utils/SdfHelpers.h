#pragma once

#include <lego_assemble/Utils/Conversions.h>

#include <initializer_list>

#include <carb/logging/Log.h>

#include <pxr/usd/sdf/attributeSpec.h>
#include <pxr/usd/sdf/primSpec.h>
#include <pxr/usd/sdf/relationshipSpec.h>
#include <pxr/usd/sdf/schema.h>

namespace lego_assemble {

template <class T>
concept SdfAttrTypeGf = pxr::GfIsGfVec<T>::value || pxr::GfIsGfQuat<T>::value ||
                        pxr::GfIsGfMatrix<T>::value;

template <class T>
pxr::SdfAttributeSpecHandle
_SetAttr(const pxr::SdfPrimSpecHandle &owner, const pxr::TfToken &name,
         const T &defaultValue, const pxr::TfToken &role = {},
         pxr::SdfVariability variability = pxr::SdfVariabilityVarying,
         bool custom = false) {
	pxr::VtValue vt(defaultValue);
	auto attr =
	    owner->GetAttributeAtPath(owner->GetPath().AppendProperty(name));
	if (!attr) {
		auto typeName = pxr::SdfSchema::GetInstance().FindType(vt, role);
		if (!typeName) {
			CARB_LOG_FATAL(
			    "SetAttr: No USD type registered for provided value.");
			return {};
		}
		attr = pxr::SdfAttributeSpec::New(owner, name, typeName, variability,
		                                  custom);
	}
	if (attr && !attr->SetDefaultValue(std::move(vt))) {
		CARB_LOG_FATAL("SetAttr: Failed to set default for '%s'.",
		               name.GetText());
	}
	return attr;
}

template <class T, typename... Args>
pxr::SdfAttributeSpecHandle SetAttr(const pxr::SdfPrimSpecHandle &owner,
                                    const pxr::TfToken &name,
                                    const T &defaultValue, Args &&...args) {
	return _SetAttr<T>(owner, name, defaultValue, std::forward<Args>(args)...);
}

template <SdfAttrTypeGf T, class V, typename... Args>
    requires as_convertible<T, V>
pxr::SdfAttributeSpecHandle SetAttr(const pxr::SdfPrimSpecHandle &owner,
                                    const pxr::TfToken &name, V &&gfLike,
                                    Args &&...args) {
	return _SetAttr<T>(owner, name, as<T>(std::forward<V>(gfLike)),
	                   std::forward<Args>(args)...);
}

template <SdfAttrTypeGf T, class U, typename... Args>
    requires as_convertible<T, std::initializer_list<U>>
pxr::SdfAttributeSpecHandle
SetAttr(const pxr::SdfPrimSpecHandle &owner, const pxr::TfToken &name,
        std::initializer_list<U> &&ilist, Args &&...args) {
	return _SetAttr<T>(owner, name, as<T>(ilist), std::forward<Args>(args)...);
}

// SetInfo for SdfSpec
template <class T>
void SetInfo(const pxr::SdfSpecHandle &spec, const pxr::TfToken &key,
             const T &value) {
	spec->SetInfo(key, pxr::VtValue(value));
}

inline pxr::SdfRelationshipSpecHandle
SetRelationship(const pxr::SdfPrimSpecHandle &owner, const pxr::TfToken &key,
                pxr::SdfPath target) {
	auto rel =
	    owner->GetRelationshipAtPath(owner->GetPath().AppendProperty(key));
	if (rel) {
		rel->GetTargetPathList().ClearEdits();
	} else {
		rel = pxr::SdfRelationshipSpec::New(owner, key);
	}
	rel->GetTargetPathList().Append(target);
	return rel;
}

// XformOp names
inline const pxr::TfToken xformOpTranslate("xformOp:translate");
inline const pxr::TfToken xformOpOrient("xformOp:orient");
inline const pxr::TfToken xformOpScale("xformOp:scale");

} // namespace lego_assemble
