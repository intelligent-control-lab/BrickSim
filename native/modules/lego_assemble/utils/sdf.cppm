export module lego_assemble.utils.sdf;

import std;
import lego_assemble.utils.conversions;
import lego_assemble.vendor.carb;
import lego_assemble.vendor.pxr;

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
			log_fatal("SetAttr: No USD type registered for provided value.");
			return {};
		}
		attr = pxr::SdfAttributeSpec::New(owner, name, typeName, variability,
		                                  custom);
	}
	if (attr && !attr->SetDefaultValue(std::move(vt))) {
		log_fatal("SetAttr: Failed to set default for '{}'.", name.GetText());
	}
	return attr;
}

export template <class T, typename... Args>
pxr::SdfAttributeSpecHandle SetAttr(const pxr::SdfPrimSpecHandle &owner,
                                    const pxr::TfToken &name,
                                    const T &defaultValue, Args &&...args) {
	return _SetAttr<T>(owner, name, defaultValue, std::forward<Args>(args)...);
}

export template <SdfAttrTypeGf T, class V, typename... Args>
    requires as_convertible<T, V>
pxr::SdfAttributeSpecHandle SetAttr(const pxr::SdfPrimSpecHandle &owner,
                                    const pxr::TfToken &name, V &&gfLike,
                                    Args &&...args) {
	return _SetAttr<T>(owner, name, as<T>(std::forward<V>(gfLike)),
	                   std::forward<Args>(args)...);
}

export template <SdfAttrTypeGf T, class U, typename... Args>
    requires as_convertible<T, std::initializer_list<U>>
pxr::SdfAttributeSpecHandle
SetAttr(const pxr::SdfPrimSpecHandle &owner, const pxr::TfToken &name,
        std::initializer_list<U> &&ilist, Args &&...args) {
	return _SetAttr<T>(owner, name, as<T>(ilist), std::forward<Args>(args)...);
}

// SetInfo for SdfSpec
export template <class T>
void SetInfo(const pxr::SdfSpecHandle &spec, const pxr::TfToken &key,
             const T &value) {
	spec->SetInfo(key, pxr::VtValue(value));
}

export pxr::SdfRelationshipSpecHandle
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
export const pxr::TfToken xformOpTranslate("xformOp:translate");
export const pxr::TfToken xformOpOrient("xformOp:orient");
export const pxr::TfToken xformOpScale("xformOp:scale");

// Compute the relative transform from \p ancestor to \p prim by composing
// local transforms along the prim hierarchy. The returned matrix maps points
// from prim-local coordinates into ancestor-local coordinates and is expressed
// in stage units. If \p outResetsXformStack is non-null, it will be set to
// true if any intermediate prim resets the xform stack.
export pxr::GfMatrix4d
ComputeRelativeTransform(const pxr::UsdPrim &prim, const pxr::UsdPrim &ancestor,
                         bool *outResetsXformStack = nullptr) {
	pxr::GfMatrix4d xform(1.0);
	bool anyReset = false;

	for (pxr::UsdPrim p = prim; p && p != ancestor; p = p.GetParent()) {
		pxr::UsdGeomXformable xf(p);
		if (!xf) {
			continue;
		}
		pxr::GfMatrix4d local(1.0);
		bool localReset = false;
		if (!xf.GetLocalTransformation(&local, &localReset)) {
			continue;
		}
		xform *= local;
		anyReset = anyReset || localReset;
		if (localReset) {
			break;
		}
	}

	if (outResetsXformStack) {
		*outResetsXformStack = anyReset;
	}
	return xform;
}

} // namespace lego_assemble
