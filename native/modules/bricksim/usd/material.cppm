export module bricksim.usd.material;

import std;
import bricksim.core.specs;
import bricksim.utils.sdf;
import bricksim.vendor;

namespace bricksim {

float srgb_to_linear(std::uint8_t value) {
	float c = static_cast<float>(value) / 255.0f;
	if (c <= 0.04045f) {
		return c / 12.92f;
	}
	return std::pow((c + 0.055f) / 1.055f, 2.4f);
}

const pxr::TfToken PreviewSurfaceShaderName{"PreviewSurface"};
const pxr::TfToken inputDiffuseColor{"inputs:diffuseColor"};
const pxr::TfToken inputRoughness{"inputs:roughness"};
const pxr::TfToken inputIor{"inputs:ior"};

export void create_lego_material(const pxr::SdfLayerHandle &layer,
                                 const pxr::SdfPath &path,
                                 const BrickColor &color) {
	pxr::SdfChangeBlock _changes;

	pxr::GfVec3f diffuse_color{
	    srgb_to_linear(color[0]),
	    srgb_to_linear(color[1]),
	    srgb_to_linear(color[2]),
	};

	auto material = pxr::SdfCreatePrimInLayer(layer, path);
	material->SetSpecifier(pxr::SdfSpecifierDef);
	material->SetTypeName(pxr::UsdShadeTokens->Material);

	auto shader = pxr::SdfCreatePrimInLayer(
	    layer, path.AppendChild(PreviewSurfaceShaderName));
	shader->SetSpecifier(pxr::SdfSpecifierDef);
	shader->SetTypeName(pxr::UsdShadeTokens->Shader);

	SetAttr<pxr::TfToken>(shader, pxr::UsdShadeTokens->infoId,
	                      pxr::UsdImagingTokens->UsdPreviewSurface);
	SetAttr<pxr::GfVec3f>(shader, inputDiffuseColor, diffuse_color,
	                      pxr::SdfValueRoleNames->Color);
	SetAttr<float>(shader, inputRoughness, 0.15f);
	SetAttr<float>(shader, inputIor, 1.53f);

	auto shader_surface =
	    SetAttr<pxr::TfToken>(shader, pxr::UsdShadeTokens->outputsSurface, {});
	auto material_surface = SetAttr<pxr::TfToken>(
	    material, pxr::UsdShadeTokens->outputsSurface, {});
	auto connections = material_surface->GetConnectionPathList();
	connections.ClearEditsAndMakeExplicit();
	connections.Add(shader_surface->GetPath());
}

} // namespace bricksim
