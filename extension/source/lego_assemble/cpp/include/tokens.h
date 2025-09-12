#pragma once

#include <pxr/base/tf/staticTokens.h>

namespace lego_assemble {

using pxr::TfStaticData;
using pxr::TfToken;

#define LEGO_TOKENS (lego_dimensions)(lego_color)(lego_conn)

TF_DECLARE_PUBLIC_TOKENS(LegoTokens, LEGO_TOKENS);

} // namespace lego_assemble
