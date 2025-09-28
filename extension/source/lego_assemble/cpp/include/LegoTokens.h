#pragma once

#include <pxr/base/tf/staticTokens.h>

namespace lego_assemble {

using pxr::TfStaticData;
using pxr::TfToken;

#define LEGO_TOKENS                                                            \
	((conn_body0, "lego_conn:body0"))((conn_body1, "lego_conn:body1"))(        \
	    (conn_pos, "lego_conn:pos"))((conn_rot, "lego_conn:rot"))(             \
	    (conn_enabled, "lego_conn:enabled"))(                                  \
	    (brick_dimensions, "lego_brick:dimensions"))(                          \
	    (brick_color, "lego_brick:color"))

TF_DECLARE_PUBLIC_TOKENS(LegoTokens, LEGO_TOKENS);

} // namespace lego_assemble
