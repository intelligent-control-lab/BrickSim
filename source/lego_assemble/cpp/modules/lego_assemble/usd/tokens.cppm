module;

#include <pxr/base/tf/staticTokens.h>

export module lego_assemble.usd.tokens;

namespace lego_assemble {

using pxr::TfStaticData;
using pxr::TfToken;

// clang-format off
#define LEGO_TOKENS                                                            \
    ((conn_body0, "lego_conn:body0"))                                          \
    ((conn_body1, "lego_conn:body1"))                                          \
    ((conn_offset_studs, "lego_conn:offset_studs"))                            \
    ((conn_yaw_index, "lego_conn:yaw_index"))                                  \
    ((conn_pos0, "lego_conn:pos0"))                                            \
    ((conn_rot0, "lego_conn:rot0"))                                            \
    ((conn_pos1, "lego_conn:pos1"))                                            \
    ((conn_rot1, "lego_conn:rot1"))                                            \
    ((conn_overlap_xy, "lego_conn:overlap_xy"))                                \
    ((conn_enabled, "lego_conn:enabled"))                                      \
    ((brick_dimensions, "lego_brick:dimensions"))                              \
    ((brick_color, "lego_brick:color"))                                        \
    ((BodyCollider, "BodyCollider"))                                           \
    ((TopCollider, "TopCollider"))                                             \
    ((Body, "Body"))                                                           \
    ((Studs, "Studs"))                                                         \
    ((StudPrototype, "StudPrototype"))
// clang-format on

export {
	TF_DECLARE_PUBLIC_TOKENS(LegoTokens, LEGO_TOKENS);
}

TF_DEFINE_PUBLIC_TOKENS(LegoTokens, LEGO_TOKENS);

} // namespace lego_assemble
