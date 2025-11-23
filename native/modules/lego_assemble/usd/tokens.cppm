module;

#include <pxr/base/tf/staticTokens.h>

export module lego_assemble.usd.tokens;

namespace lego_assemble {

using pxr::TfStaticData;
using pxr::TfToken;

// clang-format off
#define LEGO_TOKENS                                                            \
    ((Connection, "LegoConnection"))                                           \
    ((ConnStud, "lego:conn_stud"))                                             \
    ((ConnHole, "lego:conn_hole"))                                             \
    ((ConnStudInterface, "lego:conn_stud_interface"))                          \
    ((ConnHoleInterface, "lego:conn_hole_interface"))                          \
    ((ConnOffset, "lego:conn_offset"))                                         \
    ((ConnYaw, "lego:conn_yaw"))                                               \
    ((PartKind, "lego:part_kind"))                                             \
    ((PartKindBrick, "brick"))                                                 \
    ((BrickDimensions, "lego:brick_dimensions"))                               \
    ((BrickColor, "lego:brick_color"))                                         \
    ((BodyCollider, "BodyCollider"))                                           \
    ((TopCollider, "TopCollider"))                                             \
    ((Body, "Body"))                                                           \
    ((Studs, "Studs"))                                                         \
    ((StudPrototype, "StudPrototype"))                                         \
// clang-format on

export {
	TF_DECLARE_PUBLIC_TOKENS(LegoTokens, LEGO_TOKENS);
}

TF_DEFINE_PUBLIC_TOKENS(LegoTokens, LEGO_TOKENS);

} // namespace lego_assemble
