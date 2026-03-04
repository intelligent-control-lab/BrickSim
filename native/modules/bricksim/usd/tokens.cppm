module;

#include <pxr/base/tf/staticTokens.h>

export module bricksim.usd.tokens;

namespace bricksim {

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
    ((Tubes, "Tubes"))                                                         \
    ((TubePrototype, "TubePrototype"))                                         \
    ((Pillars, "Pillars"))                                                     \
    ((PillarPrototype, "PillarPrototype"))                                     \
    ((LegoWorkspaceObstacles, "lego:workspace_obstacles"))                     \
    ((LegoWorkspaceClearance, "lego:workspace_clearance"))                     \
    ((LegoWorkspaceGridResolution, "lego:workspace_grid_resolution"))          \
    ((LegoWorkspaceAllowRotation, "lego:workspace_allow_rotation"))            \
// clang-format on

export {
	TF_DECLARE_PUBLIC_TOKENS(LegoTokens, LEGO_TOKENS);
}

TF_DEFINE_PUBLIC_TOKENS(LegoTokens, LEGO_TOKENS);

} // namespace bricksim
