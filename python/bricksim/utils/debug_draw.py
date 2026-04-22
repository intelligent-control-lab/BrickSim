"""Debug drawing helpers."""

from collections.abc import Sequence
from typing import Protocol

Point3 = tuple[float, float, float]
Color4 = tuple[float, float, float, float]


class DebugDraw(Protocol):
    """Subset of the Isaac Sim debug-draw API used by BrickSim."""

    def clear_points(self) -> None:
        """Clear previously drawn debug points."""

    def draw_points(
        self,
        points: Sequence[Point3],
        colors: Sequence[Color4],
        sizes: Sequence[float],
    ) -> None:
        """Draw debug points with RGBA colors and pixel sizes."""


def acquire_debug_draw() -> DebugDraw:
    """Acquire Isaac Sim's debug-draw interface.

    Returns:
        Isaac Sim debug-draw interface.
    """
    import omni.kit.app

    ext_manager = omni.kit.app.get_app().get_extension_manager()
    if not ext_manager.get_enabled_extension_id("isaacsim.util.debug_draw"):
        if not ext_manager.set_extension_enabled_immediate(
            "isaacsim.util.debug_draw",
            True,
        ):
            raise RuntimeError("Failed to enable isaacsim.util.debug_draw")
    from isaacsim.util.debug_draw import _debug_draw  # ty: ignore[unresolved-import]

    return _debug_draw.acquire_debug_draw_interface()
