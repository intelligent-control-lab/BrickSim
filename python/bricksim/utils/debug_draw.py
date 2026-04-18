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
    from isaacsim.util.debug_draw import _debug_draw  # ty: ignore[unresolved-import]

    return _debug_draw.acquire_debug_draw_interface()
