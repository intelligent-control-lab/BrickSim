"""Typing helpers for third-party APIs with incomplete stubs."""

from typing import Protocol, runtime_checkable

import omni.usd
from pxr import Usd


@runtime_checkable
class _UsdContextWithStage(Protocol):
    """USD context surface missing from the Omni Python stub."""

    def get_stage(self) -> Usd.Stage | None:
        """Return the current USD stage.

        Returns:
            Current USD stage, or ``None`` when no stage is open.
        """
        ...


def get_usd_context_stage(usd_context: omni.usd.UsdContext) -> Usd.Stage | None:
    """Return the current USD stage from an Omni USD context.

    Args:
        usd_context: USD context returned by ``omni.usd.get_context()``.

    Returns:
        Current USD stage, or ``None`` when no stage is open.
    """
    assert isinstance(usd_context, _UsdContextWithStage)
    return usd_context.get_stage()
