"""Viewport HUD integration for BrickSim frame-time profiling."""

from dataclasses import dataclass

import carb.settings

from bricksim.core import get_last_step_profiling

SETTING_DISPLAY_LAST_STEP = "/persistent/bricksim/hud/displayLastStepProfiling"

_ViewportUpdateInfo = dict[str, object]


@dataclass(frozen=True)
class _ViewportFpsPatch:
    skip_update: object
    update_stats: object


_VIEWPORT_FPS_PATCH: _ViewportFpsPatch | None = None


def _display_last_step_enabled() -> bool:
    return bool(carb.settings.get_settings().get(SETTING_DISPLAY_LAST_STEP))


def _install_viewport_fps_patch() -> None:
    global _VIEWPORT_FPS_PATCH

    if _VIEWPORT_FPS_PATCH is not None:
        return

    from omni.kit.viewport.window.stats import ViewportFPS

    original_skip_update = ViewportFPS.skip_update
    original_update_stats = ViewportFPS.update_stats
    _VIEWPORT_FPS_PATCH = _ViewportFpsPatch(
        skip_update=original_skip_update,
        update_stats=original_update_stats,
    )

    def patched_skip_update(
        self: ViewportFPS,
        update_info: _ViewportUpdateInfo,
    ) -> bool:
        should_skip = original_skip_update(self, update_info)
        show_last_step = _display_last_step_enabled()
        toggle_changed = (
            getattr(self, "_bricksim_show_last_step_enabled", None) != show_last_step
        )
        setattr(self, "_bricksim_show_last_step_enabled", show_last_step)
        if show_last_step or toggle_changed:
            return False
        return bool(should_skip)

    def patched_update_stats(
        self: ViewportFPS,
        update_info: _ViewportUpdateInfo,
    ) -> list[str]:
        stats = list(original_update_stats(self, update_info))
        if not _display_last_step_enabled():
            return stats
        try:
            profiling = get_last_step_profiling()
        except Exception:
            stats.append("BrickSim Last Frame Time: n/a")
        else:
            stats.append(
                f"BrickSim Last Frame Time: {profiling.step_time * 1000.0:.2f} ms"
            )
        return stats

    setattr(ViewportFPS, "skip_update", patched_skip_update)
    setattr(ViewportFPS, "update_stats", patched_update_stats)


def _restore_viewport_fps_patch() -> None:
    global _VIEWPORT_FPS_PATCH

    patch = _VIEWPORT_FPS_PATCH
    if patch is None:
        return

    from omni.kit.viewport.window.stats import ViewportFPS

    setattr(ViewportFPS, "skip_update", patch.skip_update)
    setattr(ViewportFPS, "update_stats", patch.update_stats)
    _VIEWPORT_FPS_PATCH = None


class FrameTimeHudController:
    """Register the BrickSim frame-time HUD menu item and viewport patch."""

    def __init__(self) -> None:
        """Install the viewport FPS patch and menu item."""
        carb.settings.get_settings().set_default(SETTING_DISPLAY_LAST_STEP, False)
        _install_viewport_fps_patch()

        from omni.kit.viewport.menubar.core import CategoryStateItem
        from omni.kit.viewport.menubar.display import (
            ViewportDisplayMenuBarExtension,
            get_instance,
        )

        self._menubar_display_inst: ViewportDisplayMenuBarExtension | None = (
            get_instance()
        )
        self._custom_item: CategoryStateItem | None = CategoryStateItem(
            "BrickSim Frame Time",
            setting_path=SETTING_DISPLAY_LAST_STEP,
        )
        if self._menubar_display_inst is not None:
            self._menubar_display_inst.register_custom_category_item(
                "Heads Up Display",
                self._custom_item,
            )

    def destroy(self) -> None:
        """Unregister the menu item and restore the viewport FPS patch."""
        if self._menubar_display_inst is not None and self._custom_item is not None:
            self._menubar_display_inst.deregister_custom_category_item(
                "Heads Up Display",
                self._custom_item,
            )
        self._menubar_display_inst = None
        self._custom_item = None
        _restore_viewport_fps_patch()
