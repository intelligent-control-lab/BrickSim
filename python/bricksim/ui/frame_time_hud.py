import carb.settings

from bricksim.core import get_last_step_profiling


SETTING_DISPLAY_LAST_STEP = "/persistent/bricksim/hud/displayLastStepProfiling"

_ORIGINAL_VIEWPORT_FPS_SKIP_UPDATE = None
_ORIGINAL_VIEWPORT_FPS_UPDATE_STATS = None


def _display_last_step_enabled() -> bool:
    return bool(carb.settings.get_settings().get(SETTING_DISPLAY_LAST_STEP))


def _install_viewport_fps_patch() -> None:
    global _ORIGINAL_VIEWPORT_FPS_SKIP_UPDATE
    global _ORIGINAL_VIEWPORT_FPS_UPDATE_STATS

    if _ORIGINAL_VIEWPORT_FPS_SKIP_UPDATE is not None:
        return

    import omni.kit.viewport.window.stats as viewport_stats

    _ORIGINAL_VIEWPORT_FPS_SKIP_UPDATE = viewport_stats.ViewportFPS.skip_update
    _ORIGINAL_VIEWPORT_FPS_UPDATE_STATS = viewport_stats.ViewportFPS.update_stats

    def patched_skip_update(self, update_info):
        should_skip = _ORIGINAL_VIEWPORT_FPS_SKIP_UPDATE(self, update_info)
        show_last_step = _display_last_step_enabled()
        toggle_changed = (
            getattr(self, "_bricksim_show_last_step_enabled", None)
            != show_last_step
        )
        self._bricksim_show_last_step_enabled = show_last_step
        if show_last_step or toggle_changed:
            return False
        return should_skip

    def patched_update_stats(self, update_info):
        stats = list(_ORIGINAL_VIEWPORT_FPS_UPDATE_STATS(self, update_info))
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

    viewport_stats.ViewportFPS.skip_update = patched_skip_update
    viewport_stats.ViewportFPS.update_stats = patched_update_stats


def _restore_viewport_fps_patch() -> None:
    global _ORIGINAL_VIEWPORT_FPS_SKIP_UPDATE
    global _ORIGINAL_VIEWPORT_FPS_UPDATE_STATS

    if _ORIGINAL_VIEWPORT_FPS_SKIP_UPDATE is None:
        return

    import omni.kit.viewport.window.stats as viewport_stats

    viewport_stats.ViewportFPS.skip_update = _ORIGINAL_VIEWPORT_FPS_SKIP_UPDATE
    viewport_stats.ViewportFPS.update_stats = _ORIGINAL_VIEWPORT_FPS_UPDATE_STATS
    _ORIGINAL_VIEWPORT_FPS_SKIP_UPDATE = None
    _ORIGINAL_VIEWPORT_FPS_UPDATE_STATS = None


class FrameTimeHudController:
    def __init__(self):
        carb.settings.get_settings().set_default(SETTING_DISPLAY_LAST_STEP, False)
        _install_viewport_fps_patch()

        from omni.kit.viewport.menubar.core import CategoryStateItem
        from omni.kit.viewport.menubar.display import get_instance

        self._menubar_display_inst = get_instance()
        self._custom_item = CategoryStateItem(
            "BrickSim Frame Time",
            setting_path=SETTING_DISPLAY_LAST_STEP,
        )
        if self._menubar_display_inst is not None:
            self._menubar_display_inst.register_custom_category_item(
                "Heads Up Display",
                self._custom_item,
            )

    def destroy(self):
        if self._menubar_display_inst is not None:
            self._menubar_display_inst.deregister_custom_category_item(
                "Heads Up Display",
                self._custom_item,
            )
        self._menubar_display_inst = None
        self._custom_item = None
        _restore_viewport_fps_patch()
