"""Toolbar patch that adds a one-frame step button."""

import asyncio
from pathlib import Path


async def _play_then_pause_immediately():
    """Advance the timeline by one app update.

    Issue PLAY, then pause on the next app update.
    """
    import omni.kit.app
    import omni.timeline

    timeline = omni.timeline.get_timeline_interface()
    timeline.play()
    await omni.kit.app.get_app().next_update_async()
    timeline.pause()


def _patch_play_button_group():
    """Patch PlayButtonGroup to add a dedicated "Step one frame" button.

    Place it beside Play/Stop on the left toolbar.
    """
    import omni.timeline
    import omni.ui as ui
    from omni.kit.widget.toolbar.builtin_tools.play_button_group import (
        PlayButtonGroup,
    )

    if getattr(PlayButtonGroup, "_lego_step_one_frame_patched", False):
        return

    orig_clean = PlayButtonGroup.clean
    orig_get_style = PlayButtonGroup.get_style
    orig_create = PlayButtonGroup.create
    orig_on_timeline_event = PlayButtonGroup._on_timeline_event

    def _lego_clean(self):
        orig_clean(self)
        self._lego_step_one_frame_button = None

    PlayButtonGroup.clean = _lego_clean

    def _lego_get_style(self):
        style = orig_get_style(self)
        style["Button.Image::step_one_frame"] = {
            "image_url": str(Path(__file__).parent / "step_forward.svg")
        }
        return style

    PlayButtonGroup.get_style = _lego_get_style

    def _lego_create(self, default_size):
        widgets = orig_create(self, default_size)
        if widgets is None:
            widgets = {}

        def on_step_one_frame_clicked(*_):
            asyncio.ensure_future(_play_then_pause_immediately())

        step_button = ui.Button(
            name="step_one_frame",
            tooltip="Step one frame",
            width=default_size,
            height=default_size,
            clicked_fn=on_step_one_frame_clicked,
        )
        step_button.visible = self._visible
        self._lego_step_one_frame_button = step_button
        widgets["step_one_frame"] = step_button
        return widgets

    PlayButtonGroup.create = _lego_create

    def _lego_on_timeline_event(self, e):
        orig_on_timeline_event(self, e)
        if hasattr(
            omni.timeline.TimelineEventType, "DIRECTOR_CHANGED"
        ) and e.type == int(omni.timeline.TimelineEventType.DIRECTOR_CHANGED):
            step_button = getattr(self, "_lego_step_one_frame_button", None)
            if step_button is not None:
                step_button.visible = self._visible

    PlayButtonGroup._on_timeline_event = _lego_on_timeline_event
    PlayButtonGroup._lego_step_one_frame_patched = True


def _rebuild_play_button_group():
    """Rebuild the PlayButtonGroup instance.

    Make it pick up our monkey patch that adds the Step one frame button.
    """
    import omni.kit.widget.toolbar.extension as tb_ext
    from omni.kit.widget.toolbar.builtin_tools.play_button_group import (
        PlayButtonGroup,
    )

    toolbar = tb_ext.get_instance()
    if toolbar is None:
        return

    builtin = getattr(toolbar, "_builtin_tools", None)
    if builtin is None:
        return

    old_group = getattr(builtin, "_play_button_group", None)
    if old_group is not None:
        toolbar.remove_widget(old_group)
        builtin._play_button_group = None

    new_group = PlayButtonGroup()
    builtin._play_button_group = new_group
    toolbar.add_widget(new_group, 21)


def install_step_one_frame_patch():
    """Entry point invoked from extension startup.

    Patch the Isaac toolbar with an extra Step one frame control.
    """
    _patch_play_button_group()
    _rebuild_play_button_group()
