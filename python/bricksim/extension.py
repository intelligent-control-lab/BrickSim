"""Omniverse extension entry point for BrickSim."""

from typing import Protocol

import omni.ext


class _SupportsDestroy(Protocol):
    """Object with an explicit teardown hook."""

    def destroy(self) -> None:
        """Release resources held by the object."""
        ...


class BrickSimExtension(omni.ext.IExt):
    """Register and tear down BrickSim UI components inside Omniverse."""

    _frame_time_hud: _SupportsDestroy | None = None
    _connection_overlay: _SupportsDestroy | None = None
    _assembly_selection: _SupportsDestroy | None = None
    _ui: _SupportsDestroy | None = None
    _structures_browser: _SupportsDestroy | None = None

    def on_startup(self, ext_id):
        """Initialize BrickSim extension UI at startup."""
        self._init_ui()

    def on_shutdown(self):
        """Destroy BrickSim extension UI components at shutdown."""
        if self._frame_time_hud is not None:
            self._frame_time_hud.destroy()
            self._frame_time_hud = None
        if self._connection_overlay is not None:
            self._connection_overlay.destroy()
            self._connection_overlay = None
        if self._assembly_selection is not None:
            self._assembly_selection.destroy()
            self._assembly_selection = None
        if self._ui is not None:
            self._ui.destroy()
            self._ui = None
        if self._structures_browser is not None:
            self._structures_browser.destroy()
            self._structures_browser = None

    def _init_ui(self):
        try:
            import omni.kit.window.property  # noqa: F401
        except ImportError:
            # Likely running headless
            self._ui = None
            return

        # Patch Isaac Sim's toolbar at runtime to add a "Connected Component"
        # selection mode that shares picking semantics with kind:component.
        from bricksim.ui.selection_sync import AssemblySelectionSync
        from bricksim.ui.step_one_frame_patch import install_step_one_frame_patch
        from bricksim.ui.toolbar_patch import install_toolbar_patches

        install_toolbar_patches()
        install_step_one_frame_patch()
        self._assembly_selection = AssemblySelectionSync()

        from bricksim.ui.connection_overlay import ConnectionOverlayController

        self._connection_overlay = ConnectionOverlayController()

        from bricksim.ui.frame_time_hud import FrameTimeHudController

        self._frame_time_hud = FrameTimeHudController()

        from bricksim.ui.main_ui import LegoUI

        ui = LegoUI()
        self._ui = ui

        # Lego Structures dataset browser.
        from bricksim.ui.structures_browsers import LegoStructuresBrowser

        self._structures_browser = LegoStructuresBrowser(ui)
