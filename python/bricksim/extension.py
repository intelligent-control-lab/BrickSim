import omni.ext

class BrickSimExtension(omni.ext.IExt):
    def on_startup(self, ext_id):
        self._init_ui()

    def on_shutdown(self):
        if getattr(self, "_frame_time_hud", None) is not None:
            self._frame_time_hud.destroy()
            self._frame_time_hud = None
        if getattr(self, "_connection_overlay", None) is not None:
            self._connection_overlay.destroy()
            self._connection_overlay = None
        if getattr(self, "_assembly_selection", None) is not None:
            self._assembly_selection.destroy()
            self._assembly_selection = None
        if getattr(self, "_ui", None) is not None:
            self._ui.destroy()
            self._ui = None
        if getattr(self, "_structures_browser", None) is not None:
            self._structures_browser.destroy()
            self._structures_browser = None

    def _init_ui(self):
        try:
            import omni.kit.window.property as p
        except ImportError:
            # Likely running headless
            self._ui = None
            return

        # Patch Isaac Sim's toolbar at runtime to add a "Connected Component"
        # selection mode that shares picking semantics with kind:component.
        from bricksim.ui.toolbar_patch import install_toolbar_patches
        from bricksim.ui.step_one_frame_patch import install_step_one_frame_patch
        from bricksim.ui.selection_sync import AssemblySelectionSync
        install_toolbar_patches()
        install_step_one_frame_patch()
        self._assembly_selection = AssemblySelectionSync()

        from bricksim.ui.connection_overlay import ConnectionOverlayController
        self._connection_overlay = ConnectionOverlayController()

        from bricksim.ui.frame_time_hud import FrameTimeHudController
        self._frame_time_hud = FrameTimeHudController()

        from bricksim.ui.main_ui import LegoUI
        self._ui = LegoUI()

        # Lego Structures dataset browser.
        from bricksim.ui.structures_browsers import LegoStructuresBrowser
        self._structures_browser = LegoStructuresBrowser(self._ui)
