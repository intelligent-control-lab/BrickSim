import traceback
import omni.ext #type: ignore

class LegoExtension(omni.ext.IExt):
    def on_startup(self, ext_id):
        self._init_ui()

    def on_shutdown(self):
        if getattr(self, "_assembly_selection", None) is not None:
            self._assembly_selection.destroy()
            self._assembly_selection = None
        if getattr(self, "_ui", None) is not None:
            self._ui.destroy()
        if getattr(self, "_stabletext2brick_browser", None) is not None:
            self._stabletext2brick_browser.destroy()
            self._stabletext2brick_browser = None

    def _init_ui(self):
        try:
            import omni.kit.window.property as p
        except ImportError:
            # Likely running headless
            self._ui = None
            return

        # Patch Isaac Sim's toolbar at runtime to add a "Connected Component"
        # selection mode that shares picking semantics with kind:component.
        from lego_assemble.ui.toolbar_patch import install_toolbar_patches
        from lego_assemble.ui.selection_sync import AssemblySelectionSync
        install_toolbar_patches()
        self._assembly_selection = AssemblySelectionSync()

        from lego_assemble.ui.main_ui import LegoUI
        self._ui = LegoUI()

        # StableText2Brick dataset browser (optional HF-backed UI).
        try:
            from lego_assemble.ui.stabletext2brick_browser import StableText2BrickBrowser
            # Share env_id with the main UI.
            self._stabletext2brick_browser = StableText2BrickBrowser(self._ui.get_env_id)
        except Exception:
            traceback.print_exc()
            # Fail fast for the core UI; dataset browser is best-effort.
            self._stabletext2brick_browser = None
