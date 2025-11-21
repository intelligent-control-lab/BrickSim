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
