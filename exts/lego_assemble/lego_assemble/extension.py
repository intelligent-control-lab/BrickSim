import omni.ext #type: ignore

class LegoExtension(omni.ext.IExt):
    def on_startup(self, ext_id):
        self._init_ui()

    def on_shutdown(self):
        if self._ui is not None:
            self._ui.destroy()

    def _init_ui(self):
        try:
            import omni.kit.window.property as p
        except ImportError:
            # Likely running headless
            self._ui = None
            return
        from lego_assemble.ui.main_ui import LegoUI
        self._ui = LegoUI()
