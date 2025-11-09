import carb
import omni.ext #type: ignore
from lego_assemble import _native

class LegoExtension(omni.ext.IExt):
    def on_startup(self, ext_id):
        if not _native.init_natives():
            carb.log_error("Failed to initialize native components")
            return
        self._init_ui()

    def on_shutdown(self):
        if self._ui is not None:
            self._ui.destroy()
        if not _native.deinit_natives():
            carb.log_error("Failed to deinitialize native components")

    def _init_ui(self):
        try:
            import omni.kit.window.property as p
        except ImportError:
            # Likely running headless
            self._ui = None
            return
        from lego_assemble.ui.main_ui import LegoUI
        self._ui = LegoUI()
