import omni.ext
from lego_assemble.physics.interface import init_brick_physics_interface, deinit_brick_physics_interface

class LegoExtension(omni.ext.IExt):
    def on_startup(self, ext_id):
        self.brick_physics = init_brick_physics_interface()
        self._init_ui()

    def on_shutdown(self):
        deinit_brick_physics_interface()

    def _init_ui(self):
        try:
            import omni.kit.window.property as p
        except ImportError:
            # Likely running headless
            return

        from lego_assemble.ui import LegoUI
        self._ui = LegoUI()
