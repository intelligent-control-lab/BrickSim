import carb.settings
import omni.ext
from lego_assemble.physics.interface import init_brick_physics_interface, deinit_brick_physics_interface

class LegoExtension(omni.ext.IExt):
    def on_startup(self, ext_id):
        # Force enabling contact processing, see https://github.com/isaac-sim/IsaacLab/pull/1861
        self._enable_contact_processing()
        self._disableContactProcessing_sub = carb.settings.get_settings().subscribe_to_node_change_events(
            "/physics/disableContactProcessing", self._enable_contact_processing
        )

        self.brick_physics = init_brick_physics_interface()
        self._init_ui()

    def on_shutdown(self):
        deinit_brick_physics_interface()
        carb.settings.get_settings().unsubscribe_to_change_events(self._disableContactProcessing_sub)

    def _init_ui(self):
        try:
            import omni.kit.window.property as p
        except ImportError:
            # Likely running headless
            return

        from lego_assemble.ui import LegoUI
        self._ui = LegoUI()

    def _enable_contact_processing(self, *args, **kwargs):
        carb_settings = carb.settings.get_settings()
        if carb_settings.get("/physics/disableContactProcessing") is not False:
            carb_settings.set_bool("/physics/disableContactProcessing", False)
