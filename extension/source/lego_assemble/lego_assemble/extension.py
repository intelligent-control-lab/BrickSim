import carb
import carb.settings
import omni.ext
import omni.physx
from lego_assemble import _native
from lego_assemble.physics.interface import init_brick_physics_interface, deinit_brick_physics_interface
from lego_assemble.physics.brick_joint import on_simulation_event

class LegoExtension(omni.ext.IExt):
    def on_startup(self, ext_id):
        if not _native.init_natives():
            carb.log_error("Failed to initialize native components")

        # Forcibly enabling contact processing, see https://github.com/isaac-sim/IsaacLab/pull/1861
        self._enable_contact_processing()
        self._disableContactProcessing_sub = carb.settings.get_settings().subscribe_to_node_change_events(
            "/physics/disableContactProcessing", self._enable_contact_processing
        )

        self.brick_physics = init_brick_physics_interface()

        self.simulation_event_sub = omni.physx.get_physx_interface().get_simulation_event_stream_v2().create_subscription_to_pop(on_simulation_event)

        self._init_ui()

    def on_shutdown(self):
        if self._ui is not None:
            self._ui.destroy()

        self.simulation_event_sub.unsubscribe()

        deinit_brick_physics_interface()

        carb.settings.get_settings().unsubscribe_to_change_events(self._disableContactProcessing_sub)

        if not _native.deinit_natives():
            carb.log_error("Failed to deinitialize native components")

    def _init_ui(self):
        try:
            import omni.kit.window.property as p
        except ImportError:
            # Likely running headless
            self._ui = None
            return
        from lego_assemble.ui import LegoUI
        self._ui = LegoUI()

    def _enable_contact_processing(self, *args, **kwargs):
        carb_settings = carb.settings.get_settings()
        if carb_settings.get("/physics/disableContactProcessing") is not False:
            carb_settings.set_bool("/physics/disableContactProcessing", False)
