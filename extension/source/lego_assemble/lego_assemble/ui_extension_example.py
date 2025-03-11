import omni.ext
import omni.ui
import omni.usd
import omni.physx
from pxr import Usd, UsdGeom
from .brick_generator import create_brick
from .brick_physics import contact_report_event_handler

class LegoExtension(omni.ext.IExt):

    def on_startup(self, ext_id):
        print("[lego_assemble] startup")

        self.contact_report_event_sub = omni.physx.get_physx_simulation_interface().subscribe_contact_report_events(contact_report_event_handler)

        self._window = omni.ui.Window("My Window", width=300, height=300)
        with self._window.frame:
            omni.ui.Button("Click me", clicked_fn=self.on_button_clicked)

    def on_shutdown(self):
        print("[lego_assemble] shutdown")
        self.contact_report_event_sub.unsubscribe()

    def on_button_clicked(self):
        stage: Usd.Stage = omni.usd.get_context().get_stage()
        if stage is None:
            return
        stage.RemovePrim("/World/Brick_01")
        brick01 = create_brick(stage, "/World/Brick_01", dimensions=(4, 2, 3), color_name="Pink")
        UsdGeom.XformCommonAPI(brick01).SetTranslate((0, 0, 0.100))
        stage.RemovePrim("/World/Brick_02")
        brick02 = create_brick(stage, "/World/Brick_02", dimensions=(4, 2, 3), color_name="Light Blue")
        UsdGeom.XformCommonAPI(brick02).SetTranslate((0, 0, 0.050))
