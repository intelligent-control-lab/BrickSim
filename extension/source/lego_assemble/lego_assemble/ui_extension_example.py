import omni.ext
import omni.ui
import omni.usd
from pxr import Usd, UsdGeom
from .brick_generator import create_brick

class ExampleExtension(omni.ext.IExt):

    def on_clicked(self):
        stage: Usd.Stage = omni.usd.get_context().get_stage()
        if stage is not None:
            stage.RemovePrim("/World/Brick")
            brick = create_brick(stage, "/World/Brick")
            UsdGeom.XformCommonAPI(brick).SetTranslate((0, 0, 0.100))

    def on_startup(self, ext_id):
        print("[lego_assemble] startup")

        self._window = omni.ui.Window("My Window", width=300, height=300)
        with self._window.frame:
            omni.ui.Button("Click me", clicked_fn=self.on_clicked)

    def on_shutdown(self):
        print("[lego_assemble] shutdown")
