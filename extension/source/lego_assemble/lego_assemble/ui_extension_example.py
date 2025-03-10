import omni.ext
import omni.ui

class ExampleExtension(omni.ext.IExt):

    def on_startup(self, ext_id):
        print("[lego_assemble] startup")

        self._window = omni.ui.Window("My Window", width=300, height=300)
        with self._window.frame:
            omni.ui.Button("Click me", clicked_fn=lambda: print("[lego_assemble] Hello world!"))

    def on_shutdown(self):
        print("[lego_assemble] shutdown")
