import logging
import omni.ext
import omni.ui
import omni.usd
import omni.physx.scripts.physicsUtils as physicsUtils
from pxr import Usd
from . import lego_schemes
from .brick_generator import create_brick
from .brick_physics import LegoPhysicsCallback

logger = logging.getLogger(__name__)

class LegoExtension(omni.ext.IExt):

    def on_startup(self, ext_id):
        self.lego_physics_callback = LegoPhysicsCallback()

        self._window = omni.ui.Window("LEGO Assemble", width=300, height=300)
        with self._window.frame:
            with omni.ui.VStack(height=0, spacing=5):
                with omni.ui.HStack(spacing=10):
                    omni.ui.Label("Length:", width=100)
                    self._dim_x_field = omni.ui.IntDrag(min=1, max=50)
                    self._dim_x_field.model.set_value(4)
                with omni.ui.HStack(spacing=10):
                    omni.ui.Label("Width:", width=100)
                    self._dim_y_field = omni.ui.IntDrag(min=1, max=50)
                    self._dim_y_field.model.set_value(2)
                with omni.ui.HStack(spacing=10):
                    omni.ui.Label("Height:", width=100)
                    self._dim_z_field = omni.ui.IntDrag(min=1, max=50)
                    self._dim_z_field.model.set_value(3)
                with omni.ui.HStack(spacing=10):
                    omni.ui.Label("Color:", width=100)
                    self._color_options = list(lego_schemes.Colors.keys())
                    self._color_combo = omni.ui.ComboBox(self._color_options.index("Pink"), *self._color_options)
                with omni.ui.HStack(spacing=10):
                    omni.ui.Label("Base Path:", width=100)
                    self._base_path_field = omni.ui.StringField()
                    self._base_path_field.model.set_value("/World/Brick_")
                with omni.ui.HStack(spacing=10):
                    omni.ui.Label("Position:", width=100)
                    self._pos_x_field = omni.ui.FloatField()
                    self._pos_x_field.model.set_value(0)
                    self._pos_y_field = omni.ui.FloatField()
                    self._pos_y_field.model.set_value(0)
                    self._pos_z_field = omni.ui.FloatField()
                    self._pos_z_field.model.set_value(0.1)
                with omni.ui.HStack(spacing=10):
                    omni.ui.Label("Use Cache:", width=100)
                    self._use_cache_checkbox = omni.ui.CheckBox()
                    self._use_cache_checkbox.model.set_value(True)
                omni.ui.Button("Add Brick", clicked_fn=self.on_add_brick)

    def on_shutdown(self):
        self.lego_physics_callback.unsubscribe()

    def on_add_brick(self):
        width = self._dim_x_field.model.as_int
        length = self._dim_y_field.model.as_int
        height = self._dim_z_field.model.as_int
        color = self._color_options[self._color_combo.model.get_item_value_model().as_int]
        use_cache = self._use_cache_checkbox.model.as_bool
        pos_x = self._pos_x_field.model.as_float
        pos_y = self._pos_y_field.model.as_float
        pos_z = self._pos_z_field.model.as_float

        stage: Usd.Stage = omni.usd.get_context().get_stage()
        if stage is None:
            return
        uniquifier = 1
        base_path = self._base_path_field.model.as_string
        while stage.GetPrimAtPath(f"{base_path}{uniquifier}").IsValid():
            uniquifier += 1
        path = f"{base_path}{uniquifier}"
        brick = create_brick(stage, path, dimensions=(width, length, height), color_name=color, use_cache=use_cache)
        physicsUtils.set_or_add_translate_op(brick, (pos_x, pos_y, pos_z))
        logger.info(f"Added brick {path} ({width}x{length}x{height}) {color}")
