import omni.ui
import lego_assemble.physics.lego_schemes as lego_schemes
from lego_assemble.physics.interface import get_brick_physics_interface

class LegoUI():
    def __init__(self):
        self._window = omni.ui.Window("LEGO Assemble", width=300, height=300)
        self._window.deferred_dock_in("Console")
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
                    omni.ui.Label("Env id:", width=100)
                    self._base_path_field = omni.ui.StringField()
                    self._base_path_field.model.set_value("")
                with omni.ui.HStack(spacing=10):
                    omni.ui.Label("Position:", width=100)
                    self._pos_x_field = omni.ui.FloatField()
                    self._pos_x_field.model.set_value(0)
                    self._pos_y_field = omni.ui.FloatField()
                    self._pos_y_field.model.set_value(0)
                    self._pos_z_field = omni.ui.FloatField()
                    self._pos_z_field.model.set_value(0.1)
                omni.ui.Button("Add Brick", clicked_fn=self._add_brick_clicked)
                omni.ui.Button("Reset Env", clicked_fn=self._reset_env_clicked)

    def destroy(self):
        self._window.destroy()    

    def _add_brick_clicked(self):
        width = self._dim_x_field.model.as_int
        length = self._dim_y_field.model.as_int
        height = self._dim_z_field.model.as_int
        color = self._color_options[self._color_combo.model.get_item_value_model().as_int]
        pos_x = self._pos_x_field.model.as_float
        pos_y = self._pos_y_field.model.as_float
        pos_z = self._pos_z_field.model.as_float

        env_id_str = self._base_path_field.model.as_string
        env_id = int(env_id_str) if env_id_str else None
        get_brick_physics_interface().create_brick(
            dimensions=(width, length, height),
            color_name=color,
            env_id=env_id,
            pos=(pos_x, pos_y, pos_z),
        )

    def _reset_env_clicked(self):
        env_id_str = self._base_path_field.model.as_string
        env_id = int(env_id_str) if env_id_str else None
        get_brick_physics_interface().reset_env(env_id)
