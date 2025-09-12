import omni.ui
import omni.physx
import lego_assemble.physics.lego_schemes as lego_schemes
from lego_assemble.physics.interface import get_brick_physics_interface
from lego_assemble import _native

class LegoUI():
    def __init__(self):
        self._window = omni.ui.Window("LEGO Assemble", width=300, height=300)
        self._window.deferred_dock_in("Console")
        with self._window.frame:
            # Split the window into two vertical panels: left = original settings,
            # right = inv mass/inertia scale settings.
            with omni.ui.HStack(height=0, spacing=15):
                # Left: existing controls
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
                    omni.ui.Button("Save to USD", clicked_fn=self._save_to_usd)

                # Right: inv mass/inertia scale settings
                with omni.ui.VStack(height=0, spacing=5):
                    with omni.ui.HStack(spacing=10):
                        omni.ui.Label("inv_mass0:", width=100)
                        self._inv_mass0_field = omni.ui.FloatDrag(min=0.0, max=100.0)
                        self._inv_mass0_field.model.set_value(0.2)
                    with omni.ui.HStack(spacing=10):
                        omni.ui.Label("inv_inertia0:", width=100)
                        self._inv_inertia0_field = omni.ui.FloatDrag(min=0.0, max=100.0)
                        self._inv_inertia0_field.model.set_value(0.2)
                    with omni.ui.HStack(spacing=10):
                        omni.ui.Label("inv_mass1:", width=100)
                        self._inv_mass1_field = omni.ui.FloatDrag(min=0.0, max=100.0)
                        self._inv_mass1_field.model.set_value(1.0)
                    with omni.ui.HStack(spacing=10):
                        omni.ui.Label("inv_inertia1:", width=100)
                        self._inv_inertia1_field = omni.ui.FloatDrag(min=0.0, max=100.0)
                        self._inv_inertia1_field.model.set_value(1.0)

                    # Apply automatically when any field changes
                    for m in (
                        self._inv_mass0_field.model,
                        self._inv_inertia0_field.model,
                        self._inv_mass1_field.model,
                        self._inv_inertia1_field.model,
                    ):
                        m.add_value_changed_fn(self._apply_inv_mass_inertia_scales)

            # Apply initial defaults
            self._apply_inv_mass_inertia_scales()

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

    def _save_to_usd(self):
        omni.physx.get_physx_interface().update_transformations(True, True, True, True)
        omni.physx.get_physx_interface().release_physics_objects()
        omni.physx.get_physx_interface().force_load_physics_from_usd()

    def _apply_inv_mass_inertia_scales(self, *_args):
        inv_mass0 = self._inv_mass0_field.model.as_float
        inv_inertia0 = self._inv_inertia0_field.model.as_float
        inv_mass1 = self._inv_mass1_field.model.as_float
        inv_inertia1 = self._inv_inertia1_field.model.as_float
        _native.set_default_lego_joint_inv_mass_inertia(
            inv_mass0, inv_inertia0, inv_mass1, inv_inertia1
        )
