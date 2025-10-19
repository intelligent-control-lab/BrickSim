import carb
import omni.ui
import omni.usd
import omni.physx
import math
import omni.physx.scripts.physicsUtils as physicsUtils 
from pxr import Gf, UsdGeom
from lego_assemble.colors import parse_color, Colors
from lego_assemble.ui.force_monitor import ForceMonitor
from lego_assemble._native import export_lego_topology, get_lego_thresholds, set_lego_thresholds, allocate_brick, deallocate_all_bricks_in_env

class LegoUI():
    def __init__(self):
        self._window = omni.ui.Window("LEGO Assemble", width=300, height=300)
        self._window.deferred_dock_in("Console")
        with self._window.frame:
            # Split the window into two vertical panels: left = original settings,
            # middle = inv mass/inertia settings, right = live monitor.
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
                        self._color_options = list(Colors.keys())
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
                    omni.ui.Button("Export", clicked_fn=self._export)

                # Middle: thresholds (top) + inv mass/inertia scale settings (bottom)
                with omni.ui.VStack(height=0, spacing=8):
                    # Read current native thresholds and use them to initialize the UI
                    _thr = get_lego_thresholds()
                    with omni.ui.HStack(spacing=10):
                        omni.ui.Label("Distance tol (m):", width=140)
                        self._dist_tol_field = omni.ui.FloatDrag(min=0.0, max=0.05)
                        self._dist_tol_field.model.set_value(float(_thr.distance_tolerance))
                        self._dist_tol_field.model.add_value_changed_fn(
                            lambda m: self._set_threshold("distance_tolerance", float(m.as_float))
                        )
                    with omni.ui.HStack(spacing=10):
                        omni.ui.Label("Max penetration (m):", width=140)
                        self._max_pen_field = omni.ui.FloatDrag(min=0.0, max=0.05)
                        self._max_pen_field.model.set_value(float(_thr.max_penetration))
                        self._max_pen_field.model.add_value_changed_fn(
                            lambda m: self._set_threshold("max_penetration", float(m.as_float))
                        )
                    with omni.ui.HStack(spacing=10):
                        omni.ui.Label("Z angle tol (deg):", width=140)
                        self._zang_deg_field = omni.ui.FloatDrag(min=0.0, max=90.0)
                        self._zang_deg_field.model.set_value(float(math.degrees(_thr.z_angle_tolerance)))
                        self._zang_deg_field.model.add_value_changed_fn(
                            lambda m: self._set_threshold("z_angle_tolerance", math.radians(float(m.as_float)))
                        )
                    with omni.ui.HStack(spacing=10):
                        omni.ui.Label("Required force (N):", width=140)
                        self._req_force_field = omni.ui.FloatDrag(min=0.0, max=10.0)
                        self._req_force_field.model.set_value(float(_thr.required_force))
                        self._req_force_field.model.add_value_changed_fn(
                            lambda m: self._set_threshold("required_force", float(m.as_float))
                        )
                    with omni.ui.HStack(spacing=10):
                        omni.ui.Label("Yaw tol (deg):", width=140)
                        self._yaw_deg_field = omni.ui.FloatDrag(min=0.0, max=180.0)
                        self._yaw_deg_field.model.set_value(float(math.degrees(_thr.yaw_tolerance)))
                        self._yaw_deg_field.model.add_value_changed_fn(
                            lambda m: self._set_threshold("yaw_tolerance", math.radians(float(m.as_float)))
                        )
                    with omni.ui.HStack(spacing=10):
                        omni.ui.Label("Position tol (m):", width=140)
                        self._pos_tol_field = omni.ui.FloatDrag(min=0.0, max=0.05)
                        self._pos_tol_field.model.set_value(float(_thr.position_tolerance))
                        self._pos_tol_field.model.add_value_changed_fn(
                            lambda m: self._set_threshold("position_tolerance", float(m.as_float))
                        )

                # Right: monitoring column (delegated)
                with omni.ui.VStack(height=0, spacing=5):
                    self._monitor = ForceMonitor()

    def destroy(self):
        self._monitor.destroy()
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
        env_id = int(env_id_str) if env_id_str else -1
        brick_id, brick_path = allocate_brick(
            dimensions=(width, length, height),
            color=parse_color(color),
            env_id=env_id,
        )
        prim = omni.usd.get_context().get_stage().GetPrimAtPath(brick_path)
        physicsUtils.set_or_add_translate_op(UsdGeom.Xformable(prim), Gf.Vec3f(pos_x, pos_y, pos_z))

    def _reset_env_clicked(self):
        env_id_str = self._base_path_field.model.as_string
        env_id = int(env_id_str) if env_id_str else -1
        deallocate_all_bricks_in_env(env_id)

    def _save_to_usd(self):
        omni.physx.get_physx_interface().update_transformations(True, True, True, True)
        omni.physx.get_physx_interface().release_physics_objects()
        omni.physx.get_physx_interface().force_load_physics_from_usd()

    def _export(self):
        env_id_str = self._base_path_field.model.as_string
        root_path = f"/World/envs/env_{env_id_str}" if env_id_str else "/World"
        topology = export_lego_topology(root_path)
        carb.log_info(f"Exported topology: {topology}")

    def _set_threshold(self, name: str, value: float):
        thr = get_lego_thresholds()
        setattr(thr, name, value)
        set_lego_thresholds(thr)
