import carb
import math
import omni.client
import omni.ui
from lego_assemble.colors import parse_color, Colors
from lego_assemble.ui.force_monitor import ForceMonitor
from lego_assemble._native import (
    allocate_brick_part,
    deallocate_all_managed,
    export_lego,
    import_lego,
    get_assembly_thresholds,
    set_assembly_thresholds,
)
from omni.kit.window.filepicker import FilePickerDialog

class LegoUI():
    def __init__(self):
        self._window = omni.ui.Window("LEGO Assemble", width=300, height=300)
        self._window.deferred_dock_in("Console")
        self._export_dialog = None
        self._import_dialog = None
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
                    omni.ui.Button("Import", clicked_fn=self._import)
                    omni.ui.Button("Export", clicked_fn=self._export)

                # Middle: thresholds (top) + inv mass/inertia scale settings (bottom)
                with omni.ui.VStack(height=0, spacing=8):
                    # Read current native thresholds and use them to initialize the UI
                    _thr = get_assembly_thresholds()
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

                # TODO: currently disabled
                # # Right: monitoring column (delegated)
                # with omni.ui.VStack(height=0, spacing=5):
                #     self._monitor = ForceMonitor()

    def destroy(self):
        # self._monitor.destroy()
        self._window.destroy()

    def get_env_id(self) -> int:
        """Return the current env_id from the main UI."""
        env_id_str = self._base_path_field.model.as_string
        return int(env_id_str) if env_id_str else -1

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
        brick_path = allocate_brick_part(
            dimensions=(width, length, height),
            color=parse_color(color),
            env_id=env_id,
            rot=(1.0, 0.0, 0.0, 0.0),
            pos=(pos_x, pos_y, pos_z),
        )

    def _reset_env_clicked(self):
        env_id_str = self._base_path_field.model.as_string
        env_id = int(env_id_str) if env_id_str else -1
        deallocate_all_managed(env_id)

    def _export(self):
        if self._export_dialog is None:
            self._export_dialog = FilePickerDialog(
                "Export",
                apply_button_label="Save",
                click_apply_handler=self._on_export_dialog_apply,
            )
        self._export_dialog.set_file_extension(".json")
        self._export_dialog.set_filename("lego_topology.json")
        self._export_dialog.show()

    def _on_export_dialog_apply(self, filename: str, dirname: str):
        env_id_str = self._base_path_field.model.as_string
        env_id = int(env_id_str) if env_id_str else -1
        dirname = dirname.strip()
        if dirname and not dirname.endswith("/"):
            dirname += "/"
        fullpath = f"{dirname}{filename}"
        topology = export_lego(env_id)
        if fullpath.startswith("omniverse://") or fullpath.startswith("omni://"):
            omni.client.write_file(fullpath, topology.encode("utf-8"))
        else:
            with open(fullpath, "w", encoding="utf-8") as f:
                f.write(topology)
        carb.log_info(f"Exported topology to {fullpath}")
        self._export_dialog.hide()

    def _import(self):
        if self._import_dialog is None:
            self._import_dialog = FilePickerDialog(
                "Import",
                apply_button_label="Open",
                click_apply_handler=self._on_import_dialog_apply,
            )
        self._import_dialog.set_file_extension(".json")
        self._import_dialog.show()

    def _on_import_dialog_apply(self, filename: str, dirname: str):
        env_id_str = self._base_path_field.model.as_string
        env_id = int(env_id_str) if env_id_str else -1
        dirname = dirname.strip()
        if dirname and not dirname.endswith("/"):
            dirname += "/"
        fullpath = f"{dirname}{filename}"
        if fullpath.startswith("omniverse://") or fullpath.startswith("omni://"):
            result, content = omni.client.read_file(fullpath)
            if result != omni.client.Result.OK:
                raise RuntimeError(f"Failed to read {fullpath}: {result}")
            topology = memoryview(content).tobytes().decode("utf-8")
        else:
            with open(fullpath, "r", encoding="utf-8") as f:
                topology = f.read()
        # Reference transform uses quaternion order wxyz and stage units.
        import_lego(topology, env_id, (1.0, 0.0, 0.0, 0.0), (0.0, 0.0, 0.0))
        carb.log_info(f"Imported topology from {fullpath}")
        self._import_dialog.hide()

    def _set_threshold(self, name: str, value: float):
        thr = get_assembly_thresholds()
        setattr(thr, name, value)
        set_assembly_thresholds(thr)
