"""Main BrickSim Omniverse UI window."""

import asyncio
import json
import math
import os
import tempfile

import carb
import carb.settings
import omni.ui
import omni.usd
from isaacsim.core.utils.stage import get_current_stage

from bricksim import kit_runner
from bricksim.colors import Colors, parse_color
from bricksim.core import (
    allocate_brick_part,
    allocate_unmanaged_brick_part,
    arrange_parts_in_workspace,
    deallocate_all_managed,
    export_lego,
    get_assembly_thresholds,
    get_breakage_thresholds,
    get_physx_id_mappings,
    get_sync_to_usd,
    get_usd_id_mappings,
    import_lego,
    set_assembly_thresholds,
    set_breakage_thresholds,
    set_sync_to_usd,
    update_part_prototypes,
)
from bricksim.topology.legolization import (
    is_legolization_json,
    legolization_json_to_topology_json,
)
from bricksim.topology.stabletext2brick import (
    bricks_text_to_topology_json,
    is_bricks_text,
)
from bricksim.utils.sim import wait_for_next_update
from bricksim.utils.usd_parse import get_brick_dimensions, get_env_path

from .file_picker import show_file_picker_dialog

_HOT_RELOAD_SETTING = "/app/bricksim/kit_runner/has_target"


class LegoUI:
    """BrickSim settings and import/export UI."""

    def __init__(self):
        """Create the BrickSim settings window and controls."""
        self._window = omni.ui.Window("BrickSim Settings", width=300, height=300)
        self._window.deferred_dock_in("Console")
        self._hot_reload_button = None
        self._settings = carb.settings.get_settings()
        self._hot_reload_sub = self._settings.subscribe_to_node_change_events(
            _HOT_RELOAD_SETTING, self._on_hot_reload_setting_changed
        )
        with self._window.frame:
            with omni.ui.HStack(height=0, spacing=15):
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
                        self._color_combo = omni.ui.ComboBox(
                            self._color_options.index("Pink"), *self._color_options
                        )
                    with omni.ui.HStack(spacing=10):
                        omni.ui.Label("Env id:", width=100)
                        self._base_path_field = omni.ui.StringField()
                        self._base_path_field.model.set_value("")
                    omni.ui.Button("Add Brick", clicked_fn=self._add_brick_clicked)
                    omni.ui.Button("Reset Env", clicked_fn=self._reset_env_clicked)
                    omni.ui.Button(
                        "Import",
                        clicked_fn=lambda: asyncio.ensure_future(self._import_async()),
                    )
                    omni.ui.Button(
                        "Export",
                        clicked_fn=lambda: asyncio.ensure_future(self._export_async()),
                    )
                    omni.ui.Button(
                        "Set Color",
                        clicked_fn=lambda: asyncio.ensure_future(
                            self._set_bricks_color()
                        ),
                    )
                    omni.ui.Button(
                        "Update Prototypes",
                        clicked_fn=lambda: asyncio.ensure_future(
                            self._update_part_prototypes()
                        ),
                    )
                    # Hot reload button for demo iteration. Visible only when a target
                    # has been run via kit_runner (driven by carb settings).
                    self._hot_reload_button = omni.ui.Button(
                        "Hot Reload",
                        clicked_fn=self._hot_reload_clicked,
                    )
                    enabled = bool(self._settings.get(_HOT_RELOAD_SETTING))
                    self._hot_reload_button.visible = enabled

                with omni.ui.VStack(height=0, spacing=8):
                    _thr = get_assembly_thresholds()
                    with omni.ui.HStack(spacing=10):
                        omni.ui.Label("Enable assembly check", width=140)
                        self._assembly_enabled_model = omni.ui.SimpleBoolModel()
                        self._assembly_enabled_model.set_value(bool(_thr.enabled))
                        omni.ui.CheckBox(model=self._assembly_enabled_model)
                        self._assembly_enabled_model.add_value_changed_fn(
                            lambda m: self._set_threshold(
                                "enabled", bool(m.get_value_as_bool())
                            )
                        )
                    with omni.ui.HStack(spacing=10):
                        omni.ui.Label("Distance tol (m):", width=140)
                        self._dist_tol_field = omni.ui.FloatDrag(min=0.0, max=0.05)
                        self._dist_tol_field.model.set_value(
                            float(_thr.distance_tolerance)
                        )
                        self._dist_tol_field.model.add_value_changed_fn(
                            lambda m: self._set_threshold(
                                "distance_tolerance", float(m.as_float)
                            )
                        )
                    with omni.ui.HStack(spacing=10):
                        omni.ui.Label("Max penetration (m):", width=140)
                        self._max_pen_field = omni.ui.FloatDrag(min=0.0, max=0.05)
                        self._max_pen_field.model.set_value(float(_thr.max_penetration))
                        self._max_pen_field.model.add_value_changed_fn(
                            lambda m: self._set_threshold(
                                "max_penetration", float(m.as_float)
                            )
                        )
                    with omni.ui.HStack(spacing=10):
                        omni.ui.Label("Z angle tol (deg):", width=140)
                        self._zang_deg_field = omni.ui.FloatDrag(min=0.0, max=90.0)
                        self._zang_deg_field.model.set_value(
                            float(math.degrees(_thr.z_angle_tolerance))
                        )
                        self._zang_deg_field.model.add_value_changed_fn(
                            lambda m: self._set_threshold(
                                "z_angle_tolerance", math.radians(float(m.as_float))
                            )
                        )
                    with omni.ui.HStack(spacing=10):
                        omni.ui.Label("Required force (N):", width=140)
                        self._req_force_field = omni.ui.FloatDrag(min=0.0, max=10.0)
                        self._req_force_field.model.set_value(
                            float(_thr.required_force)
                        )
                        self._req_force_field.model.add_value_changed_fn(
                            lambda m: self._set_threshold(
                                "required_force", float(m.as_float)
                            )
                        )
                    with omni.ui.HStack(spacing=10):
                        omni.ui.Label("Yaw tol (deg):", width=140)
                        self._yaw_deg_field = omni.ui.FloatDrag(min=0.0, max=180.0)
                        self._yaw_deg_field.model.set_value(
                            float(math.degrees(_thr.yaw_tolerance))
                        )
                        self._yaw_deg_field.model.add_value_changed_fn(
                            lambda m: self._set_threshold(
                                "yaw_tolerance", math.radians(float(m.as_float))
                            )
                        )
                    with omni.ui.HStack(spacing=10):
                        omni.ui.Label("Position tol (m):", width=140)
                        self._pos_tol_field = omni.ui.FloatDrag(min=0.0, max=0.05)
                        self._pos_tol_field.model.set_value(
                            float(_thr.position_tolerance)
                        )
                        self._pos_tol_field.model.add_value_changed_fn(
                            lambda m: self._set_threshold(
                                "position_tolerance", float(m.as_float)
                            )
                        )
                    with omni.ui.HStack(spacing=10):
                        omni.ui.Label("Sync connections to USD", width=140)
                        self._sync_usd_model = omni.ui.SimpleBoolModel()
                        self._sync_usd_model.set_value(bool(get_sync_to_usd()))
                        omni.ui.CheckBox(model=self._sync_usd_model)
                        self._sync_usd_model.add_value_changed_fn(
                            lambda m: set_sync_to_usd(bool(m.get_value_as_bool()))
                        )

                with omni.ui.VStack(height=0, spacing=8):
                    _bthr = get_breakage_thresholds()
                    with omni.ui.HStack(spacing=10):
                        omni.ui.Label("Enable breakage check", width=140)
                        self._breakage_enabled_model = omni.ui.SimpleBoolModel()
                        self._breakage_enabled_model.set_value(bool(_bthr.enabled))
                        omni.ui.CheckBox(model=self._breakage_enabled_model)
                        self._breakage_enabled_model.add_value_changed_fn(
                            lambda m: self._set_breakage_threshold(
                                "enabled", bool(m.get_value_as_bool())
                            )
                        )
                    with omni.ui.HStack(spacing=10):
                        omni.ui.Label("Contact regularization:", width=140)
                        self._contact_reg_field = omni.ui.FloatDrag(min=0.0, max=10.0)
                        self._contact_reg_field.model.set_value(
                            float(_bthr.contact_regularization)
                        )
                        self._contact_reg_field.model.add_value_changed_fn(
                            lambda m: self._set_breakage_threshold(
                                "contact_regularization", float(m.as_float)
                            )
                        )
                    with omni.ui.HStack(spacing=10):
                        omni.ui.Label("Clutch axial comp:", width=140)
                        self._clutch_axial_comp_field = omni.ui.FloatDrag(
                            min=0.0, max=1e6
                        )
                        self._clutch_axial_comp_field.model.set_value(
                            float(_bthr.clutch_axial_compliance)
                        )
                        self._clutch_axial_comp_field.model.add_value_changed_fn(
                            lambda m: self._set_breakage_threshold(
                                "clutch_axial_compliance", float(m.as_float)
                            )
                        )
                    with omni.ui.HStack(spacing=10):
                        omni.ui.Label("Clutch radial comp:", width=140)
                        self._clutch_radial_comp_field = omni.ui.FloatDrag(
                            min=0.0, max=1e6
                        )
                        self._clutch_radial_comp_field.model.set_value(
                            float(_bthr.clutch_radial_compliance)
                        )
                        self._clutch_radial_comp_field.model.add_value_changed_fn(
                            lambda m: self._set_breakage_threshold(
                                "clutch_radial_compliance", float(m.as_float)
                            )
                        )
                    with omni.ui.HStack(spacing=10):
                        omni.ui.Label("Clutch tangential comp:", width=140)
                        self._clutch_tang_comp_field = omni.ui.FloatDrag(
                            min=0.0, max=1e6
                        )
                        self._clutch_tang_comp_field.model.set_value(
                            float(_bthr.clutch_tangential_compliance)
                        )
                        self._clutch_tang_comp_field.model.add_value_changed_fn(
                            lambda m: self._set_breakage_threshold(
                                "clutch_tangential_compliance", float(m.as_float)
                            )
                        )
                    with omni.ui.HStack(spacing=10):
                        omni.ui.Label("Friction coefficient:", width=140)
                        self._friction_coefficient_field = omni.ui.FloatDrag(
                            min=0.0, max=100.0
                        )
                        self._friction_coefficient_field.model.set_value(
                            float(_bthr.friction_coefficient)
                        )
                        self._friction_coefficient_field.model.add_value_changed_fn(
                            lambda m: self._set_breakage_threshold(
                                "friction_coefficient", float(m.as_float)
                            )
                        )
                    with omni.ui.HStack(spacing=10):
                        omni.ui.Label("Preloaded force:", width=140)
                        self._preloaded_force_field = omni.ui.FloatDrag(
                            min=0.0, max=100.0
                        )
                        self._preloaded_force_field.model.set_value(
                            float(_bthr.preloaded_force)
                        )
                        self._preloaded_force_field.model.add_value_changed_fn(
                            lambda m: self._set_breakage_threshold(
                                "preloaded_force", float(m.as_float)
                            )
                        )
                    with omni.ui.HStack(spacing=10):
                        omni.ui.Label("Breakage cooldown time:", width=140)
                        self._breakage_cooldown_field = omni.ui.FloatDrag(
                            min=0.0, max=1.0
                        )
                        self._breakage_cooldown_field.model.set_value(
                            float(_bthr.breakage_cooldown_time)
                        )
                        self._breakage_cooldown_field.model.add_value_changed_fn(
                            lambda m: self._set_breakage_threshold(
                                "breakage_cooldown_time", float(m.as_float)
                            )
                        )
                    with omni.ui.HStack(spacing=10):
                        omni.ui.Label("Debug dump", width=140)
                        self._debug_dump_model = omni.ui.SimpleBoolModel()
                        self._debug_dump_model.set_value(bool(_bthr.debug_dump))
                        omni.ui.CheckBox(model=self._debug_dump_model)
                        self._debug_dump_model.add_value_changed_fn(
                            lambda m: self._set_breakage_threshold(
                                "debug_dump", bool(m.get_value_as_bool())
                            )
                        )

                    omni.ui.Button(
                        "Dump ID Mappings", clicked_fn=self._dump_id_mappings
                    )

    def destroy(self):
        """Destroy the BrickSim settings window."""
        # self._monitor.destroy()
        self._window.destroy()
        self._hot_reload_button = None

    def get_env_id(self) -> int:
        """Return the current env_id from the main UI."""
        env_id_str = self._base_path_field.model.as_string
        return int(env_id_str) if env_id_str else -1

    def get_selected_color(self) -> tuple[int, int, int]:
        """Return the currently selected color as an RGB tuple."""
        color = self._color_options[
            self._color_combo.model.get_item_value_model().as_int
        ]
        return parse_color(color)

    def _add_brick_clicked(self):
        width = self._dim_x_field.model.as_int
        length = self._dim_y_field.model.as_int
        height = self._dim_z_field.model.as_int

        env_id_str = self._base_path_field.model.as_string
        env_id = int(env_id_str) if env_id_str else -1
        brick_path = allocate_brick_part(
            dimensions=(width, length, height),
            color=self.get_selected_color(),
            env_id=env_id,
        )
        carb.log_info(f"Added brick {brick_path} to env {env_id}")

        workspace_path = get_env_path(env_id) + "/LegoWorkspace"
        try:
            _, not_placed = arrange_parts_in_workspace(workspace_path, [brick_path])
        except Exception as exc:
            carb.log_warn(f"Failed to arrange part in workspace: {exc}")
            return
        if not_placed:
            carb.log_warn(
                f"Could not place part {not_placed} in workspace {workspace_path}"
            )

    def _reset_env_clicked(self):
        env_id_str = self._base_path_field.model.as_string
        env_id = int(env_id_str) if env_id_str else -1
        deallocate_all_managed(env_id)

    async def _export_async(self):
        success, filename, dirname = await show_file_picker_dialog(
            "Export",
            apply_button_label="Save",
            file_extension=".json",
            default_filename="lego_topology.json",
        )
        if not success:
            return

        assert filename is not None and dirname is not None
        env_id = self.get_env_id()
        topology = export_lego(env_id)
        fullpath = os.path.join(dirname, filename)
        with open(fullpath, "w", encoding="utf-8") as f:
            json.dump(topology, f, indent=2)
        carb.log_info(f"Exported topology to {fullpath}")

    async def _import_async(self):
        success, filename, dirname = await show_file_picker_dialog(
            "Import",
            apply_button_label="Open",
            file_extension=".json",
        )
        if not success:
            return

        assert filename is not None and dirname is not None
        fullpath = os.path.join(dirname, filename)
        with open(fullpath, "r", encoding="utf-8") as f:
            text = f.read()

        if is_bricks_text(text):
            # StableText2Brick format
            carb.log_info(
                "Detected StableText2Brick format, converting to topology JSON"
            )
            topology = bricks_text_to_topology_json(
                text, color=self.get_selected_color()
            )
        elif is_legolization_json(text):
            carb.log_info("Detected legolization format, converting to topology JSON")
            topology = legolization_json_to_topology_json(
                json.loads(text), color=self.get_selected_color()
            )
        else:
            # Assume direct topology JSON
            carb.log_info("Assuming direct topology JSON format")
            topology = json.loads(text)

        env_id = self.get_env_id()
        part_paths, _ = import_lego(topology, env_id)
        imported_parts = [part_paths[k] for k in sorted(part_paths)]
        carb.log_info(f"Imported topology from {fullpath}")

        workspace_path = get_env_path(env_id) + "/LegoWorkspace"
        try:
            _, not_placed = arrange_parts_in_workspace(
                workspace_path, imported_parts, [1] * len(imported_parts)
            )
        except Exception as exc:
            carb.log_warn(f"Failed to arrange parts in workspace: {exc}")
            return
        if not_placed:
            carb.log_warn(
                f"Could not place parts {not_placed} in workspace {workspace_path}"
            )

    def _set_threshold(self, name: str, value: float):
        thr = get_assembly_thresholds()
        setattr(thr, name, value)
        set_assembly_thresholds(thr)

    def _set_breakage_threshold(self, name: str, value: float):
        thr = get_breakage_thresholds()
        setattr(thr, name, value)
        set_breakage_thresholds(thr)

    def _on_hot_reload_setting_changed(self, *args):
        if self._hot_reload_button is None:
            return
        enabled = bool(self._settings.get(_HOT_RELOAD_SETTING))
        self._hot_reload_button.visible = enabled

    def _hot_reload_clicked(self):
        try:
            kit_runner.rerun()
        except Exception as exc:
            carb.log_error(f"Hot reload failed: {exc}")

    def _dump_id_mappings(self):
        usd_part_map, usd_conn_map = get_usd_id_mappings()
        try:
            physx_part_map, physx_conn_map = get_physx_id_mappings()
        except Exception as exc:
            carb.log_warn(f"Failed to get PhysX ID mappings: {exc}")
            physx_part_map, physx_conn_map = {}, {}
        print(f"USD Part ID Map: {usd_part_map}")
        print(f"USD ConnectionSegment ID Map: {usd_conn_map}")
        print(f"PhysX Part ID Map: {physx_part_map}")
        print(f"PhysX ConnectionSegment ID Map: {physx_conn_map}")
        result = {
            "usd_part_map": usd_part_map,
            "usd_conn_map": usd_conn_map,
            "physx_part_map": physx_part_map,
            "physx_conn_map": physx_conn_map,
        }
        with tempfile.NamedTemporaryFile(
            delete=False,
            prefix="id_mappings_",
            suffix=".json",
            mode="w",
            encoding="utf-8",
        ) as f:
            json.dump(result, f, indent=2)
            print(f"Dumped ID mappings to {f.name}")

    async def _update_part_prototypes(self):
        update_part_prototypes()
        # Force full USD stage resync
        stage = get_current_stage()
        world = stage.GetDefaultPrim()
        world.SetActive(False)
        await wait_for_next_update()
        world.ClearActive()

    async def _set_bricks_color(self):
        color = self.get_selected_color()
        selected_paths = (
            omni.usd.get_context().get_selection().get_selected_prim_paths()
        )
        stage = get_current_stage()
        paths_to_resync = []
        for path in selected_paths:
            prim = stage.GetPrimAtPath(path)
            if not prim.IsActive():
                continue
            dimensions = get_brick_dimensions(path)
            if dimensions is None:
                continue
            allocate_unmanaged_brick_part(dimensions=dimensions, color=color, path=path)
            # Force resync
            active_authored = prim.HasAuthoredActive()
            prim.SetActive(False)
            paths_to_resync.append((prim, active_authored))
        await wait_for_next_update()
        for prim, active_authored in paths_to_resync:
            if active_authored:
                prim.SetActive(True)
            else:
                prim.ClearActive()
