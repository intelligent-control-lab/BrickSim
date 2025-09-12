import omni.ui
import omni.physx
import omni.usd
import omni.kit.app
from pxr import Usd, UsdPhysics
import lego_assemble.physics.lego_schemes as lego_schemes
from lego_assemble.physics.interface import get_brick_physics_interface
from lego_assemble import _native

class LegoUI():
    def __init__(self):
        self._window = omni.ui.Window("LEGO Assemble", width=300, height=300)
        self._window.deferred_dock_in("Console")
        self._selected_joint_path: str | None = None
        self._conn_paths: list[str] = []
        self._update_sub = None
        self._stage_event_sub = None
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

                # Middle: inv mass/inertia scale settings
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

                # Right: monitoring column
                with omni.ui.VStack(height=0, spacing=5):
                    omni.ui.Label("Connection Monitor", height=0)
                    with omni.ui.HStack(spacing=10):
                        self._conn_combo = None
                        self._conn_combo_frame = omni.ui.Frame()
                        def _rebuild_combo():
                            items = ["None", *self._conn_paths]
                            idx = 0
                            if self._selected_joint_path in self._conn_paths:
                                idx = 1 + self._conn_paths.index(self._selected_joint_path)
                            self._conn_combo = omni.ui.ComboBox(idx, *items)
                            self._conn_combo.model.add_item_changed_fn(self._on_conn_combo_changed)
                        self._conn_combo_frame.set_build_fn(_rebuild_combo)
                        omni.ui.Button("Refresh", clicked_fn=self._refresh_connections, width=100)
                    # Display area: split into 2 columns (Force | Torque).
                    with omni.ui.HStack(spacing=20, height=0):
                        # Force column
                        with omni.ui.VStack(spacing=4, height=0):
                            omni.ui.Label("Force (N)")
                            with omni.ui.HStack():
                                omni.ui.Label("X:", width=16)
                                self._force_x_label = omni.ui.Label(
                                    "-", width=110, alignment=omni.ui.Alignment.RIGHT_CENTER
                                )
                            with omni.ui.HStack():
                                omni.ui.Label("Y:", width=16)
                                self._force_y_label = omni.ui.Label(
                                    "-", width=110, alignment=omni.ui.Alignment.RIGHT_CENTER
                                )
                            with omni.ui.HStack():
                                omni.ui.Label("Z:", width=16)
                                self._force_z_label = omni.ui.Label(
                                    "-", width=110, alignment=omni.ui.Alignment.RIGHT_CENTER
                                )
                        # Torque column
                        with omni.ui.VStack(spacing=4, height=0):
                            omni.ui.Label("Torque (N·m)")
                            with omni.ui.HStack():
                                omni.ui.Label("X:", width=16)
                                self._torque_x_label = omni.ui.Label(
                                    "-", width=110, alignment=omni.ui.Alignment.RIGHT_CENTER
                                )
                            with omni.ui.HStack():
                                omni.ui.Label("Y:", width=16)
                                self._torque_y_label = omni.ui.Label(
                                    "-", width=110, alignment=omni.ui.Alignment.RIGHT_CENTER
                                )
                            with omni.ui.HStack():
                                omni.ui.Label("Z:", width=16)
                                self._torque_z_label = omni.ui.Label(
                                    "-", width=110, alignment=omni.ui.Alignment.RIGHT_CENTER
                                )

            # Apply initial defaults
            self._apply_inv_mass_inertia_scales()
            # Build initial joint list and start update subscription
            self._refresh_connections()
            self._start_monitoring_updates()
            self._start_stage_selection_sync()

    def destroy(self):
        if self._update_sub is not None:
            self._update_sub.unsubscribe()
            self._update_sub = None
        if self._stage_event_sub is not None:
            self._stage_event_sub.unsubscribe()
            self._stage_event_sub = None
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

    # ----- Monitoring helpers -----
    def _list_lego_connections(self) -> list[str]:
        stage: Usd.Stage = omni.usd.get_context().get_stage()
        if stage is None:
            return []
        out: list[str] = []
        for prim in stage.Traverse():
            if not prim.IsValid():
                continue
            if not prim.IsA(UsdPhysics.FixedJoint):
                continue
            attr = prim.GetAttribute("lego_conn")
            if not attr or not attr.IsValid():
                continue
            try:
                if bool(attr.Get()):
                    out.append(str(prim.GetPath()))
            except Exception:
                # Fail fast: skip unreadable attr
                continue
        return out

    def _refresh_connections(self):
        self._conn_paths = self._list_lego_connections()
        # Keep selection if it still exists; otherwise clear
        if self._selected_joint_path not in self._conn_paths:
            self._select_joint(None)
        # Rebuild combobox with updated list
        if hasattr(self, "_conn_combo_frame") and self._conn_combo_frame is not None:
            self._conn_combo_frame.rebuild()

    def _select_joint(self, path: str | None):
        self._selected_joint_path = path

    def _on_conn_combo_changed(self, model, _val):
        idx = model.get_item_value_model().as_int
        path = None if idx == 0 else self._conn_paths[idx - 1]
        self._select_joint(path)

    def _start_monitoring_updates(self):
        app = omni.kit.app.get_app()
        stream = app.get_update_event_stream()
        self._update_sub = stream.create_subscription_to_pop(self._on_update)

    def _start_stage_selection_sync(self):
        # Subscribe to Stage selection changes so selecting a lego joint auto-selects here
        ctx = omni.usd.get_context()
        stream = ctx.get_stage_event_stream()
        self._stage_event_sub = stream.create_subscription_to_pop_by_type(
            int(omni.usd.StageEventType.SELECTION_CHANGED), self._on_stage_selection_changed
        )

    def _on_update(self, _e):
        path = self._selected_joint_path
        if not path:
            self._set_force_labels(None)
            self._set_torque_labels(None)
            return
        try:
            ft = _native.get_physx_joint_force_torque(path)
        except Exception:
            ft = None
        if not ft:
            self._set_force_labels(None)
            self._set_torque_labels(None)
            return
        (fx, fy, fz), (tx, ty, tz) = ft
        self._set_force_labels((fx, fy, fz))
        self._set_torque_labels((tx, ty, tz))

    def _on_stage_selection_changed(self, _event):
        stage = omni.usd.get_context().get_stage()
        if not stage:
            return
        sel = omni.usd.get_context().get_selection()
        if not sel:
            return
        paths = sel.get_selected_prim_paths()
        if not paths:
            return
        for p in paths:
            prim = stage.GetPrimAtPath(p)
            if not prim or not prim.IsA(UsdPhysics.FixedJoint):
                continue
            attr = prim.GetAttribute("lego_conn")
            if not attr or not attr.IsValid():
                continue
            try:
                if not bool(attr.Get()):
                    continue
            except Exception:
                continue
            # Ensure list contains it and update UI selection
            self._refresh_connections()
            self._select_joint(p)
            if getattr(self, "_conn_combo", None) is None and getattr(self, "_conn_combo_frame", None) is not None:
                self._conn_combo_frame.rebuild()
            if getattr(self, "_conn_combo", None) is not None:
                idx = 1 + self._conn_paths.index(p) if p in self._conn_paths else 0
                try:
                    self._conn_combo.model.get_item_value_model().set_value(idx)
                except Exception:
                    pass
            break

    def _set_force_labels(self, vec3):
        if not vec3:
            self._force_x_label.text = "-"
            self._force_y_label.text = "-"
            self._force_z_label.text = "-"
            return
        fx, fy, fz = vec3
        self._force_x_label.text = format(fx, ".6f")
        self._force_y_label.text = format(fy, ".6f")
        self._force_z_label.text = format(fz, ".6f")

    def _set_torque_labels(self, vec3):
        if not vec3:
            self._torque_x_label.text = "-"
            self._torque_y_label.text = "-"
            self._torque_z_label.text = "-"
            return
        tx, ty, tz = vec3
        self._torque_x_label.text = format(tx, ".6f")
        self._torque_y_label.text = format(ty, ".6f")
        self._torque_z_label.text = format(tz, ".6f")
