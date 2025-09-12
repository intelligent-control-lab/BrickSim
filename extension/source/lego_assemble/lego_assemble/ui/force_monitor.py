import math
import omni.ui
import omni.usd
import omni.kit.app
import omni.timeline
from collections import deque
from pxr import Usd, UsdPhysics
from lego_assemble import _native

class ForceMonitor:
    """Builds the connection monitor UI (selection + live force/torque readout + plots)
    and manages update subscriptions. Intended to be instantiated inside an omni.ui container.
    """

    def __init__(self):
        # Selection state
        self._selected_joint_path: str | None = None
        self._conn_paths: list[str] = []

        # Plot buffers (per-axis): Fx/Fy/Fz and Tx/Ty/Tz
        self._plot_len = 512
        self._force_hist = [deque(maxlen=self._plot_len) for _ in range(3)]
        self._torque_hist = [deque(maxlen=self._plot_len) for _ in range(3)]
        self._plot_update_tick = 0

        # Subscriptions
        self._update_sub = None
        self._stage_event_sub = None

        # Build UI in current context
        self._build_ui()

        # Init selection list and start subscriptions
        self._refresh_connections()
        self._start_monitoring_updates()
        self._start_stage_selection_sync()

    # ---------- UI ----------
    def _build_ui(self):
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

        # Numeric readouts (Fx/Fy/Fz | Tx/Ty/Tz)
        with omni.ui.HStack(spacing=20, height=0):
            with omni.ui.VStack(spacing=4, height=0):
                with omni.ui.HStack():
                    omni.ui.Label("Fx:", width=16)
                    self._force_x_label = omni.ui.Label("-", width=110, alignment=omni.ui.Alignment.RIGHT_CENTER)
                with omni.ui.HStack():
                    omni.ui.Label("Fy:", width=16)
                    self._force_y_label = omni.ui.Label("-", width=110, alignment=omni.ui.Alignment.RIGHT_CENTER)
                with omni.ui.HStack():
                    omni.ui.Label("Fz:", width=16)
                    self._force_z_label = omni.ui.Label("-", width=110, alignment=omni.ui.Alignment.RIGHT_CENTER)

            with omni.ui.VStack(spacing=4, height=0):
                with omni.ui.HStack():
                    omni.ui.Label("Tx:", width=16)
                    self._torque_x_label = omni.ui.Label("-", width=110, alignment=omni.ui.Alignment.RIGHT_CENTER)
                with omni.ui.HStack():
                    omni.ui.Label("Ty:", width=16)
                    self._torque_y_label = omni.ui.Label("-", width=110, alignment=omni.ui.Alignment.RIGHT_CENTER)
                with omni.ui.HStack():
                    omni.ui.Label("Tz:", width=16)
                    self._torque_z_label = omni.ui.Label("-", width=110, alignment=omni.ui.Alignment.RIGHT_CENTER)

        # Two overlaid plots (Fx/Fy/Fz) and (Tx/Ty/Tz) in RGB
        def _build_vec3_plot(bufs):
            x_vals = list(bufs[0])
            y_vals = list(bufs[1])
            z_vals = list(bufs[2])
            height_px = 100
            all_vals = []
            if x_vals:
                all_vals += x_vals
            if y_vals:
                all_vals += y_vals
            if z_vals:
                all_vals += z_vals
            y_min, y_max = (-1.0, 1.0)
            if all_vals:
                vmin = min(all_vals)
                vmax = max(all_vals)
                if not math.isfinite(vmin) or not math.isfinite(vmax):
                    vmin, vmax = -1.0, 1.0
                if abs(vmax - vmin) < 1e-9:
                    pad = 1.0 if vmax == 0.0 else 0.1 * abs(vmax)
                    y_min, y_max = (vmin - pad, vmax + pad)
                else:
                    pad = 0.05 * (vmax - vmin)
                    y_min, y_max = (vmin - pad, vmax + pad)
            with omni.ui.ZStack():
                omni.ui.Rectangle(height=height_px)
                # X in red
                omni.ui.Plot(
                    omni.ui.Type.LINE,
                    y_min,
                    y_max,
                    *(x_vals if x_vals else [0.0]),
                    height=height_px,
                    style={"color": 0xFF3C4CE7, "background_color": 0x0},
                )
                # Y in green
                omni.ui.Plot(
                    omni.ui.Type.LINE,
                    y_min,
                    y_max,
                    *(y_vals if y_vals else [0.0]),
                    height=height_px,
                    style={"color": 0xFF71CC2E, "background_color": 0x0},
                )
                # Z in blue
                omni.ui.Plot(
                    omni.ui.Type.LINE,
                    y_min,
                    y_max,
                    *(z_vals if z_vals else [0.0]),
                    height=height_px,
                    style={"color": 0xFFDB9834, "background_color": 0x0},
                )

        omni.ui.Label("Force", height=0)
        self._force_plot_frame = omni.ui.Frame()
        self._force_plot_frame.set_build_fn(lambda: _build_vec3_plot(self._force_hist))
        omni.ui.Label("Torque", height=0)
        self._torque_plot_frame = omni.ui.Frame()
        self._torque_plot_frame.set_build_fn(lambda: _build_vec3_plot(self._torque_hist))

    # ---------- Subscriptions ----------
    def destroy(self):
        if self._update_sub is not None:
            self._update_sub.unsubscribe()
            self._update_sub = None
        if self._stage_event_sub is not None:
            self._stage_event_sub.unsubscribe()
            self._stage_event_sub = None

    def _start_monitoring_updates(self):
        app = omni.kit.app.get_app()
        stream = app.get_update_event_stream()
        self._update_sub = stream.create_subscription_to_pop(self._on_update)

    def _start_stage_selection_sync(self):
        ctx = omni.usd.get_context()
        stream = ctx.get_stage_event_stream()
        self._stage_event_sub = stream.create_subscription_to_pop_by_type(
            int(omni.usd.StageEventType.SELECTION_CHANGED), self._on_stage_selection_changed
        )

    # ---------- Behavior ----------
    def _list_lego_connections(self) -> list[str]:
        stage: Usd.Stage = omni.usd.get_context().get_stage()
        if stage is None:
            return []
        out: list[str] = []
        for prim in stage.Traverse():
            if not prim.IsValid() or not prim.IsA(UsdPhysics.FixedJoint):
                continue
            attr = prim.GetAttribute("lego_conn")
            if not attr or not attr.IsValid():
                continue
            try:
                if bool(attr.Get()):
                    out.append(str(prim.GetPath()))
            except Exception:
                continue
        return out

    def _refresh_connections(self):
        self._conn_paths = self._list_lego_connections()
        if self._selected_joint_path not in self._conn_paths:
            self._select_joint(None)
        if self._conn_combo_frame is not None:
            self._conn_combo_frame.rebuild()

    def _select_joint(self, path: str | None):
        self._selected_joint_path = path
        # Reset plots on selection change
        for d in self._force_hist:
            d.clear()
        for d in self._torque_hist:
            d.clear()

    def _on_conn_combo_changed(self, model, _val):
        idx = model.get_item_value_model().as_int
        path = None if idx == 0 else self._conn_paths[idx - 1]
        self._select_joint(path)

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
        # Pause plotting when simulation is not running
        if not omni.timeline.get_timeline_interface().is_playing():
            return
        # Append per-axis samples and rebuild occasionally
        self._force_hist[0].append(fx)
        self._force_hist[1].append(fy)
        self._force_hist[2].append(fz)
        self._torque_hist[0].append(tx)
        self._torque_hist[1].append(ty)
        self._torque_hist[2].append(tz)
        self._plot_update_tick = (self._plot_update_tick + 1) % 2
        if self._plot_update_tick == 0:
            if self._force_plot_frame is not None:
                self._force_plot_frame.rebuild()
            if self._torque_plot_frame is not None:
                self._torque_plot_frame.rebuild()

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

