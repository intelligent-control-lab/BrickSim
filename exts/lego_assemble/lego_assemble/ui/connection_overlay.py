import traceback
import carb.settings
import omni.kit.app  # type: ignore
import omni.ui # type: ignore
import omni.usd # type: ignore
from dataclasses import dataclass
from omni.kit.viewport.registry import RegisterScene # type: ignore
from pxr import Gf, Usd, UsdGeom
from lego_assemble._native import compute_connection_local_transform, get_connection_utilization
from lego_assemble.utils.usd_parse import parse_connection_prim

SETTING_DISPLAY_CONNECTIONS = "/persistent/lego_assemble/visualizationDisplayConnections"

class _YieldToOtherGestures(omni.ui.scene.GestureManager):
    def __init__(self):
        super().__init__()
    def can_be_prevented(self, _):
        return True
    def should_prevent(self, _, preventer):
        if preventer.state == omni.ui.scene.GestureState.BEGAN or preventer.state == omni.ui.scene.GestureState.CHANGED:
            return True
        return False

@dataclass
class _VisualizedConnection:
    root: omni.ui.scene.Transform
    utilization_arc: omni.ui.scene.Arc
    highlight_arc: omni.ui.scene.Arc

class _ConnectionOverlayManipulator(omni.ui.scene.Manipulator):
    def __init__(self, usd_context):
        super().__init__()
        self._usd_context = usd_context
        self._settings = carb.settings.get_settings()
        self._gesture_manager = _YieldToOtherGestures()
        self._items: dict[str, _VisualizedConnection] = dict()
        self._xform_cache = UsdGeom.XformCache()
        self._stage_id: int | None = None
        self._panel: omni.ui.scene.Transform | None = None

    def _get_conn_world_pos(self, conn_path: str) -> tuple[float, float, float] | None:
        stage = self._usd_context.get_stage()
        if stage is None:
            return None
        parsed = parse_connection_prim(stage.GetPrimAtPath(conn_path))
        if parsed is None:
            return None
        stud_path_str, stud_if, hole_path_str, hole_if, offset, yaw = parsed
        mpu = UsdGeom.GetStageMetersPerUnit(stage)
        stud_prim = stage.GetPrimAtPath(stud_path_str)
        if not stud_prim.IsValid():
            return None
        try:
            (_, stud_pos_m), (_, _) = compute_connection_local_transform(stud_path=stud_path_str, stud_if=stud_if, hole_path=hole_path_str, hole_if=hole_if, offset=offset, yaw=yaw)
        except Exception:
            traceback.print_exc()
            return None
        T_W_stud = self._xform_cache.GetLocalToWorldTransform(stud_prim)
        p_W = T_W_stud.Transform(Gf.Vec3d(*stud_pos_m) / mpu)
        return float(p_W[0]), float(p_W[1]), float(p_W[2])

    def _get_conn_utilization_color(self, conn_path: str) -> str:
        try:
            u = get_connection_utilization(conn_path)
        except Exception:
            traceback.print_exc()
            u = None
        if u is None or u < 0.0:
            return "#808080FF"
        t = max(0.0, min(float(u), 1.0))
        r = int(255 * t)
        g = int(255 * (1.0 - t))
        return f"#{r:02X}{g:02X}00FF"

    def _is_selected(self, path: str) -> bool:
        return self._usd_context.get_selection().is_prim_path_selected(path)

    def on_selection_changed(self) -> None:
        for path, item in self._items.items():
            item.highlight_arc.visible = self._is_selected(path)

    def _create_item(self, path: str) -> _VisualizedConnection | None:
        pos = self._get_conn_world_pos(path)
        if pos is None:
            return None
        x, y, z = pos
        color_hex = self._get_conn_utilization_color(path)
        mouse_down_gesture = omni.ui.scene.DragGesture(
            mouse_button=0,
            on_began_fn=lambda _: self._usd_context.get_selection().set_selected_prim_paths([path], True),
            manager=self._gesture_manager,
        )
        root = omni.ui.scene.Transform(transform=omni.ui.scene.Matrix44.get_translation_matrix(x, y, z))
        with root:
            with omni.ui.scene.Transform(scale_to=omni.ui.scene.Space.SCREEN, look_at=omni.ui.scene.Transform.LookAt.CAMERA):
                utilization_arc = omni.ui.scene.Arc(radius=12.0, color=omni.ui.color(color_hex), wireframe=True, thickness=2.0)
                highlight_arc = omni.ui.scene.Arc(radius=15.0, color=omni.ui.color("#00ffff88"), wireframe=True, thickness=6.0, visible=self._is_selected(path))
                omni.ui.scene.Arc(radius=12.0, color=omni.ui.color("#00000000"), gesture=[mouse_down_gesture])
        return _VisualizedConnection(root, utilization_arc, highlight_arc)

    def update_overlay(self):
        if not self._settings.get_as_bool(SETTING_DISPLAY_CONNECTIONS):
            if self._panel is not None:
                self._panel.visible = False
            return
        if self._panel is None:
            self.invalidate()
            return
        self._panel.visible = True
        stage = self._usd_context.get_stage()
        if stage is None:
            self.invalidate()
            return
        if self._stage_id != self._usd_context.get_stage_id():
            self.invalidate()
            return
        self._xform_cache.Clear()
        visited_paths = set()
        for prim in stage.Traverse():
            if not prim.IsValid() or prim.GetTypeName() != "LegoConnection":
                continue
            path = str(prim.GetPath())
            item = self._items.get(path)
            if item is None:
                # New connection, need rebuild
                self.invalidate()
                return
            # Update existing item
            pos = self._get_conn_world_pos(path)
            if pos is None:
                continue
            x, y, z = pos
            item.root.transform = omni.ui.scene.Matrix44.get_translation_matrix(x, y, z)
            color_hex = self._get_conn_utilization_color(path)
            item.utilization_arc.color = omni.ui.color(color_hex)
            item.highlight_arc.visible = self._is_selected(path)
            visited_paths.add(path)
        if visited_paths != set(self._items.keys()):
            # Some connections were removed, need rebuild
            self.invalidate()
            return

    def on_build(self):
        if self._panel is not None:
            self.clear()
        self._panel = omni.ui.scene.Transform()
        self._items.clear()
        self._xform_cache.Clear()
        stage = self._usd_context.get_stage()
        if stage is None:
            self._stage_id = None
            return
        self._stage_id = self._usd_context.get_stage_id()
        for prim in stage.Traverse():
            if not prim.IsValid() or prim.GetTypeName() != "LegoConnection":
                continue
            path = str(prim.GetPath())
            with self._panel:
                item = self._create_item(path)
            if item is not None:
                self._items[path] = item

class ConnectionOverlayScene:
    def __init__(self, desc: dict):
        self.visible = True
        self.categories = ()
        self.name = "lego_assemble.connection_overlay"
        self._usd_context = omni.usd.get_context(desc.get("usd_context_name"))
        self._manipulator = _ConnectionOverlayManipulator(self._usd_context)
        self._update_sub =  omni.kit.app.get_app().get_update_event_stream().create_subscription_to_pop(self._on_update)
        self._selection_sub = self._usd_context.get_stage_event_stream().create_subscription_to_pop_by_type(omni.usd.StageEventType.SELECTION_CHANGED, self._on_selection_changed)

    def destroy(self):
        if self._update_sub is not None:
            self._update_sub.unsubscribe()
            self._update_sub = None
        if self._selection_sub is not None:
            self._selection_sub.unsubscribe()
            self._selection_sub = None
        self._manipulator = None

    def _on_update(self, _):
        if self._manipulator is not None:
            self._manipulator.update_overlay()

    def _on_selection_changed(self, _):
        if self._manipulator is not None:
            self._manipulator.on_selection_changed()

class ConnectionOverlayController:
    def __init__(self):
        self._viewport_overlay_registry = RegisterScene(ConnectionOverlayScene, "lego_assemble.connection_overlay")
        self._menubar_display_inst = None
        self._custom_item = None
        try:
            from omni.kit.viewport.menubar.core import CategoryStateItem # type: ignore
            from omni.kit.viewport.menubar.display import get_instance # type: ignore
            self._menubar_display_inst = get_instance()
            self._custom_item = CategoryStateItem("LEGO Connections", setting_path=SETTING_DISPLAY_CONNECTIONS)
            self._menubar_display_inst.register_custom_category_item("Show By Type", self._custom_item)
        except Exception:
            traceback.print_exc()
            self._menubar_display_inst = None
            self._custom_item = None

    def destroy(self):
        if self._menubar_display_inst is not None and self._custom_item is not None:
            self._menubar_display_inst.deregister_custom_category_item("Show By Type", self._custom_item)
        self._menubar_display_inst = None
        self._custom_item = None
        if self._viewport_overlay_registry is not None:
            self._viewport_overlay_registry.destroy()
            self._viewport_overlay_registry = None
