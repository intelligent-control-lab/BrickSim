"""Viewport overlay for visualizing BrickSim connection utilization."""

import traceback
from dataclasses import dataclass
from typing import NotRequired, TypedDict

import carb.events
import carb.settings
import omni.kit.app
import omni.ui
import omni.ui.scene
import omni.usd
from omni.kit.viewport.registry import RegisterScene
from omni.ui_scene import AbstractGesture
from pxr import Gf, UsdGeom

from bricksim.core import compute_connection_local_transform, get_connection_utilization
from bricksim.utils.connection_usd import parse_connection_prim

SETTING_DISPLAY_CONNECTIONS = "/persistent/bricksim/visualizationDisplayConnections"


class _ViewportSceneDesc(TypedDict):
    usd_context_name: str
    layer_provider: NotRequired[object]
    viewport_api: NotRequired[object]


class _YieldToOtherGestures(omni.ui.scene.GestureManager):
    def __init__(self):
        super().__init__()

    def can_be_prevented(self, arg0: AbstractGesture) -> bool:
        del arg0
        return True

    def should_prevent(
        self,
        arg0: AbstractGesture,
        arg1: AbstractGesture,
    ) -> bool:
        del arg0
        if (
            arg1.state == omni.ui.scene.GestureState.BEGAN
            or arg1.state == omni.ui.scene.GestureState.CHANGED
        ):
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
        mpu = UsdGeom.GetStageMetersPerUnit(stage)
        stud_prim = stage.GetPrimAtPath(parsed.stud_path)
        if not stud_prim.IsValid():
            return None
        try:
            (_, stud_pos_m), (_, _) = compute_connection_local_transform(
                stud_path=parsed.stud_path,
                stud_if=parsed.stud_interface,
                hole_path=parsed.hole_path,
                hole_if=parsed.hole_interface,
                offset=parsed.offset,
                yaw=parsed.yaw,
            )
        except Exception:
            traceback.print_exc()
            return None
        t_w_stud = self._xform_cache.GetLocalToWorldTransform(stud_prim)
        p_w = t_w_stud.Transform(Gf.Vec3d(*stud_pos_m) / mpu)
        return float(p_w[0]), float(p_w[1]), float(p_w[2])

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
            on_began_fn=lambda _: (
                self._usd_context.get_selection().set_selected_prim_paths([path], True)
            ),
            manager=self._gesture_manager,
        )
        root = omni.ui.scene.Transform(
            transform=omni.ui.scene.Matrix44.get_translation_matrix(x, y, z)
        )
        with root:
            with omni.ui.scene.Transform(
                scale_to=omni.ui.scene.Space.SCREEN,
                look_at=omni.ui.scene.Transform.LookAt.CAMERA,
            ):
                utilization_arc = omni.ui.scene.Arc(
                    radius=12.0,
                    color=omni.ui.color(color_hex),
                    wireframe=True,
                    thickness=2.0,
                )
                highlight_arc = omni.ui.scene.Arc(
                    radius=15.0,
                    color=omni.ui.color("#00ffff88"),
                    wireframe=True,
                    thickness=6.0,
                    visible=self._is_selected(path),
                )
                omni.ui.scene.Arc(
                    radius=12.0,
                    color=omni.ui.color("#00000000"),
                    gesture=[mouse_down_gesture],
                )
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
    """Viewport scene registered for BrickSim connection overlays."""

    _manipulator: _ConnectionOverlayManipulator | None
    _update_sub: carb.events.ISubscription | None
    _selection_sub: carb.events.ISubscription | None

    def __init__(self, desc: _ViewportSceneDesc):
        """Create the overlay scene for one USD context."""
        self.visible = True
        self.categories = ()
        self.name = "bricksim.connection_overlay"
        self._usd_context = omni.usd.get_context(desc["usd_context_name"])
        self._manipulator = _ConnectionOverlayManipulator(self._usd_context)
        self._update_sub = (
            omni.kit.app.get_app()
            .get_update_event_stream()
            .create_subscription_to_pop(self._on_update)
        )
        stage_event_stream = self._usd_context.get_stage_event_stream()
        self._selection_sub = (
            stage_event_stream.create_subscription_to_pop_by_type(
                int(omni.usd.StageEventType.SELECTION_CHANGED),
                self._on_selection_changed,
            )
        )

    def destroy(self) -> None:
        """Unsubscribe from viewport events and release the manipulator."""
        if self._update_sub is not None:
            self._update_sub.unsubscribe()
            self._update_sub = None
        if self._selection_sub is not None:
            self._selection_sub.unsubscribe()
            self._selection_sub = None
        self._manipulator = None

    def _on_update(self, _event: carb.events.IEvent) -> None:
        if self._manipulator is not None:
            self._manipulator.update_overlay()

    def _on_selection_changed(self, _event: carb.events.IEvent) -> None:
        if self._manipulator is not None:
            self._manipulator.on_selection_changed()


class ConnectionOverlayController:
    """Register the connection overlay and menubar display toggle."""

    def __init__(self):
        """Initialize viewport overlay registration and menu integration."""
        self._viewport_overlay_registry = RegisterScene(
            ConnectionOverlayScene, "bricksim.connection_overlay"
        )
        self._menubar_display_inst = None
        self._custom_item = None
        try:
            from omni.kit.viewport.menubar.core import CategoryStateItem
            from omni.kit.viewport.menubar.display import get_instance

            self._menubar_display_inst = get_instance()
            self._custom_item = CategoryStateItem(
                "LEGO Connections", setting_path=SETTING_DISPLAY_CONNECTIONS
            )
            if self._menubar_display_inst is not None:
                self._menubar_display_inst.register_custom_category_item(
                    "Show By Type", self._custom_item
                )
        except Exception:
            traceback.print_exc()
            self._menubar_display_inst = None
            self._custom_item = None

    def destroy(self):
        """Remove the viewport overlay and menu integration."""
        if self._menubar_display_inst is not None and self._custom_item is not None:
            self._menubar_display_inst.deregister_custom_category_item(
                "Show By Type", self._custom_item
            )
        self._menubar_display_inst = None
        self._custom_item = None
        if self._viewport_overlay_registry is not None:
            self._viewport_overlay_registry.destroy()
            self._viewport_overlay_registry = None
