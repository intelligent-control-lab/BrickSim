import math
import logging
import carb.events
import omni.kit.app
import omni.usd
import omni.physx
import omni.physx.scripts.physicsUtils as physicsUtils
from typing import Tuple, Optional, Literal
from pxr import Gf, Usd, UsdGeom
from . import brick_generator, brick_assembler_simple, brick_assembler_vectorized
from .brick_assembler import AssemblyEvent
from .utils import get_physics_scene

_logger = logging.getLogger(__name__)

class BrickPhysicsInterface:
    def __init__(self, mode: Literal["simple", "cpu_vectorized"] = "cpu_vectorized"):
        self.mode = mode
        self.needs_reload = False

        update_bus: carb.events.IEventStream = omni.kit.app.get_app().get_update_event_stream()
        self.update_sub = update_bus.create_subscription_to_push(self._on_update)
        self.vectorized_detector: Optional[brick_assembler_vectorized.VectorizedAssemblyDetector] = None
        self.accumulated_assembly_events = []

    def force_reload(self):
        self.needs_reload = True

    def create_brick(self, dimensions: Tuple[int, int, int], color_name: str, env_id: Optional[int] = None, pos: Gf.Vec3f = None, quat: Gf.Quatf = None) -> UsdGeom.Xform:
        stage: Usd.Stage = omni.usd.get_context().get_stage()
        path = self._next_brick_path(stage, env_id)
        brick = brick_generator.create_brick(stage, path, dimensions, color_name)
        if pos is not None:
            physicsUtils.set_or_add_translate_op(brick, pos)
        if quat is not None:
            physicsUtils.set_or_add_orient_op(brick, quat)
        self.force_reload()
        _logger.info(f"Added brick {path} ({dimensions[0]}x{dimensions[1]}x{dimensions[2]}) {color_name}")
        return brick

    def poll_assembly_events(self) -> list[AssemblyEvent]:
        acc = self.accumulated_assembly_events
        self.accumulated_assembly_events = []
        return acc

    def reset_env(self, env_id: Optional[int] = None):
        raise NotImplementedError("reset_env is not implemented") # TODO

    def _next_brick_path(self, stage: Usd.Stage, env_id: Optional[int] = None) -> str:
        prefix = f"/World/envs/env_{env_id}/Brick_" if env_id is not None else "/World/Brick_"
        unquifier = 0
        while stage.GetPrimAtPath(f"{prefix}{unquifier}").IsValid():
            unquifier += 1
        return f"{prefix}{unquifier}"

    def _on_update(self, _event: carb.events.IEvent):
        if not omni.physx.get_physx_interface().is_running():
            return

        if self.mode == "simple":
            if self.needs_reload:
                self.needs_reload = False
            assembly_events = brick_assembler_simple.handle_assembly_contacts()

        elif self.mode == "cpu_vectorized":
            if self.needs_reload or (self.vectorized_detector is None) or (not self.vectorized_detector.check()):
                current_stage: Usd.Stage = omni.usd.get_context().get_stage()
                if (current_stage is None) or (get_physics_scene(current_stage) is None):
                    # Not ready to initialize now
                    return
                omni.physx.get_physx_interface().force_load_physics_from_usd()
                self.vectorized_detector = brick_assembler_vectorized.VectorizedAssemblyDetector()
                self.needs_reload = False
                _logger.info("Brick assembly detector reloaded")
            assembly_events = self.vectorized_detector.handle_assembly_contacts()

        else:
            raise RuntimeError(f"Unknown mode: {self.mode}")

        self.accumulated_assembly_events.extend(assembly_events)

        for event in assembly_events:
            _logger.info(
                f"Brick assembly: {event.brick0} and {event.brick1}, "
                f"joint={event.joint}, "
                f"p_0={event.p0}, "
                f"p_1={event.p1}, "
                f"yaw={math.degrees(event.yaw)}"
            )

    def _destroy(self):
        self.update_sub.unsubscribe()

_brick_physics_interface = None

def get_brick_physics_interface() -> BrickPhysicsInterface:
    """Get the BrickPhysicsInterface instance."""
    global _brick_physics_interface
    if _brick_physics_interface is None:
        raise RuntimeError("BrickPhysicsInterface is not initialized.")
    return _brick_physics_interface

def init_brick_physics_interface():
    global _brick_physics_interface
    if _brick_physics_interface is not None:
        raise RuntimeError("BrickPhysicsInterface is already initialized.")
    _brick_physics_interface = BrickPhysicsInterface()
    return _brick_physics_interface

def deinit_brick_physics_interface():
    global _brick_physics_interface
    if _brick_physics_interface is None:
        raise RuntimeError("BrickPhysicsInterface is not initialized.")
    _brick_physics_interface._destroy()
    _brick_physics_interface = None
