import math
import logging
import carb.events
import omni.kit.app
import omni.usd
import omni.physx
import omni.physx.scripts.physicsUtils as physicsUtils
from typing import Tuple, Optional, Literal
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics
from . import brick_spawner, brick_assembler_simple
from .brick_assembler_vectorized import VectorizedAssemblyDetector, BrickTracker
from .brick_assembler import AssemblyEvent, path_for_brick
from .utils import get_physics_scene, add_to_collision_group

_logger = logging.getLogger(__name__)

class BrickPhysicsInterface:
    def __init__(self, mode: Literal["simple", "cpu_vectorized"] = "cpu_vectorized"):
        self.mode = mode

        update_bus: carb.events.IEventStream = omni.kit.app.get_app().get_update_event_stream()
        self.update_sub = update_bus.create_subscription_to_push(self._on_update)
        self.vectorized_detector: Optional[VectorizedAssemblyDetector] = None
        self.tracker: Optional[BrickTracker] = None
        self.accumulated_assembly_events = []
        self.uniqueifier = 0 # Global monotonically-increasing brick id

    def invalidate(self):
        if self.vectorized_detector is not None:
            self.vectorized_detector.destroy()
            self.vectorized_detector = None

    def create_brick(self, dimensions: Tuple[int, int, int], color_name: str, env_id: Optional[int] = None, pos: Gf.Vec3f = None, quat: Gf.Quatf = None) -> Tuple[UsdGeom.Xform, int]:
        self.invalidate()

        brick_id = self.uniqueifier
        self.uniqueifier += 1
        path = path_for_brick(brick_id=brick_id, env_id=env_id)

        stage: Usd.Stage = omni.usd.get_context().get_stage()
        brick = brick_spawner.create_brick(stage, path, dimensions, color_name)
        if env_id is not None:
            add_to_collision_group(stage, env_id, brick.GetPath())
        if pos is not None:
            physicsUtils.set_or_add_translate_op(brick, pos)
        if quat is not None:
            physicsUtils.set_or_add_orient_op(brick, quat)
        _logger.info(f"Added brick {path} ({dimensions[0]}x{dimensions[1]}x{dimensions[2]}) {color_name}")
        return brick, brick_id

    def poll_assembly_events(self) -> list[AssemblyEvent]:
        acc = self.accumulated_assembly_events
        self.accumulated_assembly_events = []
        return acc

    def reset_env(self, env_id: Optional[int] = None):
        self.invalidate()
        stage: Usd.Stage = omni.usd.get_context().get_stage()

        root_path = f"/World/envs/env_{env_id}" if env_id is not None else "/World"
        root_prim = stage.GetPrimAtPath(root_path)
        if not root_prim.IsValid():
            _logger.warning(f"Resetting bricks in env {env_id} but it does not exist")
            return
        root_path_sdf: Sdf.Path = root_prim.GetPath()

        collision_group_prim = stage.GetPrimAtPath(f"/World/collisions/group{env_id}")
        if collision_group_prim.IsValid():
            collision_group = UsdPhysics.CollisionGroup(collision_group_prim)
            collider_collection: Usd.CollectionAPI = collision_group.GetCollidersCollectionAPI()
            collision_rel = collider_collection.GetIncludesRel()
        else:
            collision_rel = None

        for child in root_prim.GetAllChildrenNames():
            if child.startswith(("Brick_", "Conn_")):
                path = root_path_sdf.AppendChild(child)
                stage.RemovePrim(path)
                if collision_rel is not None:
                    collision_rel.RemoveTarget(path)

        _logger.info(f"Resetting bricks in env {env_id}")

    def get_tracker(self, num_envs: int, num_trackings: int) -> BrickTracker:
        if self.mode != "cpu_vectorized":
            raise RuntimeError("BrickTracker is only available in cpu_vectorized mode")

        self._ensure_vectorized_detector()

        if self.tracker is None:
            self.tracker = BrickTracker(num_envs, num_trackings)
            self.tracker.set_backend(self.vectorized_detector)
        else:
            if (self.tracker.num_envs, self.tracker.num_trackings) != (num_envs, num_trackings):
                raise RuntimeError(f"BrickTracker has been initialized with a different config ({self.tracker.tracked_brick_ids.shape})")

        return self.tracker

    def _ensure_vectorized_detector(self) -> Optional[VectorizedAssemblyDetector]:
        if self.mode != "cpu_vectorized":
            raise RuntimeError("Vectorized assembly detector is only available in cpu_vectorized mode")
        if (self.vectorized_detector is None) or (not self.vectorized_detector.check()):
            current_stage: Usd.Stage = omni.usd.get_context().get_stage()
            if (current_stage is None) or (get_physics_scene(current_stage) is None):
                # Not ready to initialize now
                return None
            omni.physx.get_physx_interface().force_load_physics_from_usd()
            self.vectorized_detector = VectorizedAssemblyDetector()
            if self.tracker is not None:
                self.tracker.set_backend(self.vectorized_detector)
            _logger.info("Brick assembly detector reloaded")

    def _on_update(self, _event: carb.events.IEvent):
        if not omni.physx.get_physx_interface().is_running():
            return

        if self.mode == "simple":
            assembly_events = brick_assembler_simple.handle_assembly_contacts()

        elif self.mode == "cpu_vectorized":
            self._ensure_vectorized_detector()
            if self.vectorized_detector is None:
                return
            assembly_events = self.vectorized_detector.handle_assembly_contacts()

        else:
            raise RuntimeError(f"Unknown mode: {self.mode}")

        self.accumulated_assembly_events.extend(assembly_events)

        for event in assembly_events:
            _logger.info(
                f"Brick assembly: {event.brick0} and {event.brick1}, "
                f"env={event.env_id}, "
                f"p0={event.p0}, "
                f"p1={event.p1}, "
                f"yaw={math.degrees(event.yaw)}"
            )

    def _destroy(self):
        self.update_sub.unsubscribe()
        if self.vectorized_detector is not None:
            self.vectorized_detector.destroy()

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
