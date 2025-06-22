import math
import logging
import omni.usd
import omni.physx
import omni.physx.scripts.physicsUtils as physicsUtils
from typing import Tuple, Optional
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics
from omni.physx.bindings._physx import ContactEventHeaderVector, ContactDataVector
from . import spawner
from .assembler_impl import VectorizedAssemblyDetector, BrickTracker
from .assembler import AssemblyEvent, path_for_brick
from .utils import get_physics_scene, add_to_collision_group

_logger = logging.getLogger(__name__)

class BrickPhysicsInterface:
    def __init__(self):
        self.update_sub = omni.physx.get_physx_simulation_interface().subscribe_contact_report_events(self._on_contact_report)
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
        stage: Usd.Stage = omni.usd.get_context().get_stage()

        while True:
            brick_id = self.uniqueifier
            self.uniqueifier += 1
            path = path_for_brick(brick_id=brick_id, env_id=env_id)
            if not stage.GetPrimAtPath(path).IsValid():
                break

        brick = spawner.create_brick(stage, path, dimensions, color_name)
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
        self._ensure_vectorized_detector()

        if self.tracker is None:
            self.tracker = BrickTracker(num_envs, num_trackings)
            self.tracker.set_backend(self.vectorized_detector)
        else:
            if (self.tracker.num_envs, self.tracker.num_trackings) != (num_envs, num_trackings):
                raise RuntimeError(f"BrickTracker has been initialized with a different config ({self.tracker.tracked_brick_ids.shape})")

        return self.tracker

    def _ensure_vectorized_detector(self) -> Optional[VectorizedAssemblyDetector]:
        if (self.vectorized_detector is None) or (not self.vectorized_detector.check()):
            current_stage: Usd.Stage = omni.usd.get_context().get_stage()
            if (current_stage is None) or (get_physics_scene(current_stage) is None):
                # Not ready to initialize now
                return None
            omni.physx.get_physx_interface().force_load_physics_from_usd()
            self.vectorized_detector = VectorizedAssemblyDetector()
            if self.tracker is not None:
                self.tracker.set_backend(self.vectorized_detector)
            _logger.info(f"Brick assembly detector reloaded")

    def _on_contact_report(self, contacts: ContactEventHeaderVector, contact_data: ContactDataVector):
        if not omni.physx.get_physx_interface().is_running():
            return

        self._ensure_vectorized_detector()
        if self.vectorized_detector is None:
            return

        assembly_events = self.vectorized_detector.handle_assembly_contacts(contacts, contact_data)
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
