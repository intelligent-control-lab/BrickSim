import math

from bricksim._native import AssemblyThresholds, BreakageThresholds, set_assembly_thresholds, set_breakage_thresholds


def configure_assembly_thresholds(
    enabled: bool = True,
    distance_tolerance: float = 0.001,
    max_penetration: float = 0.005,
    z_angle_tolerance: float = 5.0 / 180.0 * math.pi,
    required_force: float = 1.0,
    yaw_tolerance: float = 5.0 / 180.0 * math.pi,
    position_tolerance: float = 0.002,
) -> None:
    """Configure global native assembly-detection thresholds for the active process.

    These thresholds map directly to the native ``AssemblyThresholds`` struct.
    They are process-global detector settings, not environment-specific Isaac
    Lab configuration.

    Args:
        enabled: Whether native assembly detection is enabled.
        distance_tolerance: Maximum stud/hole separation in meters.
        max_penetration: Maximum tolerated penetration in meters.
        z_angle_tolerance: Maximum stud/hole tilt mismatch in radians.
        required_force: Minimum compressive force magnitude in newtons.
        yaw_tolerance: Maximum snapped yaw error in radians.
        position_tolerance: Maximum planar position error in meters.
    """
    thresholds = AssemblyThresholds()
    thresholds.enabled = enabled
    thresholds.distance_tolerance = distance_tolerance
    thresholds.max_penetration = max_penetration
    thresholds.z_angle_tolerance = z_angle_tolerance
    thresholds.required_force = required_force
    thresholds.yaw_tolerance = yaw_tolerance
    thresholds.position_tolerance = position_tolerance
    set_assembly_thresholds(thresholds)


def configure_breakage_thresholds(
    enabled: bool = True,
    contact_regularization: float = 0.1,
    clutch_axial_compliance: float = 1.0,
    clutch_radial_compliance: float = 1.0,
    clutch_tangential_compliance: float = 1.0,
    friction_coefficient: float = 0.2,
    preloaded_force: float = 3.5,
    slack_fraction_warn: float = 0.1,
    slack_fraction_b_floor: float = 1e-9,
    debug_dump: bool = False,
    breakage_cooldown_time: float = 0.05,
) -> None:
    """Configure global native breakage-detection thresholds for the active process.

    These thresholds map directly to the native ``BreakageThresholds`` struct.
    They are process-global detector settings, not environment-specific Isaac
    Lab configuration.

    Args:
        enabled: Whether native breakage detection is enabled.
        contact_regularization: Contact regularization scalar.
        clutch_axial_compliance: Axial clutch compliance scalar.
        clutch_radial_compliance: Radial clutch compliance scalar.
        clutch_tangential_compliance: Tangential clutch compliance scalar.
        friction_coefficient: Effective clutch friction coefficient.
        preloaded_force: Nominal clutch preload in newtons.
        slack_fraction_warn: Warning threshold for solver slack fraction.
        slack_fraction_b_floor: Lower bound used in slack normalization.
        debug_dump: Whether to emit native breakage debug dumps.
        breakage_cooldown_time: Cooldown after disassembly events in seconds.
    """
    thresholds = BreakageThresholds()
    thresholds.enabled = enabled
    thresholds.contact_regularization = contact_regularization
    thresholds.clutch_axial_compliance = clutch_axial_compliance
    thresholds.clutch_radial_compliance = clutch_radial_compliance
    thresholds.clutch_tangential_compliance = clutch_tangential_compliance
    thresholds.friction_coefficient = friction_coefficient
    thresholds.preloaded_force = preloaded_force
    thresholds.slack_fraction_warn = slack_fraction_warn
    thresholds.slack_fraction_b_floor = slack_fraction_b_floor
    thresholds.debug_dump = debug_dump
    thresholds.breakage_cooldown_time = breakage_cooldown_time
    set_breakage_thresholds(thresholds)
