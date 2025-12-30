from .colors import *

# Keep `import lego_assemble` usable in non-Isaac environments (e.g., offline analysis
# of debug dumps) by only importing Isaac/Omniverse-dependent modules when `carb`
# is available.
try:
    import carb as _carb  # type: ignore
except ModuleNotFoundError:
    _carb = None

if _carb is not None:
    from ._native import *
    from .extension import *
    from .envs import *
    from .utils.usd_parse import *
    from .utils.sim import *

del _carb
