"""Asset path constants and packaged resources."""

from pathlib import Path as _Path

ASSETS_DIR = _Path(__file__).resolve().parent
STAGES_DIR = ASSETS_DIR / "stages"
ROBOTS_DIR = ASSETS_DIR / "robots"
DEFAULT_STAGE_PATH = STAGES_DIR / "default_stage.usda"
FR3_ROBOT_USD_PATH = ROBOTS_DIR / "fr3" / "robot.usda"
