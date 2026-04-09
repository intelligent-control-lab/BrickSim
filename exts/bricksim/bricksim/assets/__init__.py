from pathlib import Path

ASSETS_DIR = Path(__file__).resolve().parent
STAGES_DIR = ASSETS_DIR / "stages"
ROBOTS_DIR = ASSETS_DIR / "robots"
DEFAULT_STAGE_PATH = STAGES_DIR / "default_stage.usda"
FRANKA_ROBOT_USD_PATH = ROBOTS_DIR / "franka" / "robot.usda"
