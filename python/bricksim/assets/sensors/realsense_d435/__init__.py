"""RealSense D435 asset paths and camera calibration defaults."""

from pathlib import Path

D435_USD_PATH = Path(__file__).resolve().parent / "d435.usd"

# Nominal D435 color intrinsics at 1280x720.
# Storage: row-major 3x3 pinhole matrix.
D435_DEFAULT_COLOR_INTRINSICS_1280_720 = [
    924.277380,
    0.0,
    640.0,
    0.0,
    925.738464,
    360.0,
    0.0,
    0.0,
    1.0,
]
