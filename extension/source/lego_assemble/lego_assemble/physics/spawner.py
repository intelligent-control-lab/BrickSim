from lego_assemble.physics import lego_schemes
from lego_assemble._native import create_brick as native_create_brick

def create_brick(path: str, dimensions: tuple[int, int, int], color_name: str) -> None:
    native_create_brick(path, dimensions, lego_schemes.parse_color(color_name))
