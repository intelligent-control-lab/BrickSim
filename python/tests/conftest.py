"""Shared pytest support."""

from isaaclab.app import AppLauncher

isaacsim_app = AppLauncher({"headless": True}).app

import bricksim  # noqa: E402, F401
