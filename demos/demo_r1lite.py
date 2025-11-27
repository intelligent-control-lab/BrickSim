import os
from omni.kit.async_engine import run_coroutine
from isaacsim.core.utils.stage import open_stage_async

async def main():
    stage_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "../resources/r1lite_demo.usda")
    await open_stage_async(stage_path)
    print("R1Lite demo stage loaded.")

run_coroutine(main())
