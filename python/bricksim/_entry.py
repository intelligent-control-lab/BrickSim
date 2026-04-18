import sys


async def _open_default_stage():
    import carb
    from isaacsim.core.utils.stage import get_current_stage, open_stage_async

    from bricksim.assets import DEFAULT_STAGE_PATH

    success, error = await open_stage_async(str(DEFAULT_STAGE_PATH))
    if not success:
        carb.log_error(f"Failed to open default stage: {error}")
        return
    # Prevent the user from saving to the original stage file.
    stage = get_current_stage()
    stage.GetRootLayer().SetPermissionToSave(False)


def main():
    # Importing bricksim bootstraps the extension inside Kit.
    import bricksim  # noqa: F401

    args = sys.argv[1:]
    if args:
        from bricksim import kit_runner

        kit_runner.run(args[0], cli_args=args[1:])
    else:
        import omni.kit.async_engine as async_engine

        async_engine.run_coroutine(_open_default_stage())


if __name__ == "__main__":
    main()
