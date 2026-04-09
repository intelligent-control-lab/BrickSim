import asyncio
from typing import Optional, Tuple

from omni.kit.window.filepicker import FilePickerDialog


FilePickerResult = Tuple[bool, Optional[str], Optional[str]]


async def show_file_picker_dialog(
    title: str,
    *,
    apply_button_label: str,
    file_extension: Optional[str] = None,
    default_filename: Optional[str] = None,
) -> FilePickerResult:
    """
    Show a FilePickerDialog and await its completion.

    The returned tuple is (success, filename, dirname). ``success`` is True
    when the user clicks the apply button, False on cancel. ``filename`` and
    ``dirname`` are None when the dialog is cancelled.

    The coroutine always completes when the dialog is closed via apply or
    cancel; it will not hang indefinitely on user cancellation.
    """
    loop = asyncio.get_event_loop()
    future: "asyncio.Future[FilePickerResult]" = loop.create_future()

    def _resolve(success: bool, filename: Optional[str], dirname: Optional[str]) -> None:
        if future.done():
            return
        future.set_result((success, filename, dirname))

    def _on_apply(filename: str, dirname: str) -> None:
        _resolve(True, filename, dirname)

    def _on_cancel(filename: str, dirname: str) -> None:  # dirname is unused here but kept for signature compatibility
        _resolve(False, None, None)

    dialog = FilePickerDialog(
        title,
        allow_multi_selection=False,
        apply_button_label=apply_button_label,
        click_apply_handler=_on_apply,
        click_cancel_handler=_on_cancel,
    )

    if file_extension is not None:
        dialog.set_file_extension(file_extension)

    if default_filename is not None:
        dialog.set_filename(default_filename)

    dialog.show()

    try:
        return await future
    finally:
        # Ensure the dialog is hidden even if the awaiter is cancelled.
        try:
            dialog.hide()
        except Exception:
            # Dialog may already be destroyed/hidden; ignore.
            pass
