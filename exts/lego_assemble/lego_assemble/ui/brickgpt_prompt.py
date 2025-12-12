import carb
import json
import threading
import omni.kit.app  # type: ignore
import omni.ui as ui
from lego_assemble.ui.main_ui import LegoUI
from lego_assemble.importers.stabletext2brick import bricks_text_to_topology_json
from lego_assemble._native import import_lego, arrange_parts_in_workspace
from lego_assemble.utils.usd_parse import get_env_path

class BrickGPTPromptWindow:
    """Simple BrickGPT prompt window attached to the Console."""

    def __init__(self, main_ui: LegoUI):
        self._main_ui = main_ui
        self._window = ui.Window("BrickGPT", width=400, height=200)
        self._window.deferred_dock_in("Console")

        self._prompt_model = ui.SimpleStringModel("")
        self._status_label: ui.Label | None = None
        self._brickgpt = None
        self._update_sub = None
        self._pending_status: str | None = None
        self._pending_bricks_text: str | None = None
        self._generating: bool = False

        with self._window.frame:
            with ui.VStack(spacing=5, height=0):
                ui.Label("Prompt:", height=0)
                ui.StringField(self._prompt_model, height=0)
                ui.Button("Generate", clicked_fn=self._on_generate_clicked)
                self._status_label = ui.Label("", height=0)

        # Subscribe to Kit update events so we can safely
        # apply UI changes originating from the worker thread.
        app = omni.kit.app.get_app()
        stream = app.get_update_event_stream()
        self._update_sub = stream.create_subscription_to_pop(self._on_update)

    def destroy(self):
        if self._update_sub is not None:
            self._update_sub.unsubscribe()
            self._update_sub = None
        if self._window:
            self._window.destroy()
            self._window = None

    def _set_status(self, text: str) -> None:
        if self._status_label is not None:
            self._status_label.text = text

    def _on_update(self, _e) -> None:
        """Apply any pending status updates on the main thread."""
        if self._pending_status is not None:
            status = self._pending_status
            self._pending_status = None
            self._set_status(status)

        if self._pending_bricks_text is not None:
            bricks_text = self._pending_bricks_text
            self._pending_bricks_text = None
            self.do_import(bricks_text)

    def _ensure_model(self) -> None:
        """Lazily construct BrickGPT with stability analysis effectively disabled."""
        if self._brickgpt is not None:
            return

        try:
            from brickgpt.models import BrickGPT, BrickGPTConfig
        except Exception as exc:
            self._set_status(f"Failed to import BrickGPT: {exc}")
            raise

        # Disable stability-based regeneration and Gurobi usage:
        # just generate a structure once and return its brick text.
        cfg = BrickGPTConfig(
            max_regenerations=0,
            use_gurobi=False,
        )
        self._brickgpt = BrickGPT(cfg)

    def _on_generate_clicked(self):
        if self._generating:
            self._set_status("Already generating...")
            return

        prompt = (self._prompt_model.as_string or "").strip()
        if not prompt:
            self._set_status("Prompt is empty.")
            return

        self._generating = True
        self._set_status("Generating...")

        def _worker():
            try:
                self._ensure_model()
                result = self._brickgpt(prompt)
                bricks = result.get("bricks")

                if bricks is None:
                    self._pending_status = "No bricks generated."
                    return

                bricks_txt = bricks.to_txt()
                self._pending_bricks_text = bricks_txt
                self._pending_status = "Generated:\n" + bricks_txt
            except Exception as exc:
                self._pending_status = f"Generation failed: {exc}"
            finally:
                self._generating = False

        threading.Thread(target=_worker, daemon=True).start()

    def do_import(self, bricks_text: str):
        topology = bricks_text_to_topology_json(bricks_text, color=self._main_ui.get_selected_color())
        env_id = self._main_ui.get_env_id()
        part_paths, _ = import_lego(topology, env_id)
        imported_parts = [part_paths[k] for k in sorted(part_paths)]

        workspace_path = get_env_path(env_id) + "/LegoWorkspace"
        try:
            _, not_placed = arrange_parts_in_workspace(workspace_path, imported_parts, [1] * len(imported_parts))
        except Exception as exc:
            carb.log_warn(f"Failed to arrange parts in workspace: {exc}")
            self._set_status("Arrangement in workspace failed.")
            return
        if not_placed:
            carb.log_warn(f"Could not place parts {not_placed} in workspace {workspace_path}")
            self._set_status("Some parts could not be placed in workspace.")
            return
