import json
from pathlib import Path
from typing import Callable, List, Tuple

import carb
import omni.ui as ui

from lego_assemble._native import import_lego
from lego_assemble.importers.stabletext2brick import bricks_text_to_topology_json

Example = Tuple[str, str, int]  # (uuid, caption, dataset_index)


class StableText2BrickBrowser:
    """Browser UI for the StableText2Brick dataset."""

    MAX_VISIBLE = 100

    def __init__(self, env_id_provider: Callable[[], int] | None = None) -> None:
        self._window = ui.Window("StableText2Brick", width=500, height=600)
        self._window.deferred_dock_in("Console")

        # Index state
        self._index_loaded: bool = False
        self._examples: List[Example] = []

        # UI / selection / filter state
        self._selected_ds_index: int | None = None
        self._filter_text: str = ""

        self._search_model = ui.SimpleStringModel("")
        self._status_model = ui.SimpleStringModel(
            "Loading StableText2Brick index (resources/stabletext2brick/index.json) ..."
        )
        self._status_label: ui.Label | None = None
        self._list_frame: ui.Frame | None = None

        # Env id provider from main LEGO UI
        self._get_env_id = env_id_provider

        # When the search text changes, just recompute + rebuild list.
        def _on_search_changed(model: ui.AbstractValueModel) -> None:
            self._filter_text = model.as_string or ""
            self._rebuild_list()

        self._search_model.add_value_changed_fn(_on_search_changed)

        # ----- Static window layout -----
        with self._window.frame:
            with ui.VStack(spacing=2):
                # Search row
                with ui.HStack(spacing=5, height=0):
                    ui.Label("Search (id / caption):", width=120, height=0)
                    ui.StringField(self._search_model, height=0)

                # Status line
                self._status_label = ui.Label(self._status_model.as_string, height=0)

                # Scroll area + list frame
                with ui.ScrollingFrame(
                    horizontal_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_ALWAYS_OFF,
                    vertical_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_AS_NEEDED,
                ):
                    with ui.VStack(spacing=2):
                        # This is now just a container — no build_fn on it.
                        self._list_frame = ui.Frame()

        # Initial population
        self._rebuild_list()

    # ------------------------------------------------------------------ lifecycle

    def destroy(self) -> None:
        if self._window:
            self._window.destroy()
            self._window = None

    # --------------------------------------------------------------------- helpers

    def _set_status(self, msg: str) -> None:
        self._status_model.set_value(msg)
        if self._status_label is not None:
            self._status_label.text = msg

    # ------------ filesystem helpers ------------

    def _repo_root(self) -> Path | None:
        try:
            return Path(__file__).resolve().parents[4]
        except Exception as exc:
            carb.log_error(
                f"[StableText2Brick] failed to resolve repo root from __file__: {exc}"
            )
            return None

    def _index_path(self) -> Path | None:
        root = self._repo_root()
        if root is None:
            return None
        return root / "resources" / "stabletext2brick" / "index.json"

    def _structures_dir(self) -> Path | None:
        root = self._repo_root()
        if root is None:
            return None
        return root / "resources" / "stabletext2brick" / "structures"

    # ------------ dataset loading ------------

    def _ensure_index_loaded(self) -> None:
        """Lazy-load index.json once."""
        if self._index_loaded:
            return

        carb.log_info("[StableText2Brick] loading local index.json")
        self._index_loaded = True

        index_path = self._index_path()
        if index_path is None:
            self._examples = []
            self._set_status(
                "StableText2Brick: failed to resolve repo root from __file__."
            )
            return

        if not index_path.is_file():
            msg = (
                "Missing index. Run scripts/export_stabletext2brick_index.py "
                "resources/stabletext2brick/ to build index."
            )
            carb.log_error(msg)
            self._examples = []
            self._set_status(msg)
            return

        try:
            with index_path.open("r", encoding="utf-8") as f:
                data = json.load(f)
        except Exception as exc:
            carb.log_error(f"[StableText2Brick] failed to read index.json: {exc}")
            self._examples = []
            self._set_status("StableText2Brick: failed to read index.json.")
            return

        examples: List[Example] = []
        for ds_idx, entry in enumerate(data):
            try:
                uuid = str(entry["uuid"])
                caption = str(entry.get("caption", ""))
                examples.append((uuid, caption, ds_idx))
            except Exception:
                # Skip malformed entries
                continue

        self._examples = examples
        total = len(self._examples)
        shown = min(total, self.MAX_VISIBLE)
        self._set_status(f"{total} loaded, {total} matches, showing {shown}")
        carb.log_info(
            f"[StableText2Brick] index loaded: total_examples={len(self._examples)}"
        )

    # ------------ filtering ------------

    def _filtered_examples(self) -> List[Example]:
        text = (self._filter_text or "").strip().lower()
        if not self._examples:
            return []

        if not text:
            return self._examples

        visible: List[Example] = []
        for uuid, caption, ds_idx in self._examples:
            if text in uuid.lower() or text in caption.lower():
                visible.append((uuid, caption, ds_idx))
        return visible

    # ------------ list rebuild ------------

    def _rebuild_list(self) -> None:
        """Clear and rebuild the list frame manually (no set_build_fn)."""
        if self._list_frame is None:
            return

        self._ensure_index_loaded()

        # Remove previous children
        self._list_frame.clear()

        # Build fresh contents
        with self._list_frame:
            self._build_list_contents()

    def _build_list_contents(self) -> None:
        carb.log_info(
            f"[StableText2Brick] _build_list_contents; "
            f"examples={len(self._examples)}, filter={self._filter_text!r}"
        )

        with ui.VStack(spacing=2):
            if not self._examples:
                # Index failed to load or is empty. Message already in status line.
                ui.Label("No index loaded. See status line for details.", height=0)
                return

            visible_all = self._filtered_examples()
            total_loaded = len(self._examples)
            matches = len(visible_all)

            visible = visible_all[: self.MAX_VISIBLE]
            shown = len(visible)

            self._set_status(
                f"{total_loaded} loaded, {matches} matches, showing {shown}"
            )

            if not visible:
                ui.Label("No matches.", height=0)
                return

            for uuid, caption, ds_idx in visible:
                selected = self._selected_ds_index == ds_idx
                bg_color = 0x40808080 if selected else 0x00000000

                row = ui.ZStack(height=22)
                with row:
                    ui.Rectangle(style={"background_color": bg_color})
                    with ui.HStack(spacing=8, height=0):
                        ui.Spacer(width=5)
                        ui.Label(uuid, width=260, height=0, ellipsize=True)
                        ui.Label(caption, height=0, ellipsize=True)

                # Single-click: select
                def _on_single_click(x, y, button, modifiers, ds_idx=ds_idx):
                    if button != 0:
                        return
                    self._selected_ds_index = ds_idx
                    self._rebuild_list()

                # Double-click: import
                def _on_double_click(x, y, button, modifiers, ds_idx=ds_idx):
                    if button != 0:
                        return
                    self._selected_ds_index = ds_idx
                    self._on_import_clicked()

                row.set_mouse_pressed_fn(_on_single_click)
                row.set_mouse_double_clicked_fn(_on_double_click)

    # ----------------------------------------------------------------- actions

    def _resolve_selected_example(self) -> Example | None:
        if self._selected_ds_index is None:
            return None
        for uuid, caption, ds_idx in self._examples:
            if ds_idx == self._selected_ds_index:
                return (uuid, caption, ds_idx)
        return None

    def _on_import_clicked(self) -> None:
        self._ensure_index_loaded()

        example = self._resolve_selected_example()
        if example is None:
            carb.log_error("[StableText2Brick] import requested with no selection")
            self._set_status("No selection to import.")
            return

        uuid, caption, _ = example

        # Locate bricks text
        structures_dir = self._structures_dir()
        if structures_dir is None:
            self._set_status("StableText2Brick: failed to resolve structures directory.")
            return

        bricks_path = structures_dir / f"{uuid}.txt"
        if not bricks_path.is_file():
            carb.log_error(
                f"[StableText2Brick] structure file not found: {bricks_path}"
            )
            self._set_status("Structure file not found. See console.")
            return

        try:
            bricks_text = bricks_path.read_text(encoding="utf-8")
        except Exception as exc:
            carb.log_error(
                f"[StableText2Brick] failed to read structure file {bricks_path}: {exc}"
            )
            self._set_status("Error reading structure file. See console.")
            return

        # Convert text -> topology
        try:
            topology = bricks_text_to_topology_json(bricks_text)
        except Exception as exc:
            carb.log_error(f"[StableText2Brick] conversion error: {exc}")
            self._set_status("Conversion error. See console.")
            return

        try:
            topology["_meta"] = {
                "structure_id": uuid,
                "caption": caption,
            }
        except Exception:
            pass

        topology_str = json.dumps(topology)

        # Resolve env id
        env_id = -1
        if self._get_env_id is not None:
            try:
                env_id = int(self._get_env_id())
            except Exception:
                env_id = -1

        # Import into world
        try:
            import_lego(
                topology_str,
                env_id,
                (1.0, 0.0, 0.0, 0.0),  # wxyz
                (0.0, 0.0, 0.0),
            )
        except Exception as exc:
            carb.log_error(f"[StableText2Brick] import_lego failed: {exc}")
            self._set_status("Import failed. See console.")
            return

        carb.log_info(
            f"[StableText2Brick] Imported structure {uuid} into env_id={env_id}"
        )
        self._set_status("Import completed.")
