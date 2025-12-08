import json
from pathlib import Path

import carb
import omni.ui as ui

from lego_assemble._native import import_lego
from lego_assemble.importers.stabletext2brick import bricks_text_to_topology_json
from lego_assemble.ui.main_ui import LegoUI

DatasetItem = tuple[str, str, int]  # (uuid, caption, dataset_index)


class StableText2BrickBrowser:
    """Browser UI for the StableText2Brick dataset."""

    MAX_VISIBLE = 100

    def __init__(self, main_ui: LegoUI) -> None:
        self._window = ui.Window("StableText2Brick", width=500, height=600)
        self._window.deferred_dock_in("Console")

        # Index state
        self._index_loaded: bool = False
        self._dataset: list[DatasetItem] = []

        # UI / selection / filter state
        self._selected_ds_index: int | None = None
        self._filter_text: str = ""

        self._search_model = ui.SimpleStringModel("")
        self._status_model = ui.SimpleStringModel("Loading StableText2Brick index...")
        self._status_label: ui.Label | None = None
        self._list_frame: ui.Frame | None = None

        # Env id provider from main LEGO UI
        self._main_ui = main_ui

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
                        self._list_frame = ui.Frame()
                        self._list_frame.set_build_fn(self._build_list_contents)

        # Initial population
        self._rebuild_list()

    def destroy(self) -> None:
        if self._window:
            self._window.destroy()
            self._window = None

    def _set_status(self, msg: str) -> None:
        self._status_model.set_value(msg)
        if self._status_label is not None:
            self._status_label.text = msg

    def _index_path(self) -> Path:
        return Path(__file__).resolve().parents[4] / "resources" / "stabletext2brick" / "index.json"

    def _structures_dir(self) -> Path:
        return Path(__file__).resolve().parents[4] / "resources" / "stabletext2brick" / "structures"

    def _ensure_index_loaded(self) -> None:
        """Lazy-load index.json once."""
        if self._index_loaded:
            return

        carb.log_info("[StableText2Brick] loading local index.json")
        self._index_loaded = True

        index_path = self._index_path()
        if not index_path.is_file():
            self._dataset = []
            carb.log_error(f"[StableText2Brick] index.json not found at {index_path}")
            self._set_status("Missing index. Run scripts/generate_stabletext2brick.py")
            return

        try:
            with index_path.open("r", encoding="utf-8") as f:
                index = json.load(f)
        except Exception as e:
            self._dataset = []
            carb.log_error(f"[StableText2Brick] failed to read index.json: {e}")
            self._set_status("Failed to read index")
            return

        dataset: list[DatasetItem] = []
        for idx, entry in enumerate(index):
            try:
                uuid = str(entry["uuid"])
                caption = str(entry.get("caption", ""))
                dataset.append((uuid, caption, idx))
            except Exception:
                # Skip malformed entries
                carb.log_warn(f"[StableText2Brick] skipping malformed index entry: {entry}")
                continue

        self._dataset = dataset
        total = len(self._dataset)
        shown = min(total, self.MAX_VISIBLE)
        self._set_status(f"{total} loaded, {total} matches, showing {shown}")
        carb.log_info(f"[StableText2Brick] index loaded: total {len(self._dataset)}")

    def _filtered_dataset(self) -> list[DatasetItem]:
        text = (self._filter_text or "").strip().lower()
        if not self._dataset:
            return []

        if not text:
            return self._dataset

        visible: list[DatasetItem] = []
        for uuid, caption, ds_idx in self._dataset:
            if text in uuid.lower() or text in caption.lower():
                visible.append((uuid, caption, ds_idx))
        return visible

    def _rebuild_list(self) -> None:
        """Rebuild the list frame via its build_fn."""
        if self._list_frame is None:
            return

        self._ensure_index_loaded()

        # Mark frame dirty; omni.ui will call _build_list_contents at a safe time.
        self._list_frame.rebuild()

    def _build_list_contents(self) -> None:
        with ui.VStack(spacing=2):
            if not self._dataset:
                ui.Label("No index loaded.", height=0)
                return

            matches = self._filtered_dataset()
            shown = matches[: self.MAX_VISIBLE]
            num_total = len(self._dataset)
            num_matches = len(matches)
            num_shown = len(shown)

            self._set_status(f"{num_total} loaded, {num_matches} matches, showing {num_shown}")

            if not shown:
                ui.Label("No matches.", height=0)
                return

            for uuid, caption, ds_idx in shown:
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

    def _get_selected_item(self) -> DatasetItem | None:
        if self._selected_ds_index is None:
            return None
        for uuid, caption, ds_idx in self._dataset:
            if ds_idx == self._selected_ds_index:
                return (uuid, caption, ds_idx)
        return None

    def _on_import_clicked(self) -> None:
        self._ensure_index_loaded()

        item = self._get_selected_item()
        if item is None:
            self._set_status("Nothing to import.")
            return

        uuid, caption, _ = item

        # Locate bricks text
        bricks_path = self._structures_dir() / f"{uuid}.txt"
        if not bricks_path.is_file():
            carb.log_error(f"[StableText2Brick] structure file not found: {bricks_path}")
            self._set_status("Structure file not found.")
            return

        try:
            bricks_text = bricks_path.read_text(encoding="utf-8")
        except Exception as e:
            carb.log_error(f"[StableText2Brick] failed to read structure file {bricks_path}: {e}")
            self._set_status("Error reading structure file.")
            return

        color = None
        if self._main_ui is not None:
            color = self._main_ui.get_selected_color()

        # Convert text -> topology
        topology = bricks_text_to_topology_json(bricks_text, color=color)
        topology_str = json.dumps(topology)

        # Resolve env id
        env_id = -1
        if self._main_ui is not None:
            env_id = int(self._main_ui.get_env_id())

        # Import into world
        try:
            import_lego(
                topology_str,
                env_id,
                (1.0, 0.0, 0.0, 0.0),  # wxyz
                (0.0, 0.0, 0.0),
            )
        except Exception as e:
            carb.log_error(f"[StableText2Brick] import_lego failed: {e}")
            self._set_status(f"Import failed: {e}")
            return

        carb.log_info(f"[StableText2Brick] Imported structure {uuid} into env_id={env_id}")
        self._set_status("Import completed.")
