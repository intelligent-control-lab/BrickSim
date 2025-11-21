import json
from pathlib import Path
from typing import Callable, List, Tuple

import carb
import omni.ui as ui

from lego_assemble._native import import_lego
from lego_assemble.importers.stabletext2brick import bricks_text_to_topology_json


class StableText2BrickBrowser:
    """Browser UI for the StableText2Brick dataset.

    - Uses resources/stabletext2brick/index.json as the dataset index.
    - Shows a searchable list of (uuid, caption) rows.
    - On double-click, loads resources/stabletext2brick/structures/{uuid}.txt,
      converts the bricks text via `bricks_text_to_topology_json`, and imports
      into the env_id provided by the main LEGO UI.
    """

    def __init__(self, env_id_provider: Callable[[], int] | None = None) -> None:
        self._window = ui.Window("StableText2Brick", width=500, height=600)

        # Local index and cached example info.
        self._dataset_load_attempted = False
        # (uuid, caption, index) from index.json
        self._examples: List[Tuple[str, str, int]] = []

        # Selection / filter / status state.
        self._selected_index: int | None = None
        self._filter_text: str = ""
        self._search_model = ui.SimpleStringModel("")
        self._status_model = ui.SimpleStringModel(
            "Loading StableText2Brick index (resources/stabletext2brick/index.json) ..."
        )
        # Status line shows both counts and the last info/error message.
        self._status_label: ui.Label | None = None

        # Env id provider (from main UI); may be None.
        self._get_env_id = env_id_provider

        self._list_frame: ui.Frame | None = None

        # When search text changes, rebuild only the list, to keep focus in the field.
        def _on_search_changed(model: ui.AbstractValueModel) -> None:
            self._filter_text = model.as_string
            if self._list_frame is not None:
                self._list_frame.rebuild()

        self._search_model.add_value_changed_fn(_on_search_changed)

        # Build the static UI once: header + scrollable list frame.
        with self._window.frame:
            with ui.VStack(spacing=2):
                # Search box (env_id taken from main UI).
                with ui.HStack(spacing=5, height=0):
                    ui.Label("Search (id / caption):", width=120, height=0)
                    ui.StringField(self._search_model, height=0)

                # Status line: counts + last info/error message.
                self._status_label = ui.Label(self._status_model.as_string, height=0)

                # Scrollable list area: the frame is rebuilt independently.
                with ui.ScrollingFrame(
                    horizontal_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_ALWAYS_OFF,
                    vertical_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_AS_NEEDED,
                ):
                    with ui.VStack(spacing=2):
                        self._list_frame = ui.Frame()
                        self._list_frame.set_build_fn(self._build_list_contents)

    def destroy(self) -> None:
        if self._window:
            self._window.destroy()
            self._window = None

    # --------------------------------------------------------------------- utils

    def _set_status(self, msg: str) -> None:
        self._status_model.set_value(msg)
        if self._status_label is not None:
            self._status_label.text = msg

    def _load_dataset(self) -> None:
        """Load StableText2Brick index from resources/stabletext2brick/index.json."""
        carb.log_info("[StableText2Brick] _load_dataset called (local index)")
        self._dataset_load_attempted = True

        # Compute path to resources/stabletext2brick/index.json relative to this file.
        try:
            root = Path(__file__).resolve().parents[4]
        except Exception as exc:
            msg = f"StableText2Brick: failed to resolve repo root from __file__: {exc}"
            carb.log_error(msg)
            self._examples = []
            self._set_status("Error. See console.")
            if self._list_frame is not None:
                self._list_frame.rebuild()
            return

        index_path = root / "resources" / "stabletext2brick" / "index.json"
        if not index_path.is_file():
            msg = (
                "Missing index. Run scripts/export_stabletext2brick_index.py resources/stabletext2brick/ to build index."
            )
            carb.log_error(msg)
            self._examples = []
            self._set_status(
                "Missing index. Run scripts/export_stabletext2brick_index.py resources/stabletext2brick/ to build index."
            )
            if self._list_frame is not None:
                self._list_frame.rebuild()
            return

        try:
            with index_path.open("r", encoding="utf-8") as f:
                data = json.load(f)
        except Exception as exc:
            msg = f"StableText2Brick: failed to read index.json: {exc}"
            carb.log_error(msg)
            self._examples = []
            self._set_status("Error. See console.")
            if self._list_frame is not None:
                self._list_frame.rebuild()
            return

        examples: List[Tuple[str, str, int]] = []
        for idx, entry in enumerate(data):
            try:
                uuid = str(entry["uuid"])
                caption = str(entry.get("caption", ""))
                examples.append((uuid, caption, idx))
            except Exception:
                continue

        self._dataset = None
        self._examples = examples
        total = len(self._examples)
        shown = min(total, 100)
        self._set_status(f"{total} loaded, {total} matches, showing {shown}")

        if self._list_frame is not None:
            self._list_frame.rebuild()

    # ----------------------------------------------------------------- list UI

    def _build_list_contents(self) -> None:
        """Builds the scrollable list of structures."""
        carb.log_info(
            f"[StableText2Brick] _build_list_contents called; "
            f"examples={len(self._examples)}, "
            f"load_attempted={self._dataset_load_attempted}"
        )
        # Lazy-load the dataset the first time the list is built.
        if not self._dataset_load_attempted:
            self._load_dataset()
            carb.log_info(
                f"[StableText2Brick] after _load_dataset; examples={len(self._examples)}"
            )
        with ui.VStack(spacing=2):
            if not self._examples:
                # No index or entries; status line already contains the message.
                return

            text = self._filter_text.strip().lower()
            visible_all: List[Tuple[str, str, int]] = []

            for sid, caption, ds_idx in self._examples:
                if not text:
                    visible_all.append((sid, caption, ds_idx))
                else:
                    sid_l = sid.lower()
                    cap_l = caption.lower()
                    if text in sid_l or text in cap_l:
                        visible_all.append((sid, caption, ds_idx))

            total_loaded = len(self._examples)
            matches = len(visible_all)

            # Only show the top 100 matches.
            visible = visible_all[:100]
            shown = len(visible)
            self._set_status(f"{total_loaded} loaded, {matches} matches, showing {shown}")

            for sid, caption, ds_idx in visible:
                selected = self._selected_index == ds_idx
                bg_color = 0x40808080 if selected else 0x00000000

                # Make the entire row clickable by attaching mouse handlers
                # to the row container (ZStack), not just the text labels.
                row = ui.ZStack(height=22)
                with row:
                    ui.Rectangle(style={"background_color": bg_color})
                    with ui.HStack(spacing=8, height=0):
                        ui.Spacer(width=5)
                        # Fixed-width ID column to emulate a table-like layout.
                        ui.Label(
                            sid,
                            width=260,
                            height=0,
                            ellipsize=True,
                        )
                        ui.Label(
                            caption,
                            height=0,
                            ellipsize=True,
                        )

                # Single-click: select row and refresh highlight.
                def _on_single_click(x, y, b, m, ds_idx=ds_idx):
                    if b != 0:
                        return
                    self._selected_index = ds_idx
                    if self._list_frame is not None:
                        self._list_frame.rebuild()

                # Double-click: import immediately.
                def _on_double_click(x, y, b, m, ds_idx=ds_idx):
                    if b != 0:
                        return
                    self._selected_index = ds_idx
                    self._on_import_clicked()

                # Attach handlers to the whole row so clicks anywhere on the
                # row (not just on text) are detected.
                row.set_mouse_pressed_fn(_on_single_click)
                row.set_mouse_double_clicked_fn(_on_double_click)

    # ----------------------------------------------------------------- actions

    def _on_import_clicked(self) -> None:
        if self._selected_index is None:
            self._set_status("Error. See console.")
            return

        try:
            uuid, caption, _ = self._examples[self._selected_index]
        except Exception as exc:
            carb.log_error(f"[StableText2Brick] failed to access index entry: {exc}")
            self._set_status("Error. See console.")
            return

        # Locate the bricks text for this uuid.
        try:
            root = Path(__file__).resolve().parents[4]
            structures_dir = root / "resources" / "stabletext2brick" / "structures"
            bricks_path = structures_dir / f"{uuid}.txt"
        except Exception as exc:
            carb.log_error(f"[StableText2Brick] failed to resolve structures path: {exc}")
            self._set_status("Error. See console.")
            return

        if not bricks_path.is_file():
            carb.log_error(f"[StableText2Brick] structure file not found: {bricks_path}")
            self._set_status("Error. See console.")
            return

        try:
            bricks_text = bricks_path.read_text(encoding="utf-8")
        except Exception as exc:
            carb.log_error(f"[StableText2Brick] failed to read {bricks_path}: {exc}")
            self._set_status("Error. See console.")
            return

        try:
            topology = bricks_text_to_topology_json(bricks_text)
        except Exception as exc:
            carb.log_error(f"[StableText2Brick] conversion error: {exc}")
            self._set_status("Error. See console.")
            return

        try:
            topology["_meta"] = {
                "structure_id": uuid,
                "caption": caption,
            }
        except Exception:
            # Metadata is non-essential.
            pass

        topology_str = json.dumps(topology)

        if self._get_env_id is not None:
            try:
                env_id = int(self._get_env_id())
            except Exception:
                env_id = -1
        else:
            env_id = -1

        try:
            # Reference transform uses quaternion order wxyz and stage units.
            import_lego(topology_str, env_id, (1.0, 0.0, 0.0, 0.0), (0.0, 0.0, 0.0))
        except Exception as exc:
            carb.log_error(f"[StableText2Brick] import_lego failed: {exc}")
            self._set_status("Error. See console.")
            return

        carb.log_info(
            f"[StableText2Brick] Imported structure {uuid} into env_id={env_id}"
        )
        self._set_status("Import completed.")
