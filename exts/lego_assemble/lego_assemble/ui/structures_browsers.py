import json
from dataclasses import dataclass
from pathlib import Path

import carb
import omni.ui as ui

from lego_assemble._native import import_lego, arrange_parts_in_workspace
from lego_assemble.importers.legolization import legolization_json_to_topology_json
from lego_assemble.ui.main_ui import LegoUI
from lego_assemble.utils.usd_parse import get_env_path


@dataclass
class DatasetItem:
    """Single entry in the LegoSim structures dataset."""

    category: str
    model_id: str
    json_path: Path
    caption: str


class LegoStructuresBrowser:
    """Browser UI for the LegoSim legolization-based structures dataset."""

    MAX_VISIBLE = 100
    NUM_BRICKS_LIMIT = 50

    def __init__(self, main_ui: LegoUI) -> None:
        self._window = ui.Window("Lego Structures", width=500, height=600)
        self._window.deferred_dock_in("Console")

        # Dataset and UI state
        self._dataset: list[DatasetItem] = []
        self._selected_index: int | None = None
        self._filter_text: str = ""

        self._search_model = ui.SimpleStringModel("")
        self._status_model = ui.SimpleStringModel("Loading dataset...")
        self._status_label: ui.Label | None = None
        self._list_frame: ui.Frame | None = None

        # Main LEGO UI for env id / color
        self._main_ui = main_ui

        # Static window layout
        def _on_search_changed(model: ui.AbstractValueModel) -> None:
            self._filter_text = model.as_string or ""
            self._rebuild_list()

        self._search_model.add_value_changed_fn(_on_search_changed)

        with self._window.frame:
            with ui.VStack(spacing=2):
                # Search row
                with ui.HStack(spacing=5, height=0):
                    ui.Label("Search:", width=60, height=0)
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

        # Load dataset eagerly (no async / lazy loading)
        self._load_dataset()
        self._rebuild_list()

    # ------------------------------------------------------------------ helpers

    def destroy(self) -> None:
        if self._window:
            self._window.destroy()
            self._window = None

    def _set_status(self, msg: str) -> None:
        self._status_model.set_value(msg)
        if self._status_label is not None:
            self._status_label.text = msg

    def _dataset_json_path(self) -> Path:
        """Location of simulator_testset/dataset.json relative to this file."""
        return (
            Path(__file__).resolve().parents[4]
            / "resources"
            / "legosim_dataset"
            / "data"
            / "lego"
            / "data"
            / "simulator_testset"
            / "dataset.json"
        )

    def _dataset_root(self) -> Path:
        """Root directory corresponding to /data in dataset.json paths."""
        return (
            Path(__file__).resolve().parents[4]
            / "resources"
            / "legosim_dataset"
        )

    def _load_dataset(self) -> None:
        """Read and flatten simulator_testset/dataset.json into DatasetItem list."""
        dataset_path = self._dataset_json_path()
        if not dataset_path.is_file():
            carb.log_error(f"[LegoStructures] dataset.json not found at {dataset_path}")
            self._dataset = []
            self._set_status("Dataset not found. Run scripts/download_legosim_dataset.py")
            return

        try:
            with dataset_path.open("r", encoding="utf-8") as f:
                raw = json.load(f)
        except Exception as e:
            carb.log_error(f"[LegoStructures] failed to read dataset.json: {e}")
            self._dataset = []
            self._set_status("Failed to read dataset.json")
            return

        items: list[DatasetItem] = []
        root = self._dataset_root()

        # The structure is:
        # {category: {model_id: {json_path: { ...fields... }}}}
        for category, models in raw.items():
            if not isinstance(models, dict):
                continue
            for model_id, paths in models.items():
                if not isinstance(paths, dict):
                    continue
                for json_key, meta in paths.items():
                    try:
                        num_bricks = int(meta.get("num_bricks", 0))
                        if num_bricks > self.NUM_BRICKS_LIMIT:
                            continue
                        caption = str(meta.get("caption", ""))
                        json_fname = str(meta.get("json_fname", json_key))
                        # json_fname is an absolute-style path starting with /data/...
                        rel = json_fname.lstrip("/")  # remove leading slash
                        json_path = root / rel
                        items.append(
                            DatasetItem(
                                category=str(category),
                                model_id=str(model_id),
                                json_path=json_path,
                                caption=caption,
                            )
                        )
                    except Exception:
                        carb.log_warn(f"[LegoStructures] skipping malformed entry in category={category}, model={model_id}")
                        continue

        self._dataset = items
        total = len(self._dataset)
        shown = min(total, self.MAX_VISIBLE)
        self._set_status(f"{total} loaded, {total} matches, showing {shown}")
        carb.log_info(f"[LegoStructures] dataset loaded: total {total}")

    def _filtered_dataset(self) -> list[DatasetItem]:
        text = (self._filter_text or "").strip().lower()
        if not self._dataset:
            return []

        if not text:
            return self._dataset

        visible: list[DatasetItem] = []
        for item in self._dataset:
            haystack = " ".join(
                [item.category, item.model_id, item.caption, str(item.json_path)]
            ).lower()
            if text in haystack:
                visible.append(item)
        return visible

    def _rebuild_list(self) -> None:
        if self._list_frame is None:
            return
        self._list_frame.rebuild()

    def _build_list_contents(self) -> None:
        with ui.VStack(spacing=2):
            if not self._dataset:
                ui.Label("No dataset loaded.", height=0)
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

            for idx, item in enumerate(shown):
                selected = self._selected_index == idx
                bg_color = 0x40808080 if selected else 0x00000000

                row = ui.ZStack(height=22)
                with row:
                    ui.Rectangle(style={"background_color": bg_color})
                    with ui.HStack(spacing=8, height=0):
                        ui.Spacer(width=5)
                        ui.Label(f"[{item.category}]", width=80, height=0, ellipsize=True)
                        ui.Label(item.model_id, width=240, height=0, ellipsize=True)
                        ui.Label(item.caption, height=0, ellipsize=True)

                def _on_single_click(x, y, button, modifiers, idx=idx):
                    if button != 0:
                        return
                    self._selected_index = idx
                    self._rebuild_list()

                def _on_double_click(x, y, button, modifiers, idx=idx):
                    if button != 0:
                        return
                    self._selected_index = idx
                    self._on_import_clicked(shown[idx])

                row.set_mouse_pressed_fn(_on_single_click)
                row.set_mouse_double_clicked_fn(_on_double_click)

    # ----------------------------------------------------------------- actions

    def _get_selected_item(self) -> DatasetItem | None:
        if self._selected_index is None:
            return None
        matches = self._filtered_dataset()
        if not matches:
            return None
        if self._selected_index < 0 or self._selected_index >= len(matches):
            return None
        return matches[self._selected_index]

    def _on_import_clicked(self, item: DatasetItem | None = None) -> None:
        if item is None:
            item = self._get_selected_item()
        if item is None:
            self._set_status("Nothing to import.")
            return

        json_path = item.json_path
        if not json_path.is_file():
            carb.log_error(f"[LegoStructures] JSON file not found: {json_path}")
            self._set_status("JSON file not found.")
            return

        try:
            with json_path.open("r", encoding="utf-8") as f:
                lego_structure = json.load(f)
        except Exception as e:
            carb.log_error(f"[LegoStructures] failed to read {json_path}: {e}")
            self._set_status("Error reading JSON file.")
            return

        color = None
        if self._main_ui is not None:
            color = self._main_ui.get_selected_color()

        try:
            topology = legolization_json_to_topology_json(lego_structure, color=color)
            topology_str = json.dumps(topology)
        except Exception as e:
            carb.log_error(f"[LegoStructures] legolization_json_to_topology_json failed: {e}")
            self._set_status("Conversion to topology failed.")
            return

        env_id = -1
        if self._main_ui is not None:
            env_id = int(self._main_ui.get_env_id())

        try:
            imported_parts, _ = import_lego(topology_str, env_id)
        except Exception as e:
            carb.log_error(f"[LegoStructures] import_lego failed: {e}")
            self._set_status(f"Import failed: {e}")
            return

        carb.log_info(f"[LegoStructures] Imported structure {item.category}/{item.model_id} from {item.json_path} into env_id={env_id}")

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

        self._set_status("Import completed.")
