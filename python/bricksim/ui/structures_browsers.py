"""UI browser for importing BrickSim structures dataset entries."""

import asyncio
import json

import carb
import omni.ui as ui

from bricksim.colors import parse_color
from bricksim.core import arrange_parts_in_workspace, import_lego
from bricksim.structures import BricksimDatasetItem, load_bricksim_dataset
from bricksim.topology.legolization import legolization_json_to_topology_json
from bricksim.utils.env_paths import get_env_path

from .main_ui import LegoUI


class LegoStructuresBrowser:
    """Browse and import structures from the BrickSim dataset."""

    MAX_VISIBLE = 100
    NUM_BRICKS_LIMIT = 100

    def __init__(self, main_ui: LegoUI) -> None:
        """Create the structures browser window."""
        self._window = ui.Window("StableText2Brick Structures", width=500, height=600)
        self._window.deferred_dock_in("Console")

        self._dataset: list[BricksimDatasetItem] = []
        self._selected_index: int | None = None

        self._search_model = ui.SimpleStringModel("")
        self._use_baseplate_model = ui.SimpleBoolModel()
        self._use_baseplate_model.set_value(False)
        self._show_connected_only = ui.SimpleBoolModel()
        self._show_connected_only.set_value(False)
        self._status_model = ui.SimpleStringModel("Loading dataset...")
        self._status_label: ui.Label | None = None
        self._list_frame: ui.Frame | None = None

        self._main_ui = main_ui

        self._search_model.add_value_changed_fn(lambda _: self._rebuild_list())
        self._use_baseplate_model.add_value_changed_fn(lambda _: self._rebuild_list())
        self._show_connected_only.add_value_changed_fn(lambda _: self._rebuild_list())

        with self._window.frame:
            with ui.VStack(spacing=2):
                # Search row
                with ui.HStack(spacing=5, height=0):
                    ui.Label("Search:", width=60, height=0)
                    ui.StringField(self._search_model, height=0)
                    ui.Label(
                        "Use Baseplate",
                        width=100,
                        height=0,
                        alignment=ui.Alignment.RIGHT_CENTER,
                    )
                    ui.CheckBox(model=self._use_baseplate_model, width=20, height=0)
                    ui.Label(
                        "Show Connected Only",
                        width=150,
                        height=0,
                        alignment=ui.Alignment.RIGHT_CENTER,
                    )
                    ui.CheckBox(model=self._show_connected_only, width=20, height=0)

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

        asyncio.ensure_future(self._load_dataset())

    def destroy(self) -> None:
        """Destroy the structures browser window."""
        if self._window:
            self._window.destroy()
            self._window = None

    def _set_status(self, msg: str) -> None:
        self._status_model.set_value(msg)
        if self._status_label is not None:
            self._status_label.text = msg

    async def _load_dataset(self) -> None:
        items: list[BricksimDatasetItem] = []
        try:
            for item in (await load_bricksim_dataset()).values():
                if item.num_bricks > self.NUM_BRICKS_LIMIT:
                    continue
                items.append(item)
            self._dataset = items
            total = len(self._dataset)
            shown = min(total, self.MAX_VISIBLE)
            self._set_status(f"{total} loaded, {total} matches, showing {shown}")
            carb.log_info(f"[LegoStructures] dataset loaded: total {total}")
        except Exception as e:
            carb.log_error(f"[LegoStructures] failed to load dataset: {e}")
            self._dataset = []
            self._set_status(f"Failed to load dataset: {e}")
        self._rebuild_list()

    def _filtered_dataset(self) -> list[BricksimDatasetItem]:
        if not self._dataset:
            return []

        filter_text = self._search_model.get_value_as_string().strip().lower()
        if filter_text == "":
            filter_text = None
        use_baseplate = self._use_baseplate_model.get_value_as_bool()
        show_connected_only = self._show_connected_only.get_value_as_bool()

        visible: list[BricksimDatasetItem] = []
        for item in self._dataset:
            if show_connected_only:
                if use_baseplate and not item.all_connected_with_plate:
                    continue
                if not use_baseplate and not item.all_connected_no_plate:
                    continue
            if filter_text is not None:
                haystack = " ".join(
                    [item.category, item.model_id, item.caption, str(item.json_path)]
                ).lower()
                if filter_text not in haystack:
                    continue
            visible.append(item)
        return visible

    def _rebuild_list(self) -> None:
        if self._list_frame is None:
            return
        self._list_frame.rebuild()

    def _build_list_contents(self) -> None:
        with ui.VStack(spacing=2):
            if not self._dataset:
                return

            matches = self._filtered_dataset()
            shown = matches[: self.MAX_VISIBLE]
            num_total = len(self._dataset)
            num_matches = len(matches)
            num_shown = len(shown)

            self._set_status(
                f"{num_total} loaded, {num_matches} matches, showing {num_shown}"
            )

            if not shown:
                ui.Label("No matches.", height=0)
                return

            with ui.HStack(spacing=8, height=0):
                ui.Spacer(width=5)
                ui.Label("Category", width=80, height=0, ellipsize=True)
                ui.Label("Model", width=240, height=0, ellipsize=True)
                ui.Label(
                    "Bricks", width=50, height=0, alignment=ui.Alignment.RIGHT_CENTER
                )
                ui.Label("Caption", height=0, ellipsize=True)

            for idx, item in enumerate(shown):
                selected = self._selected_index == idx
                bg_color = 0x40808080 if selected else 0x00000000

                row = ui.ZStack(height=22)
                with row:
                    ui.Rectangle(style={"background_color": bg_color})
                    with ui.HStack(spacing=8, height=0):
                        ui.Spacer(width=5)
                        ui.Label(
                            f"[{item.category}]", width=80, height=0, ellipsize=True
                        )
                        ui.Label(item.model_id, width=240, height=0, ellipsize=True)
                        ui.Label(
                            str(item.num_bricks),
                            width=50,
                            height=0,
                            alignment=ui.Alignment.RIGHT_CENTER,
                        )
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

    def _get_selected_item(self) -> BricksimDatasetItem | None:
        if self._selected_index is None:
            return None
        matches = self._filtered_dataset()
        if not matches:
            return None
        if self._selected_index < 0 or self._selected_index >= len(matches):
            return None
        return matches[self._selected_index]

    def _on_import_clicked(self, item: BricksimDatasetItem | None = None) -> None:
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
        use_baseplate = bool(self._use_baseplate_model.get_value_as_bool())

        try:
            topology = legolization_json_to_topology_json(
                lego_structure,
                color=color,
                include_base_plate=use_baseplate,
                base_plate_size=(32, 32),
                base_plate_color=parse_color("Light Gray"),
            )
        except Exception as e:
            carb.log_error(
                f"[LegoStructures] legolization_json_to_topology_json failed: {e}"
            )
            self._set_status("Conversion to topology failed.")
            return

        env_id = -1
        if self._main_ui is not None:
            env_id = int(self._main_ui.get_env_id())

        try:
            part_paths, _ = import_lego(topology, env_id)
            imported_parts = [part_paths[k] for k in sorted(part_paths)]
        except Exception as e:
            carb.log_error(f"[LegoStructures] import_lego failed: {e}")
            self._set_status(f"Import failed: {e}")
            return

        carb.log_info(
            "[LegoStructures] Imported structure "
            f"{item.category}/{item.model_id} from {item.json_path} "
            f"into env_id={env_id}"
        )

        workspace_path = get_env_path(env_id) + "/LegoWorkspace"
        try:
            _, not_placed = arrange_parts_in_workspace(
                workspace_path, imported_parts, [1] * len(imported_parts)
            )
        except Exception as exc:
            carb.log_warn(f"Failed to arrange parts in workspace: {exc}")
            self._set_status("Arrangement in workspace failed.")
            return
        if not_placed:
            carb.log_warn(
                f"Could not place parts {not_placed} in workspace {workspace_path}"
            )
            self._set_status("Some parts could not be placed in workspace.")
            return

        self._set_status("Import completed.")
