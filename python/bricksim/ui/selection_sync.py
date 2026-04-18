"""Selection synchronization for BrickSim connected-component picking."""

import carb.settings
import omni.usd

from bricksim.core import compute_connected_component
from bricksim.utils.sim import get_current_stage


class AssemblySelectionSync:
    """Expands a single-part pick into its full connected component.

    This applies when Assembly mode is active.
    """

    PICKING_MODE_SETTING = "/persistent/app/viewport/pickingMode"
    LEGO_SELECTION_MODE_SETTING = "/exts/bricksim/selection_mode"
    LEGO_CC_VALUE = "connected_component"

    def __init__(self):
        """Subscribe to USD selection changes."""
        self._settings = carb.settings.get_settings()
        self._usd_context = omni.usd.get_context()
        stage_event_stream = self._usd_context.get_stage_event_stream()
        self._stage_event_sub = stage_event_stream.create_subscription_to_pop_by_type(
            int(omni.usd.StageEventType.SELECTION_CHANGED), self._on_selection_changed
        )
        self._updating_selection = False

    def destroy(self):
        """Unsubscribe from USD selection changes."""
        if self._stage_event_sub is not None:
            self._stage_event_sub.unsubscribe()
            self._stage_event_sub = None

    def _is_assembly_mode(self) -> bool:
        mode = self._settings.get(self.PICKING_MODE_SETTING)
        return isinstance(mode, str) and mode == "kind:assembly"

    def _is_connected_component_mode(self) -> bool:
        picking_mode = self._settings.get(self.PICKING_MODE_SETTING)
        lego_mode = self._settings.get(self.LEGO_SELECTION_MODE_SETTING)
        return (
            isinstance(picking_mode, str)
            and picking_mode == "kind:component"
            and lego_mode == self.LEGO_CC_VALUE
        )

    def _on_selection_changed(self, _event):
        if self._updating_selection or not (
            self._is_assembly_mode() or self._is_connected_component_mode()
        ):
            return

        stage = get_current_stage()
        if stage is None:
            return
        selection = self._usd_context.get_selection()
        if selection is None:
            return

        selected_paths = selection.get_selected_prim_paths()
        if not selected_paths:
            return

        component_paths: list[str] = []
        seen = set()
        for path in selected_paths:
            part_paths, conn_paths = compute_connected_component(path)
            for comp_path in (*part_paths, *conn_paths):
                if comp_path in seen:
                    continue
                seen.add(comp_path)
                component_paths.append(comp_path)

        if not component_paths or set(component_paths) == set(selected_paths):
            return

        self._updating_selection = True
        try:
            selection.set_selected_prim_paths(component_paths, True)
        finally:
            self._updating_selection = False
