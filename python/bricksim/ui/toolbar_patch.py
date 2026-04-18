"""Toolbar patches for BrickSim connected-component selection."""

import carb.settings
from carb.dictionary import Item
from carb.settings import ChangeEventType

# Engine-facing picking mode (used by omni.usd and others).
_ENGINE_PICKING_MODE_SETTING = "/persistent/app/viewport/pickingMode"

# UI-facing picking mode (used by SelectModeModel after we redirect it).
_UI_PICKING_MODE_SETTING = "/exts/bricksim/viewport/pickingMode_ui"

# LEGO-specific selection mode flag.
_LEGO_SELECTION_MODE_SETTING = "/exts/bricksim/selection_mode"
_LEGO_CC_VALUE = "connected_component"
_LEGO_COMPONENT_VALUE = "component"

# Synthetic value used only as the radio value for the extra "Connected
# Component" menu row on the toolbar.
_LEGO_PLACEHOLDER_VALUE = "lego_connected_component"


_ui_to_engine_bridge_active = False
_engine_to_ui_bridge_active = False


def _redirect_select_mode_model():
    """Redirect SelectModeModel to watch and write a UI-specific setting key.

    Use this instead of the engine-facing '/persistent/app/viewport/pickingMode'.

    This allows us to store our synthetic LEGO placeholder in the UI setting,
    while we keep the engine setting always valid and parseable for omni.usd.
    """
    from omni.kit.widget.toolbar.builtin_tools.models.select_mode_model import (
        SelectModeModel,
    )

    if getattr(SelectModeModel, "_lego_redirected", False):
        return

    # Remember the original engine-facing setting key and replace it with our UI key.
    setattr(
        SelectModeModel,
        "_lego_engine_setting",
        SelectModeModel.PICKING_MODE_SETTING,
    )
    SelectModeModel.PICKING_MODE_SETTING = _UI_PICKING_MODE_SETTING
    setattr(SelectModeModel, "_lego_redirected", True)


def _install_bridges():
    """Install bridges between UI and engine pickingMode settings.

    All toolbar UI reads/writes go through the UI key, and we propagate changes
    to the engine key (and vice versa) while
    translating the LEGO placeholder into a valid engine value.
    """
    global _ui_to_engine_bridge_active, _engine_to_ui_bridge_active

    settings = carb.settings.get_settings()

    engine_mode = settings.get_as_string(_ENGINE_PICKING_MODE_SETTING)
    lego_mode = settings.get_as_string(_LEGO_SELECTION_MODE_SETTING)

    # Initialize the UI setting to reflect the current engine + LEGO mode.
    if engine_mode == "kind:component" and lego_mode == _LEGO_CC_VALUE:
        ui_mode = _LEGO_PLACEHOLDER_VALUE
    else:
        ui_mode = engine_mode or "type:ALL"

    settings.set_default_string(_UI_PICKING_MODE_SETTING, ui_mode)
    settings.set(_UI_PICKING_MODE_SETTING, ui_mode)

    def on_ui_change(_item: Item, _event_type: ChangeEventType) -> None:
        global _ui_to_engine_bridge_active, _engine_to_ui_bridge_active
        if _ui_to_engine_bridge_active:
            return
        _ui_to_engine_bridge_active = True
        try:
            ui_mode = settings.get_as_string(_UI_PICKING_MODE_SETTING)

            if ui_mode == _LEGO_PLACEHOLDER_VALUE:
                engine = "kind:component"
                lego = _LEGO_CC_VALUE
            else:
                engine = ui_mode
                lego = _LEGO_COMPONENT_VALUE if ui_mode == "kind:component" else ""

            current_engine = settings.get_as_string(_ENGINE_PICKING_MODE_SETTING)
            if engine != current_engine:
                _engine_to_ui_bridge_active = True
                settings.set(_ENGINE_PICKING_MODE_SETTING, engine)
                _engine_to_ui_bridge_active = False

            current_lego = settings.get_as_string(_LEGO_SELECTION_MODE_SETTING)
            if lego != current_lego:
                settings.set(_LEGO_SELECTION_MODE_SETTING, lego)
        finally:
            _ui_to_engine_bridge_active = False

    def on_engine_change(_item: Item, _event_type: ChangeEventType) -> None:
        global _ui_to_engine_bridge_active, _engine_to_ui_bridge_active
        if _engine_to_ui_bridge_active:
            return
        _engine_to_ui_bridge_active = True
        try:
            engine = settings.get_as_string(_ENGINE_PICKING_MODE_SETTING)

            lego = settings.get_as_string(_LEGO_SELECTION_MODE_SETTING)
            if engine == "kind:component" and lego == _LEGO_CC_VALUE:
                ui_mode = _LEGO_PLACEHOLDER_VALUE
            else:
                ui_mode = engine

            current_ui = settings.get_as_string(_UI_PICKING_MODE_SETTING)
            if ui_mode != current_ui:
                _ui_to_engine_bridge_active = True
                settings.set(_UI_PICKING_MODE_SETTING, ui_mode)
                _ui_to_engine_bridge_active = False
        finally:
            _engine_to_ui_bridge_active = False

    settings.subscribe_to_node_change_events(_UI_PICKING_MODE_SETTING, on_ui_change)
    settings.subscribe_to_node_change_events(
        _ENGINE_PICKING_MODE_SETTING, on_engine_change
    )


def _patch_select_button_group():
    """Patch SelectButtonGroup.

    - add a 'Connected Component' entry in the Select Mode menu that uses the
      synthetic LEGO placeholder value, and
    - use a dedicated icon name and tooltip for that mode based on the
      effective picking mode (engine + LEGO selection flag).
    """
    from omni.kit.widget.options_menu import (
        OptionItem,
        OptionRadios,
        OptionSeparator,
        OptionsModel,
    )
    from omni.kit.widget.toolbar.builtin_tools.models.select_mode_model import (
        SelectModeModel,
    )
    from omni.kit.widget.toolbar.builtin_tools.select_button_group import (
        LIGHT_TYPES,
        SelectButtonGroup,
    )

    if getattr(SelectButtonGroup, "_lego_cc_patched", False):
        return

    def _build_select_menu_model(self):
        radios = [
            ("Select by Type", None),
            ("All Prim Types", "type:ALL"),
            ("Meshes", "type:Mesh"),
            ("Lights", LIGHT_TYPES),
            ("Camera", "type:Camera"),
        ]
        if self._custom_types:
            for name, types in self._custom_types:
                radios.append((name, types))

        radios.extend(
            [
                ("Select by Model Kind", None),
                ("All Model Kinds", "kind:model.ALL"),
                ("Assembly", "kind:assembly"),
                ("Group", "kind:group"),
                ("Component", "kind:component"),
                # LEGO-specific Connected Component mode. The value is
                # synthetic and will be translated by our bridges.
                ("Connected Component", _LEGO_PLACEHOLDER_VALUE),
                ("Subcomponent", "kind:subcomponent"),
            ]
        )

        plugin_kinds = self._plugin_kinds()
        if len(plugin_kinds) > 0:
            for k in plugin_kinds:
                radios.append((str(k).capitalize(), f"kind:{k}"))

        self._include_prims_with_no_kind_item = OptionItem(
            "Include Prims with no Kind",
            default=True,
            hide_on_click=True,
            model=self._select_no_kinds_model,
            enabled=self._enable_no_kinds_option(None),
        )
        self._include_references_item = OptionItem(
            "Include References and Payloads",
            default=True,
            hide_on_click=True,
            model=self._select_include_ref_model,
            enabled=self._enable_no_kinds_option(None),
        )

        option_radios = type.__call__(
            OptionRadios,
            radios,
            model=self._select_mode_model,
            default=SelectModeModel.PICKING_MODE_DEFAULT,
        )

        items = [
            option_radios,
            OptionSeparator(),
            self._include_prims_with_no_kind_item,
            self._include_references_item,
        ]
        if self._options_model is None:
            self._options_model = OptionsModel("Select Mode", items)
        else:
            self._options_model.rebuild_items(items)

    setattr(SelectButtonGroup, "_build_select_menu_model", _build_select_menu_model)

    # Patch get_style so we can use a distinct icon name for the LEGO
    # Connected Component mode while reusing the component glyph.
    orig_get_style = SelectButtonGroup.get_style

    def _lego_get_style(self):
        style = orig_get_style(self)
        comp_icon = style.get("Button.Image::component")
        if comp_icon is not None:
            style["Button.Image::component_cc"] = dict(comp_icon)
        else:
            style["Button.Image::component_cc"] = {
                "image_url": f"{self._icon_path}/component.svg"
            }
        return style

    setattr(SelectButtonGroup, "get_style", _lego_get_style)

    # Patch how the main Select Mode button chooses its icon and tooltip so
    # that we can distinguish between plain 'Component' and LEGO Connected
    # Component mode using the UI model value (which we keep in sync with the
    # engine picking mode via the bridges above).
    orig_get_name = SelectButtonGroup._get_select_mode_button_name

    def _lego_get_select_mode_button_name(self):
        mode = self._select_mode_model.get_value_as_string()
        # When the UI-mode is our LEGO placeholder, use the dedicated
        # component_cc icon entry; otherwise, fall back to the stock behavior.
        if mode == _LEGO_PLACEHOLDER_VALUE:
            return "component_cc"
        return orig_get_name(self)

    setattr(
        SelectButtonGroup,
        "_get_select_mode_button_name",
        _lego_get_select_mode_button_name,
    )

    orig_get_tooltip = SelectButtonGroup._get_select_mode_tooltip

    def _lego_get_select_mode_tooltip(self):
        mode = self._select_mode_model.get_value_as_string()
        # When the UI-mode is our LEGO placeholder, show a dedicated tooltip
        # while preserving the T hotkey display.
        if mode == _LEGO_PLACEHOLDER_VALUE:
            return f"Connected Component ({self._mode_hotkey.get_as_string('T')})"
        return orig_get_tooltip(self)

    setattr(
        SelectButtonGroup,
        "_get_select_mode_tooltip",
        _lego_get_select_mode_tooltip,
    )

    setattr(SelectButtonGroup, "_lego_cc_patched", True)


def _rebuild_select_button_group():
    """Rebuild the SelectButtonGroup instance on the main toolbar.

    Make it pick up our patched SelectModeModel and menu definitions.

    We intentionally avoid calling `clean()` on the old group here: doing so
    while the UI is live can invalidate models that omni.ui ToolButtons still
    reference, which later causes AbstractValueModel::get_value_as_bool to be
    called on an invalid model during the UI draw.
    """
    import omni.kit.widget.toolbar.extension as tb_ext
    from omni.kit.widget.toolbar.builtin_tools.select_button_group import (
        SelectButtonGroup,
    )

    toolbar = tb_ext.get_instance()
    if toolbar is None:
        return

    builtin = getattr(toolbar, "_builtin_tools", None)
    if builtin is None:
        return

    old_group = getattr(builtin, "_select_button_group", None)
    if old_group is not None:
        toolbar.remove_widget(old_group)
        builtin._select_button_group = None

    new_group = SelectButtonGroup()
    builtin._select_button_group = new_group
    toolbar.add_widget(new_group, 0)


def install_toolbar_patches():
    """Entry point invoked from our extension startup.

    Patch the Isaac toolbar at runtime. Safe to call multiple times.
    """
    _redirect_select_mode_model()
    _install_bridges()
    _patch_select_button_group()
    _rebuild_select_button_group()
