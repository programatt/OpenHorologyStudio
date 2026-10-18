# SPDX-License-Identifier: Apache-2.0
"""Fusion 360 add-in: export the active design for Open Horology Studio.

Writes `<name>.step` (STEP AP242) and `<name>.roles.toml` (a role preset) into a folder you pick.
Set a custom attribute on a body or component (group "OHS", name "role", value e.g. "wheel") to give
it a role; parts without one are classified from their names. Then, in a terminal:

    ohs import <name>.step -o <name>.hsproj --preset <name>.roles.toml --beat-rate 25200

This add-in needs Fusion 360's `adsk` module and cannot be run outside it; the role logic lives in
integrations/common/ohs_preset.py, which is tested without Fusion.
"""
import os
import sys
import traceback

try:  # pragma: no cover - only available inside Fusion 360
    import adsk.core
    import adsk.fusion
except ImportError:  # the module can still be imported for testing
    adsk = None

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "common"))
import ohs_preset  # noqa: E402

_handlers = []


def collect_parts(design):
    """One dict per body of the design: its name and its OHS role attribute, if any."""
    parts = []
    root = design.rootComponent
    components = [root] + [occ.component for occ in root.allOccurrences]
    seen = set()
    for comp in components:
        if comp.entityToken in seen:
            continue
        seen.add(comp.entityToken)
        for body in comp.bRepBodies:
            attr = body.attributes.itemByName("OHS", "role") or comp.attributes.itemByName("OHS", "role")
            parts.append({"name": body.name, "role": attr.value if attr else ""})
    return parts


def export(design, folder, name, beat_rate_vph=None):
    """Write the STEP file and the role preset; returns their paths."""
    step_path = os.path.join(folder, name + ".step")
    options = design.exportManager.createSTEPExportOptions(step_path)
    design.exportManager.execute(options)
    preset_path = os.path.join(folder, name + ".roles.toml")
    ohs_preset.write_preset(preset_path, collect_parts(design), beat_rate_vph)
    return step_path, preset_path


def run(context):  # pragma: no cover - entry point called by Fusion 360
    ui = None
    try:
        app = adsk.core.Application.get()
        ui = app.userInterface
        design = adsk.fusion.Design.cast(app.activeProduct)
        if design is None:
            ui.messageBox("Open a design first.")
            return
        dialog = ui.createFolderDialog()
        dialog.title = "Export for Open Horology Studio"
        if dialog.showDialog() != adsk.core.DialogResults.DialogOK:
            return
        name = design.rootComponent.name.replace(" ", "_")
        step_path, preset_path = export(design, dialog.folder, name)
        ui.messageBox("Wrote:\n" + step_path + "\n" + preset_path + "\n\nNow run ohs import with --preset.")
    except Exception:
        if ui:
            ui.messageBox("OHS export failed:\n" + traceback.format_exc())


def stop(context):  # pragma: no cover
    pass
