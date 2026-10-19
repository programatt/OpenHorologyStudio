# CAD add-ins

Optional add-ins that export an assembly from your CAD tool as STEP AP242 plus a **role preset**
(`<name>.roles.toml`), so `ohs import` does not have to guess what each part is. They are built
and versioned separately from the OHS core.

| Tool | Folder | Status |
| --- | --- | --- |
| Any (Python) | `common/ohs_preset.py` | The shared role logic and preset writer. Tested in CI (`tests/python/test_preset.py`), including an import through the real `ohs`. |
| Fusion 360 | `fusion360-addin/OHSExport/` | Python add-in. Install it from Fusion's *Scripts and Add-Ins* dialog. Needs Fusion's `adsk` module, so only the shared logic is tested here. |
| SolidWorks | `solidworks-addin/OHSExport.cs` | C# source sketch against the SolidWorks API. Not built or tested in this repository (no SDK available). |

## Giving a part a role

Set a custom property (SolidWorks) or attribute (Fusion 360, group `OHS`, name `role`) called
`OHS_ROLE` on the part. Valid roles: `plate`, `bridge`, `barrel`, `mainspring`, `arbor`, `wheel`,
`pinion`, `escape_wheel`, `pallet_fork`, `balance`, `hairspring`, `roller`, `jewel`, `screw`, `rotor`
(the oscillating weight of an automatic).
Parts without a role are classified from their names (`escape wheel`, `third pinion`, `main plate`,
...). The preset matches each part by its exact name, so keep part names unique and stable.

## Using the export

```
ohs import movement.step -o movement.hsproj --preset movement.roles.toml --beat-rate 28800
ohs rig check movement.hsproj
```

OM10 users do not need an add-in: `ohs thirdparty om10 fetch --file <your downloaded .stp>`
applies the OM10 role preset that ships with OHS.
