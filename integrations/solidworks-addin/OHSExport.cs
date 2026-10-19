// SPDX-License-Identifier: Apache-2.0
// SolidWorks add-in: export the active assembly for Open Horology Studio.
//
// Writes <name>.step (STEP AP242) and <name>.roles.toml (a role preset) next to the assembly. Give a
// component a role with the custom property OHS_ROLE (wheel, pinion, plate, bridge, ...); components
// without one are classified from their names, with the same rules as integrations/common/ohs_preset.py.
// Then: ohs import <name>.step -o <name>.hsproj --preset <name>.roles.toml --beat-rate 25200
//
// This file is a source sketch against the SolidWorks API (SolidWorks.Interop.sldworks). It has not been
// built here: no SolidWorks SDK is available to the project's automated tests. Build it as a COM add-in
// (see README.md) and report problems in the issue tracker.
using System;
using System.Collections.Generic;
using System.IO;
using System.Text;
using System.Text.RegularExpressions;
using SolidWorks.Interop.sldworks;
using SolidWorks.Interop.swconst;

namespace OpenHorologyStudio
{
    public static class OhsExport
    {
        static readonly string[] Roles = { "plate", "bridge", "barrel", "mainspring", "arbor", "wheel", "pinion", "escape_wheel",
                                           "pallet_fork", "balance", "hairspring", "roller", "jewel", "screw", "rotor", "date_ring",
                                           "stem", "spring", "lever", "bearing" };

        // Name fragments in the order the OHS classifier uses; the first hit wins.
        static readonly (string Pattern, string Role)[] NameRules =
        {
            ("escape", "escape_wheel"), ("pallet|fork", "pallet_fork"), ("hairspring|balance[_ ]?spring", "hairspring"),
            ("balance", "balance"), ("roller|impulse", "roller"), ("main[_ ]?spring", "mainspring"), ("barrel", "barrel"),
            ("jewel|ruby|sapphire", "jewel"), ("screw|bolt", "screw"), ("arbor|staff|axle|shaft", "arbor"), ("pinion", "pinion"),
            ("bridge|cock", "bridge"),
            // The keyless work, after the springs of the train and the bridges ("spring screw", "bearing bridge").
            ("\\bstem\\b", "stem"), ("spring", "spring"), ("lever|yoke|click|pawl|jumper", "lever"), ("bearing|bush|sleeve|tube", "bearing"),
            ("rotor|oscillating", "rotor"),
            ("date[_ -]?(wheel|ring|disc|disk|indicator)|calendar[_ -]?ring|quanti[eè]me", "date_ring"), ("plate|mainplate|base", "plate"), ("wheel|gear", "wheel"),
        };

        public static string GuessRole(string name)
        {
            var lowered = name.ToLowerInvariant();
            // "date wheel jumper" and "date wheel finger" hold or push the ring; they are not it.
            var dateHelper = Regex.IsMatch(lowered, "jumper|finger|driv|click");
            foreach (var (pattern, role) in NameRules)
                if (!(role == "date_ring" && dateHelper) && Regex.IsMatch(lowered, pattern)) return role;
            return null;
        }

        public static string BuildPreset(IEnumerable<(string Name, string Role)> parts)
        {
            var sb = new StringBuilder("# Written by the Open Horology Studio SolidWorks add-in. Edit freely.\n\n");
            var seen = new HashSet<string>();
            foreach (var (name, explicitRole) in parts)
            {
                if (!seen.Add(name)) continue;
                var role = string.IsNullOrWhiteSpace(explicitRole) ? GuessRole(name) : explicitRole.Trim().ToLowerInvariant().Replace('-', '_');
                if (role == null) { sb.Append("# ").Append(name).Append(": no role (set OHS_ROLE or rename the part)\n"); continue; }
                if (Array.IndexOf(Roles, role) < 0) throw new ArgumentException($"{name}: OHS_ROLE '{role}' is not an OHS role");
                sb.Append("[[rule]]\nmatch = '^").Append(Regex.Escape(name)).Append("$'\nrole = \"").Append(role).Append("\"\n\n");
            }
            return sb.ToString();
        }

        public static void Export(ISldWorks app, IAssemblyDoc assembly, ModelDoc2 doc, string folder, string name)
        {
            var parts = new List<(string, string)>();
            var components = (object[])assembly.GetComponents(false);
            foreach (Component2 comp in components)
            {
                var model = (ModelDoc2)comp.GetModelDoc2();
                if (model == null) continue;
                var mgr = model.Extension.CustomPropertyManager[""];
                mgr.Get5("OHS_ROLE", false, out string _, out string resolved, out bool _);
                parts.Add((comp.Name2, resolved));
            }
            int errors = 0, warnings = 0;
            doc.Extension.SaveAs(Path.Combine(folder, name + ".step"), (int)swSaveAsVersion_e.swSaveAsCurrentVersion,
                                 (int)swSaveAsOptions_e.swSaveAsOptions_Silent, null, ref errors, ref warnings);
            File.WriteAllText(Path.Combine(folder, name + ".roles.toml"), BuildPreset(parts), new UTF8Encoding(false));
        }
    }
}
