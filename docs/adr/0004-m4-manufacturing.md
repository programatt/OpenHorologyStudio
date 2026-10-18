# ADR 4: M4 manufacturing: tolerances, DFM, collisions, drawings, reports

- **Tolerance analysis is a Monte Carlo of independent dimensions.** Each tolerance is a random
  variable (normal with ±tol at 3σ, or uniform). Sample `s` draws from a counter-based generator
  (splitmix64 seeded by seed and `s`), so results are identical for any thread count and any
  platform. Backlash is linearised about the rig's centre distance:
  `j = j0 + 2 (C − C0) tan α − (Δs₁ + Δs₂)`, with the centre distance taken from the random
  pivot positions; the contact ratio is evaluated exactly at the actual centre distance. End shake
  is plate-to-bridge distance plus flatness minus arbor length; side shake is hole minus pivot
  diameter, the smaller of the two pivots. Sensitivity is the squared correlation of each input
  with the output, normalised to shares. The defaults are a workshop-grade table, not a measured
  process capability; `--tolerances` takes yours.
- **DFM rules are data.** `data/dfm/rules.toml` lists, per process, the rules to run and their
  limits, and is embedded at configure time; `--rules` adds or overrides processes. Geometry
  rules work on the 2D section outline of a part: the smallest concave corner radius (a hole is a
  concave feature whose radius is its own), the thinnest wall by ray casting across material, the
  pocket depth over the largest cutter that fits, plus tooth rules (cutter module within a
  tolerance, tooth and leaf counts, undercut below 2/sin²α) and the jewel press-fit window.
  Every rule reports a finding, Info when it passes. Rules cannot see features that vary through
  the thickness, since the part is judged by its mid-height section.
- **Collision sweeps are 2D.** Parts are reduced to section outlines with a height range and the
  mobile they turn with. A pair is swept from the assembled pose: a meshing pair through one tooth
  pitch of the driving wheel, any other pair through one revolution of its faster mobile. Poses
  beyond that depend on the whole train's common period and are not covered. The result is the
  closest approach, or the deepest overlap estimated from vertices inside the other solid;
  overlaps shallower than 20 µm count as contact within the outline accuracy.
- **Gear generator.** The same involute outline used for the FE tooth sector is extruded to a STEP
  solid (profile sampled to a few micrometres), and drawn as SVG, DXF and PDF with a front view,
  a side view and the gear data a cutter set-up needs. `--profile-only` writes just the outline
  and bore for cutting files. The profile is an involute with a circular root fillet, not the
  cycloidal form of a traditional calibre.
- **Reports are data.** A report is a `Document` of headings, tables, key values, figures and
  callouts with pass/warn/fail marks, rendered to self-contained HTML (light and dark, inline SVG)
  and to PDF by a small in-house writer (PDF 1.4, Helvetica, vector figures), so no PDF library
  or browser is needed. Charts and drawings share one vector description and so one SVG, DXF and
  PDF path. PDF text is WinAnsi: characters outside it are spelled out.
- **Part drawings** show the outline, round holes with centre marks and diameters, overall size,
  and hole positions from the lower-left datum, with the PDF at the largest standard scale that
  fits. The gate of M4 is that a part made from such a drawing fits; the automated stand-in is the
  gate test that reads the DXF back and checks the area, radii and dimensions against the solid.
- **Provenance.** Reports of an OM10 project carry the CC BY-SA 3.0 notice.
