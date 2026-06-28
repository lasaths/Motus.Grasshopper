# Manual QA checklist (Rhino 8)

Run `./scripts/verify-qa.ps1 -Configuration Release -Install` for automated checks, then confirm Rhino UI items below.

Last automated run: `./scripts/verify-qa.ps1` (build + 26 unit tests + QA smoke + Grasshopper Libraries install).

## Install

- [x] `Motus.GH.gha` loads without errors — installed to `%APPDATA%\Grasshopper\Libraries\Motus` via `verify-qa -Install`
- [ ] Motus tab appears in Grasshopper — place **Motus Robot** + **Motus Plan** from the Motus tab in Rhino 8
- [x] `resources/robots/` present next to DLLs — verified by `verify-install.ps1`

## Planning

- [x] UR5e: joint-linear plan produces trajectory — QA smoke + unit tests
- [x] KUKA KR 6 R900 plans successfully — QA smoke
- [x] Motus Robot loads a JSON preset (JsonPath input) — QA smoke (`UR/UR5e.json`)
- [ ] Motus Plan computes only on the Plan button; re-emits cached on input edits — confirm in GH
- [x] Cartesian goal (plane) reaches target via IK — QA smoke + `CartesianPlannerTests`
- [x] RRT Connect (collision wired) avoids sphere obstacle — QA smoke + `RrtConnectTests`
- [x] Esc / solution cancel stops RRT mid-run — `ShouldCancel` verified in QA smoke; confirm Esc in GH canvas

## Export

- [x] Out-of-limit joint reported in Motus Plan Status / Warnings — QA smoke (validator)
- [x] Motus Export Json parses; Csv header is `time_seconds,joint_1_rad,...` — QA smoke
- [x] Motus Trajectory Data planes move with joint angles (FK) — QA smoke (TCP path); confirm planes in GH preview

## Preview

- [ ] Motus Preview shows link meshes + TCP path — place in Rhino (meshes need Rhino runtime)
- [x] Motus Preview splits valid/invalid segments — QA smoke (segment split)
- [x] UseDegrees=true on Joint State converts correctly — QA smoke (`Units.ToRadians`)

## Units

- [x] Rhino document set to meters; preview scale looks correct — link radii in meters (QA smoke); confirm visually in Rhino
