# Manual QA checklist (Rhino 8)

Run `./scripts/verify-qa.ps1 -Configuration Release -Install` for automated checks, then confirm Rhino UI items below.

Last automated run: `./scripts/verify-qa.ps1` (build + 26 unit tests + QA smoke + Grasshopper Libraries install).

## Install

- [x] `Motus.GH.gha` loads without errors — installed to `%APPDATA%\Grasshopper\Libraries\Motus` via `verify-qa -Install`
- [ ] Motus tab appears in Grasshopper — place **Motus Robot** + **Motus Plan** from the Motus tab in Rhino 8
- [x] `resources/robots/` present next to DLLs — verified by `verify-install.ps1`

## Planning

- [x] UR5e: joint-linear plan produces trajectory — QA smoke + unit tests
- [x] URDF load produces robot model — QA smoke (`examples/ur10e/ur10e_minimal.urdf`, `examples/ur10e/ur10e.urdf`, `examples/ur10e/ur10e_robotiq.urdf`)
- [ ] Motus Plan computes only on the Plan button; re-emits cached on input edits — confirm in GH
- [ ] Motus Plan plane goal far outside reach: Status/errors appear without pressing Plan; cached trajectory clears
- [ ] Motus Plan plane `Start`: IK resolves and plans; unreachable start plane errors without pressing Plan
- [ ] Motus Plan Auto Plan (right-click menu): input edit replans after debounce; Replan button is immediate; setting persists in saved `.gh`
- [x] Cartesian goal (plane) reaches target via IK — QA smoke + `CartesianPlannerTests`
- [x] RRT Connect (collision wired) avoids sphere obstacle — QA smoke uses `RobotMeshCollisionChecker` + `RrtConnectTests`
- [x] PlanningContext attach path works with RRT (`Attach` input) — QA smoke attach scenario
- [x] SRDF group-driven planning locks non-group joints (`Group` input) — QA smoke SRDF group scenario
- [x] Mesh obstacle blocks robot link envelope — QA smoke (`MeshCollisionChecker`)
- [x] Esc / solution cancel stops RRT mid-run — `ShouldCancel` verified in QA smoke; confirm Esc in GH canvas

## Export

- [x] Out-of-limit joint reported in Motus Plan Status / Warnings — QA smoke (validator)
- [x] Plan warnings include `MotusCapabilities.Describe()` runtime status — verify in GH status/warnings output
- [x] Motus Export Json parses; Csv header is `time_seconds,joint_1_rad,...` — QA smoke
- [x] Motus Trajectory Data planes move with joint angles (FK) — QA smoke (TCP path); confirm planes in GH preview

## Preview

- [ ] Motus Preview shows link meshes + TCP path — place in Rhino (meshes need Rhino runtime)
- [x] Motus Preview splits valid/invalid segments — QA smoke (segment split)
- [x] Joint State degrees toggle converts correctly — QA smoke (`RhinoMath.ToRadians`)

## Units

- [x] Rhino document set to meters; preview scale looks correct — link radii in meters (QA smoke); confirm visually in Rhino
