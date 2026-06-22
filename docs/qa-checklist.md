# Manual QA checklist (Rhino 8)

Run after `./build.ps1 -Configuration Release` and copying artifacts to Grasshopper Libraries.

## Install

- [ ] `Motus.GH.gha` loads without errors
- [ ] Motus tab appears in Grasshopper
- [ ] `resources/robots/` present next to DLLs

## Planning

- [ ] UR5e: Plan Joint Path with Run=true produces trajectory
- [ ] KUKA KR 6 R900 plans successfully
- [ ] Custom Robot loads a JSON preset path
- [ ] Run=false returns cached trajectory
- [ ] AutoReplan=true replans on input change
- [ ] Cartesian Path reaches plane goal via IK
- [ ] RRT Connect avoids collision sphere obstacle
- [ ] Esc / solution cancel stops RRT mid-run

## Validation & export

- [ ] Out-of-limit joint → Validate returns Valid=false
- [ ] JSON export parses; CSV header is `time_seconds,j1,...`
- [ ] Trajectory to Planes moves with joint angles (FK)

## Preview

- [ ] Preview Robot shows link meshes + TCP plane
- [ ] Preview Trajectory splits valid/invalid segments when collision wired
- [ ] UseDegrees=true on Joint State converts correctly

## Units

- [ ] Rhino document set to meters; preview scale looks correct
