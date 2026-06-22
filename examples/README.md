# Example Grasshopper Definitions

Binary `.gh` files must be saved from **Rhino 8** after you verify the plugin with [scripts/verify-install.ps1](../scripts/verify-install.ps1) and [docs/qa-checklist.md](../docs/qa-checklist.md).

Once tested, save canvases here:

| File | Canvas |
|------|--------|
| `01_basic_joint_planning.gh` | UR5e joint plan + validate + preview |
| `02_ur_presets.gh` | UR3e / UR10e / UR16e variants |
| `03_kuka_preset_planning.gh` | KR 6 R900 |
| `04_export_json_csv.gh` | JSON, CSV, joint lists |
| `05_external_ur_rtde.gh` | Optional — needs UR.RTDE.Grasshopper |
| `06_external_virtualrobot.gh` | Optional — needs VirtualRobot |

## 01 — Basic joint planning

```
Motus UR Preset (UR5e) → Motus Robot Model
Panel (6 zeros) → Motus Joint State → Start
Panel (6×0.5 rad) → Motus Joint State → Goal
Run toggle → Motus Plan Joint Path ← Robot, Start, Goal
→ Motus Validate Trajectory, Motus Trajectory Info
→ Motus Preview Robot, Motus Preview TCP Path
```

## 02 — UR preset planning

Same as 01; swap preset model names (UR3e, UR10e, UR16e, UR20, UR30).

## 03 — KUKA preset planning

```
Motus KUKA Preset (KR 6 R900) → Motus Robot Model → …
```

## 04 — Export JSON / CSV

```
Motus Plan Joint Path → Trajectory
→ Motus Trajectory to JSON / CSV / Joint Lists
→ Panel / Stream Filter for file write
```

## 05 — External: UR.RTDE.Grasshopper (optional)

Requires UR.RTDE.Grasshopper installed separately.

## 06 — External: VirtualRobot (optional)

Requires VirtualRobot installed separately.

## Saving

File → Save As → `examples/0N_....gh` after validating the graph in Rhino.
