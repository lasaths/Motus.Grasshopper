# Example Grasshopper Definitions

Save these canvases from Rhino 8 after building the component chains below. Binary `.gh` files are not checked in yet — recreate them locally using this guide.

## 01 — Basic joint planning

```
Motus UR Preset (UR5e) → Motus Robot Model
Panel (6 zeros) → Motus Joint State → Start
Panel (6×0.5 rad) → Motus Joint State → Goal
Run toggle → Motus Plan Joint Path ← Robot, Start, Goal
→ Motus Validate Trajectory, Motus Trajectory Info
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
→ Motus Trajectory to JSON
→ Motus Trajectory to CSV
→ Motus Trajectory to Joint Lists
→ Panel / Stream Filter for file write
```

## 05 — External: UR.RTDE.Grasshopper (optional plugin)

Requires UR.RTDE.Grasshopper installed separately.

```
Motus trajectory joint lists → manual wiring to UR RTDE joint/move components
Document blend/speed overrides in your RTDE graph
```

## 06 — External: VirtualRobot (optional plugin)

Requires VirtualRobot installed separately.

```
Motus Trajectory to Joint Lists or Planes → VirtualRobot posture/trajectory inputs
Use Motus preview for quick checks; VirtualRobot for detailed model
```

## Saving

File → Save As → `examples/0N_....gh` in this repository after validating the graph.
