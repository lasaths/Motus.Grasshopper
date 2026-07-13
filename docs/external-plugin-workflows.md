# External Plugin Workflows

Motus is intentionally independent. It outputs neutral trajectories and exports that you wire manually into other tools.
Execution ownership lives in the downstream control plugin: controller transport, retries, IO timing, safety gates, and runtime fault handling.

## UR.RTDE.Grasshopper

1. Plan with **Motus Plan**
2. Get joint trees from **Motus Trajectory Data** (`Joints`) or JSON from **Motus Export**
3. In UR.RTDE.Grasshopper, map Motus joint arrays to your RTDE move/joint components
4. Verify speeds, blends, and safety on the real controller before running

Motus does not reference UR.RTDE.Grasshopper at build time.

## VirtualRobot / Robots

1. Use Motus presets only as a planning reference (limits may differ from plugin models)
2. Export planes or joint lists from Motus
3. Feed into VirtualRobot/Robots trajectory or posture components as supported by those plugins
4. Use Motus preview for quick checks; use VirtualRobot/Robots for detailed visualization if needed

## CSV / JSON / scripting

- **Json** (Motus Export) — structured trajectory with times and joint arrays in radians
- JSON includes planner metadata (`contractVersion`, `diagnostics`, optional `provenance`) and execution hints (`toolState`, `motionType`, `ToolMode` intent)
- **Csv** (Motus Export) — `time_seconds,joint_1_rad,...` for spreadsheets or Python scripts
- **Joints** (Motus Trajectory Data) — Grasshopper trees for parametric downstream graphs

## Example files

See [examples/README.md](../examples/README.md) for intended canvas layouts. Example `.gh` files may require external plugins for workflows 05–06 only.
