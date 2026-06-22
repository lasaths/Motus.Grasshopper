# External Plugin Workflows

Motus is intentionally independent. It outputs neutral trajectories and exports that you wire manually into other tools.

## UR.RTDE.Grasshopper

1. Plan with Motus (**Motus Plan Joint Path**)
2. Export joint lists or JSON (**Motus Trajectory to Joint Lists** / **JSON**)
3. In UR.RTDE.Grasshopper, map Motus joint arrays to your RTDE move/joint components
4. Verify speeds, blends, and safety on the real controller before running

Motus does not reference UR.RTDE.Grasshopper at build time.

## VirtualRobot / Robots

1. Use Motus presets only as a planning reference (limits may differ from plugin models)
2. Export planes or joint lists from Motus
3. Feed into VirtualRobot/Robots trajectory or posture components as supported by those plugins
4. Use Motus preview for quick checks; use VirtualRobot/Robots for detailed visualization if needed

## CSV / JSON / scripting

- **JSON** — structured trajectory with times and joint arrays in radians
- **CSV** — `time_seconds,j1,j2,...` for spreadsheets or Python scripts
- **Joint Lists** — Grasshopper trees for parametric downstream graphs

## Example files

See [examples/README.md](../examples/README.md) for intended canvas layouts. Example `.gh` files may require external plugins for workflows 05–06 only.
