# Preview

Motus uses **DH forward kinematics** for Rhino viewport preview when the robot preset has a kinematics profile (all bundled UR/KUKA presets).

## What is shown

| Component | Geometry |
|-----------|----------|
| Motus Preview Robot | FK link lines, cylinder link meshes, TCP plane (tool frame) |
| Motus Preview TCP Path | TCP polyline along trajectory |
| Motus Preview Trajectory | Start/goal TCP points, valid/invalid TCP segments, goal link lines |

## Invalid segments

Wire a **Motus Collision Scene** into **Motus Preview Trajectory** (and enable acceleration checks) to split TCP motion into **Valid** vs **Invalid** line outputs. Color them downstream in Grasshopper.

## Tips

- Optional **Base** / **Tool** inputs override preset frames on preview and export components.
- Keep Rhino document units in **meters**.
- For production visualization, export trajectory data to VirtualRobot or Robots.
