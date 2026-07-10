# Preview

Motus uses **DH forward kinematics** for Rhino viewport preview when the robot preset has a kinematics profile (all bundled UR/KUKA presets). URDF-loaded robots use chain FK via `KinematicsResolver`.

## Motus Preview

A single component handles all preview, with a built-in **Play / Stop** button for timed playback. Inputs: a `Trajectory`. Outputs:

| Output | Geometry |
|--------|----------|
| Meshes | URDF visual geometry when available; otherwise simplified link meshes |
| Links | FK link lines at the current frame |
| TCP Path | TCP polyline along the whole trajectory |
| State / Time / Index | Joint state, elapsed time, waypoint index at the playhead |
| Invalid | TCP segments that fail joint/velocity/acceleration limits |

## Playback

Click **Play** to animate; the frame outputs (`Meshes`, `Links`, `State`, `Time`, `Index`) follow the playhead. Click **Stop** to pause. When stopped, the outputs reflect the current waypoint index (start by default).

## Frames and units

- Base and tool frames come from the robot preset.
- Keep Rhino document units in **meters**.
- For production visualization, export trajectory data to VirtualRobot or Robots.

## URDF visual geometry

- `Motus Load URDF` forwards URDF `<visual>` geometry to preview.
- Supported visual types: `box`, `cylinder`, `sphere`, and mesh `.stl`.
- Unsupported visual mesh formats (for example `.dae`) are skipped and preview falls back to simplified FK geometry.
