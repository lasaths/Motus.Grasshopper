# Grasshopper Components

All components live under the **Motus** tab. The palette is intentionally small: pick a robot, give a target, plan, preview, export.

## Model

| Component | Inputs | Outputs |
|-----------|--------|---------|
| Motus Robot | Model name (dropdown) or JsonPath; optional Base / Tool planes | Robot model |
| Motus Load URDF | Path; optional BaseLink / TipLink | Robot model with URDF kinematics chain |
| Motus Joint State | Joint list, UseDegrees; optional Robot for count validation | Joint state |

`Motus Robot` loads a bundled preset by name (dropdown attached automatically) or, when `JsonPath` is wired, from a JSON file. Optional `Base` and `Tool` planes override the preset frames for FK, planning, and preview (plane goals are **TCP** targets in the base frame).

`Motus Load URDF` loads a serial-chain URDF via `UrdfRobotLoader`. The output carries the URDF joint chain and joint names for FK/planning parity with the web viewer.

`Motus Joint State` expects joint values in **URDF chain order** when the robot has `JointNames` metadata (bundled UR presets and URDF loads). Wire the optional `Robot` input to validate axis count.

### Joint order

| Source | Order |
|--------|--------|
| Bundled UR presets | `shoulder_pan` → `wrist_3` (see preset `jointNames`) |
| URDF load | Actuated joints along the chain from `BaseLink` to `TipLink` |
| Joint list without robot wired | Positional only — wire `Robot` to catch count mismatches |

### Home pose

`Motus Plan` optional `Start` defaults to home from `resources/viewer_presets.json` (keyed by model name), then all zeros. Same poses as the URDF web viewer.

## Plan

| Component | Notes |
|-----------|-------|
| Motus Plan | One planner for all goals. Click the **Plan** button to compute. |

`Motus Plan` inputs: `Robot`, `Goal`, optional `Start`, optional `Collision`.

- `Goal` is either a **Plane** (Cartesian TCP LIN) or a **Joint State**.
- `Start` is optional; unwired it uses the viewer home pose or zeros.
- The planner is inferred from the inputs:
  - `Goal` is a plane → Cartesian linear plan (TCP straight line, IK per step, retimed duration in seconds).
  - `Goal` is joints + `Collision` wired → RRT-Connect.
  - `Goal` is joints, no collision → joint-linear plan.
- Trajectory output preserves robot chain and frame overrides for preview/export.
- `Status` reports success, errors, or validation warnings. Toggling inputs does not replan; press **Plan** again.

## Collision

| Component | Notes |
|-----------|-------|
| Motus Collision Sphere | Center point + radius (m) |
| Motus Collision Box | Plane + half extents (m) |
| Motus Collision Mesh | Mesh or Brep obstacle (meters); plane bakes world pose into vertices |
| Motus Collision Scene | Merge collision objects; optional **Srdf** path for allowed pairs (`link:N` or obstacle names) |

## Preview

| Component | Notes |
|-----------|-------|
| Motus Preview | Animated FK preview with a built-in **Play / Stop** button |

`Motus Preview` takes a `Trajectory` and outputs link `Meshes` and `Links` at the current playback frame, the full `TCP Path` polyline, the `State` / `Time` / `Index` at the playhead, and `Invalid` TCP segments (joint/velocity/acceleration limits).

Playback interpolates joint angles between waypoints by elapsed time (not discrete index stepping). FK uses `KinematicsResolver` (DH presets or URDF chain). Base and tool frames come from the trajectory context (preset or robot overrides).

## Export

| Component | Output |
|-----------|--------|
| Motus Trajectory Data | TCP `Planes`, waypoint `Times`, per-axis `Joints` tree |
| Motus Export | `Json` and `Csv` strings |

JSON export includes `jointNames` when the robot model provides them. Point count is the length of `Times`; duration is the last `Times` value (native Grasshopper list ops).

## Units

Joint inputs: **radians** by default. Enable **UseDegrees** on Motus Joint State for degree input. Planes and preview geometry use **meters**.
