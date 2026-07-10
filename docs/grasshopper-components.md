# Grasshopper Components

All components live under the **Motus** tab. The palette stays small: pick a robot, give a target, plan, preview, export.

## Model

| Component | Inputs | Outputs |
|-----------|--------|---------|
| Motus Robot | Model name (dropdown) or JsonPath; optional Base plane; optional **Tool** (`Motus Tool`, overrides bundled gripper) | Robot model |
| Motus Tool | Name, TCP plane (flange frame), optional gripper Mesh/Brep | Tool definition |
| Motus Load Mesh | Path to `.stl`, optional plane | Triangle mesh (wire to Motus Tool `Geometry`) |
| Motus Load URDF | Path; optional BaseLink / TipLink | Robot model with URDF kinematics chain |
| Motus Joint State | Joint list (right-click **J** input → toggle °) | Joint state |
| Motus TCP Pose | Robot, Joint state | TCP plane (FK position + orientation in base frame) |

`Motus Robot` loads a bundled preset by name (dropdown attached automatically) or, when `JsonPath` is wired, from a JSON file. **UR10e** includes a bundled **Robotiq 2F-85** tool (TCP + mesh) unless you wire **Motus Tool** to override it. Optional `Base` overrides the robot base frame.

`Motus Tool` defines the end-effector **TCP** in the flange frame (Z = tool axis, matching KUKA|prc / Robots conventions). Optional `Geometry` is collision + preview volume in TCP-local coordinates. Box/sphere tools use the fast collision path; **mesh** tool geometry disables native FCL and falls back to the mesh checker. UR presets with non-zero TCP use numerical IK (analytic IK requires flange-equivalent tool).

`Motus Load URDF` loads a serial-chain URDF via `UrdfRobotLoader`. The output carries the URDF joint chain and joint names for FK/planning parity with the web viewer, and forwards URDF visual geometry for preview when available.

`Motus Joint State` expects joint values in **URDF chain order** when the robot has `JointNames` metadata (bundled UR presets and URDF loads).

`Motus TCP Pose` runs forward kinematics for a joint state and outputs the TCP as a **Plane** in the robot base frame. Wire it before **Motus Plan** or **Motus Motion Segment** (LIN/CIRC) when you have joint targets but need a Cartesian goal.

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
| Motus Plan | One planner for all goals. Click **Plan** to compute, or enable **Auto Plan** from the right-click menu. |
| Motus Motion Segment | Build a single PTP, LIN, or CIRC segment (`MotionSegmentGoo`). |
| Motus Program Plan | Plan a mixed segment list via `IndustrialMotionPlanner` (click **Plan**). |
| Motus Planning Group | Build or forward a planning group (manual joints or SRDF-derived). |
| Motus Attach Body | Build an attached body from a collision object in TCP-local frame. |

`Motus Plan` inputs:

- `Robot`
- `Goal` (**Plane** or **Joint State**)
- optional `Start`
- optional `Step` (m, plane goals only; default 0.005 — TCP LIN discretization)
- optional `Collision` (scene)
- optional `Group` (`PlanningGroup`)
- optional `Attach` (list of attached bodies)

- `Goal` is either a **Plane** (Cartesian TCP LIN) or a **Joint State**.
- `Start` is optional; unwired it uses the viewer home pose or zeros.
- `Step` applies only to plane goals. Long TCP moves auto-scale step size (max ~150 waypoints) so planning stays bounded.
- `Group` applies `PlanningContext.ForGroup(...)` so non-group joints stay locked.
- `Attach` applies `PlanningContext.Attach(...)` so grasped geometry participates in collision checks.
- The planner is inferred from the inputs:
  - `Goal` is a plane → workspace check, goal IK, then Cartesian LIN (TCP straight line, IK per step, retimed duration in seconds). Falls back to joint-space path if LIN fails but goal IK succeeded.
  - `Goal` is joints + `Collision` wired → RRT-Connect.
  - `Goal` is joints, no collision → joint-linear plan.
- Plane goal **Status** errors distinguish: outside reach, goal IK failed, or LIN path failed at intermediate poses.
- Trajectory output preserves robot chain and frame overrides for preview/export.
- **Manual mode (default):** toggling inputs does not replan; press **Plan** again.
- **Auto Plan** (right-click menu): replans when inputs change, debounced ~400 ms. Button shows **Replan** (amber) and skips debounce when clicked. Locked components never auto-replan. Status suffixes: `(auto)`, `(auto, cached)`, or `Planning…`. A remark appears while stale trajectories are still on the output.
- `Status` reports success, errors, or validation warnings.
- `Warnings` includes runtime capability text from `MotusCapabilities.Describe()` (managed/native OMPL/FCL status).

### Motion programs (0.6)

| Component | GUID |
|-----------|------|
| Motus Motion Segment | `7c4e9a2f-1b3d-4e8a-9f6c-2d8b5a7e9c31` |
| Motus Program Plan | `8d5f0b3e-2c4e-4f9b-0a7d-3e9c6b8f0d42` |

`Motus Motion Segment` has a **Type** dropdown (`PTP` / `LIN` / `CIRC`). All inputs stay visible; only the active type is validated:

| Type | Required | Optional |
|------|----------|----------|
| PTP | `Goal` (Joint State) | `Robot` (joint count), `Blend` (m) |
| LIN | `Goal` (Plane, TCP pose) | `Step` (m, default 0.005), `Blend` |
| CIRC | `Via` + `Goal` (Planes) | `Samples` (default 16), `Blend` |

`Motus Program Plan` inputs match `Motus Plan` collision/group/attach semantics:

- `Robot`, `Segments` (list of `MotionSegmentGoo`), optional `Start`
- optional `Collision`, `Group`, `Attach`

Unlike `Motus Plan` plane goals, **LIN failures do not fall back to joint-space paths** — errors surface in `Status`.

Exported trajectories include `motionType`, `segmentIndex`, and `blendRadiusMeters` per waypoint (see `examples/08_motion_program.ghx`).

## Collision

| Component | Notes |
|-----------|-------|
| Motus Collision Sphere | Center point + radius (m) |
| Motus Collision Box | Plane + half extents (m) |
| Motus Collision Mesh | Mesh or Brep obstacle (meters); plane bakes world pose into vertices |
| Motus Collision Scene | Merge collision objects; optional **Srdf** path for allowed pairs (`link:N` or obstacle names). Outputs scene plus optional SRDF groups/end-effector map. |

`Motus Collision Scene` outputs:

- `Scene` collision scene
- `Groups` SRDF planning groups (when an SRDF is provided and parsed)
- `EndEffectors` `name=parent_link` entries from SRDF

## Preview

| Component | Notes |
|-----------|-------|
| Motus Preview | Animated FK preview with a built-in **Play / Stop** button |

`Motus Preview` takes a `Trajectory` and outputs link `Meshes` and `Links` at the current playback frame, the full `TCP Path` polyline, the `State` / `Time` / `Index` at the playhead, and `Invalid` TCP segments (joint/velocity/acceleration limits).

Playback interpolates joint angles between waypoints by elapsed time (not discrete index stepping). FK uses `KinematicsResolver` (DH presets or URDF chain). Base and tool frames come from the trajectory context (preset or robot overrides).

For URDF robots, preview shows mesh visuals (`.stl` / `.dae`) loaded from the URDF folder. Preset capsule collision is used for planning only, not drawn in the viewport.

## Export

| Component | Output |
|-----------|--------|
| Motus Trajectory Data | TCP `Planes`, waypoint `Times`, per-axis `Joints` tree |
| Motus Export | `Json` and `Csv` strings |

JSON export includes `jointNames` when the robot model provides them. Point count is the length of `Times`; duration is the last `Times` value (native Grasshopper list ops).

## Units

Joint inputs: **radians** by default. Right-click the **J** input on **Motus Joint State** and toggle **Degrees** for degree input (persisted in the `.gh`/`.ghx`). Planes and preview geometry use **meters**.
