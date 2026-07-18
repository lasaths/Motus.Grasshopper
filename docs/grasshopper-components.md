# Grasshopper Components

All components live under the **Motus** tab. The palette stays small: pick a robot, give a target, plan, preview, export.

## Model

| Component | Inputs | Outputs |
|-----------|--------|---------|
| Motus UR10e Robotiq | *(none)* | Bundled UR10e + Robotiq 2F-85 robot |
| Motus Robot | Path to `.urdf` / `.xacro`; optional BaseLink / TipLink; optional Base plane; optional **Tool** | Robot model with URDF kinematics chain |
| Motus Tool | Name, TCP plane (flange frame), optional gripper Mesh/Brep | Tool definition |
| Motus Tool State | Optional Tool; Preset (Open/Closed/Custom); Width, Speed, Force | End-effector state (`EndEffectorStateGoo`) |
| Motus Load Mesh | Path to `.stl`, optional plane | Triangle mesh (wire to Motus Tool `Geometry`) |
| Motus Joint State | Joint list (right-click **J** input → toggle °) | Joint state |
| Motus TCP Pose | Robot, Joint state | TCP plane (FK position + orientation in base frame) |

`Motus UR10e Robotiq` is the zero-config bundled robot (`resources/robots/ur10e_robotiq/`). It previews at the UR10e home pose on placement.

`Motus Robot` loads any serial-chain URDF via `UrdfRobotLoader`. Optional `Base` overrides the robot base frame; optional `Tool` overrides the end-effector. Previews at home when the path resolves (UR10e heuristic or zeros).

`Motus Tool` defines the end-effector **TCP** in the flange frame (Z = tool axis, matching KUKA|prc / Robots conventions). Optional `Geometry` is collision + preview volume in TCP-local coordinates. Tools named `robotiq_*` auto-attach `ToolCapabilities` (width, speed, force). Box/sphere tools use the fast collision path; **mesh** tool geometry disables native FCL and falls back to the mesh checker. UR presets with non-zero TCP use numerical IK (analytic IK requires flange-equivalent tool).

`Motus Tool State` builds an `EndEffectorState` for motion program segments. Wire **Preset** Open/Closed for Robotiq jaw width, or Custom + **Width**. Optional **Tool** validates parameter names against the tool schema.

`Motus Load URDF` was removed; use **Motus Robot** instead.

`Motus Joint State` expects joint values in **URDF chain order** when the robot has `JointNames` metadata (bundled UR presets and URDF loads).

`Motus TCP Pose` runs forward kinematics for a joint state and outputs the TCP as a **Plane** in the robot base frame. Wire it before **Motus Plan** or **Motus Move** (LIN/CIRC) when you have joint targets but need a Cartesian goal.

### Joint order

| Source | Order |
|--------|--------|
| Bundled UR presets | `shoulder_pan` → `wrist_3` (see preset `jointNames`) |
| URDF load | Actuated joints along the chain from `BaseLink` to `TipLink` |
| Joint list without robot wired | Positional only — wire `Robot` to catch count mismatches |

### Home pose

`Motus Plan` optional `Start` accepts a **Plane** (IK → joints) or **Joint State**. Unwired, it defaults to UR10e home (hardcoded) when the robot matches UR10e, otherwise all zeros.

## Plan

| Component | Notes |
|-----------|-------|
| Motus Plan (nick **Quick**) | Quick single/multi-goal planner. Plane = TCP LIN; joint = joint-linear or RRT with collision. Click **Plan**, or **Auto Plan** from the right-click menu. |
| Motus RRT Settings | Tune sampling planners (`MaxIter`, `TimeLimit`, `Planner`, `GoalBias`, `Step`) → wire `Settings` to **Motus Plan** `RrtSettings`. Planner dropdown lists algorithms from `SamplingPlannerRegistry.ListAvailable()` (stub builds show managed RRT-Connect only; full native adds RRT*, AORRTC, etc.). See [AGENTS.md](../AGENTS.md). |
| Motus Move | One PTP/LIN/CIRC/SET/WAIT program line. Type (± ToolMode) are Arup-style on-component dropdowns; pins morph by type. |
| Motus Program | Plan a Motus Move list via `IndustrialMotionPlanner` (click **Plan**; wire order = program order). |
| Motus Planning Group | Build or forward a planning group (manual joints or SRDF-derived). |
| Motus Attach Body | Build an attached body from a collision object in TCP-local frame. |

`Motus Plan` inputs:

- `Robot`
- `Goal` (**list** of **Plane** and/or **Joint State** — visited in order; each segment starts from the previous end pose)
- optional `Start`
- optional `Step` (m, plane goals only; default 0.005 — TCP LIN discretization)
- optional `Collision` (scene — **required** for obstacle-aware planning; without it, red obstacle previews are display-only)
- optional `Group` (`PlanningGroup`)
- optional `Attach` (list of attached bodies)
- optional `RrtSettings` (**Motus RRT Settings** output — joint goals + collision only)

- Each `Goal` item is either a **Plane** (Cartesian TCP LIN) or a **Joint State**. Wire multiple sources into the list input (or use **Merge**) to chain waypoints; see `examples/01_quick_plan.ghx`.
- `Start` is optional (Plane or Joint State); unwired it uses the viewer home pose or zeros.
- `Step` applies only to plane goals. Long TCP moves auto-scale step size (max ~150 waypoints) so planning stays bounded.
- `Group` applies `PlanningContext.ForGroup(...)` so non-group joints stay locked.
- `Attach` applies `PlanningContext.Attach(...)` so grasped geometry participates in collision checks.
- The planner is inferred from the inputs:
  - `Goal` is a plane → workspace check, goal IK, then Cartesian LIN (TCP straight line, IK per step, retimed duration in seconds). Falls back to joint-space path if LIN fails but goal IK succeeded.
  - `Goal` is joints + `Collision` wired → sampling planner via **Motus RRT Settings** → `Plan.RrtSettings` (default RRT-Connect).
  - Optional **Motus RRT Settings** → `RrtSettings`: `MaxIter` (default 4000), `TimeLimit` (s, 0 = none), `Planner` (registry `ShortName`, e.g. `RrtConnect`; unavailable planners hidden), `GoalBias` (0–1), `Step` (rad). Ignored for plane goals and free-space joint goals.
  - `Goal` is joints, no collision → joint-linear plan.
- Plane goal **Status** errors distinguish: outside reach, goal IK failed, or LIN path failed at intermediate poses.
- Plane goals run a **workspace + IK reachability check on every solve** (no Plan click). Unreachable targets set Status/errors immediately and clear any cached trajectory.
- Trajectory output preserves robot chain and frame overrides for preview/export.
- **Manual mode (default):** toggling inputs does not replan; press **Plan** again. Unreachable plane goals still report immediately.
- **Auto Plan** (right-click menu): replans when inputs change, debounced ~400 ms. Button shows **Replan** (amber) and skips debounce when clicked. Locked components never auto-replan. Status suffixes: `(auto)`, `(auto, cached)`, or `Planning…`. A remark appears while stale trajectories are still on the output.
- `Status` reports success, errors, or validation warnings.
- `Warnings` includes runtime capability text from `MotusCapabilities.Describe()` (managed/native OMPL/FCL status).
- When `Collision` is unwired, a component remark notes that obstacle previews are visual only.
- Plane goals with collision wired validate **link envelopes** along the LIN path; the TCP polyline may still pass through obstacles that do not intersect link geometry.

### Troubleshooting: TCP path through a sphere

The white **TCP Path** in **Motus Preview** is an FK polyline between trajectory waypoints. It is not a collision-safe Cartesian sweep, and Preview does not flag obstacle hits unless you wire the same **ColScene** into Preview **Collision** (orange segments).

| Symptom | Likely cause | Fix |
|---------|--------------|-----|
| Red sphere visible, plan ignores it | `ColScene` not wired to **Motus Plan** `Collision` | `ColSphere` → `ColScene` → `Plan.Collision` |
| Plane goal, Status Success, TCP pierces sphere | LIN validates **link capsules**, not the TCP point | Expected for plane goals; use **Joint State** goals + `Collision` for RRT avoidance (`examples/02_collision_srdf.ghx`) |
| Warning: joint-space fallback | LIN failed; path is not a straight TCP line | Use a nearer `Start`, a **Joint State** goal, or accept the joint-linear path |
| Joint goal, no avoidance | No collision scene on `Plan` | Wire `ColScene`; without it joint goals use joint-linear interpolation only |

**Goal type vs collision behavior**

| Goal | `Collision` wired | Planner behavior |
|------|-------------------|------------------|
| Plane | Yes | TCP-linear (LIN) + validate against obstacles (no rerouting) |
| Plane | No | TCP-linear in free space |
| Joint State | Yes | RRT-Connect (tries to avoid obstacles) |
| Joint State | No | Joint-linear interpolation |

Wire **Motus Preview** `Collision` to the same scene to highlight TCP segments that fail link-envelope checks (orange viewport lines). Red `Invalid` output remains joint/velocity/acceleration limits only.

### Motion programs (0.6)

| Component | GUID |
|-----------|------|
| Motus RRT Settings | `11d59b15-ffe2-488e-83b8-52eddf772025` |
| Motus Move | `7c4e9a2f-1b3d-4e8a-9f6c-2d8b5a7e9c31` |
| Motus Program | `8d5f0b3e-2c4e-4f9b-0a7d-3e9c6b8f0d42` |

`Motus Move` uses **on-component** Type (± ToolMode) dropdowns (Arup-style attributes — not a floating GH Value List). Pins morph to the active type:

| Type | Required | Optional |
|------|----------|----------|
| PTP | `Goal` (Joint State) | `Blend` (m), `ToolState` |
| LIN | `Goal` (Plane, TCP pose) | `Step` (m, default 0.005), `Blend`, `ToolState` |
| CIRC | `Via` + `Goal` (Planes) | `Samples` (default 16), `Blend`, `ToolState` |
| SET | `ToolState` | `Duration` (s ramp; 0 = instant) |
| WAIT | `Duration` (s) | — |

**ToolMode** (face dropdown on PTP/LIN/CIRC): `Hold`, `Ramp`, `Instant`. Execution hints for downstream adapters; Motus does not command hardware.

Exported trajectories include optional `toolState` per waypoint and `toolCapabilities` in JSON (see `examples/04_motion_program.ghx`).

`Motus Program` inputs match `Motus Plan` collision/group/attach semantics. Tool state on moves is validated against the robot's wired **Tool** capabilities when present.

`Motus Preview` outputs optional **ToolState** and **Width** at the playhead. Gripper mesh preview morphs with jaw width when tool capabilities are present.

**Motus Export** JSON includes `contractVersion`, `diagnostics`, optional `provenance`, and tool metadata (`toolState`, `toolCapabilities`) for downstream consumers.

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

| Component / parameter | Notes |
|-----------------------|-------|
| Motus Preview | Animated FK preview with a built-in **Play / Stop** button; right-click for **Override / URDF / Custom** mesh colours |
| Motus Scrub | Resizable **0–1** canvas slider; wire to Preview **Position** for manual scrubbing |

`Motus Preview` takes a `Trajectory`, optional `ShowStart`, optional **Position** (0–1), and optional **Collision** (same `ColScene` as Plan). Right-click to choose **Override**, **URDF**, or **Custom** viewport mesh colours; expose the hidden **Custom Colours** list input from the menu when using Custom mode (one colour per **Meshes** slot). It outputs link `Meshes` and `Links` at the current playback frame, the full `TCP Path` polyline (FK between waypoints — not a collision-safe sweep), the `State` / `Time` / `Index` at the playhead, and `Invalid` TCP segments (joint/velocity/acceleration limits only). When **Collision** is wired, obstacle hits along the TCP polyline draw in **orange** in the viewport.

`Motus Scrub` is a floating parameter (no inputs) with a single numeric output locked to 0–1. Resize the control horizontally for finer scrub precision on long trajectories. Dragging scrubs preview-only until release; manual scrub **pauses** Play. During Play, the scrub thumb syncs to the current position.

Playback interpolates joint angles between waypoints by elapsed time via `AtTime` (not discrete index stepping). FK uses `KinematicsResolver` (DH presets or URDF chain). Base and tool frames come from the trajectory context (preset or robot overrides).

For URDF robots, preview shows mesh visuals (`.stl` / `.dae`) loaded from the URDF folder. Preset capsule collision is used for planning only, not drawn in the viewport.

## Export

| Component | Output |
|-----------|--------|
| Motus Waypoints | Controller-oriented trees: `Joints` (`Q`) as `{waypoint → q}`, TCP `Planes`, `Times` (default GH plane fans on `P` hidden; path viz on Motus Preview) |
| Motus Export | `Json` and `Csv` strings |

**Motus Waypoints** reshapes a planned trajectory for live controllers (e.g. UR Write). It does not connect to or command robots.

- `Q` — data tree, **one branch per waypoint**, `AxisCount` joint values (radians). Wire to joint MoveJ-style inputs.
- `P` — TCP planes via FK (same length as `Q` after decimate).
- `Tm` — waypoint times (seconds); metadata for downstream graphs.
- `D` (Decimate) — keep every Nth waypoint; **always keeps first and last**. Default `1` = all points.

Dense Motus paths executed as discrete MoveJ segments are stop-and-go; use Decimate to thin. Prefer `Q` → joint moves for planned path fidelity. Use `P` → linear TCP moves only for Cartesian-intent (LIN) paths — FK planes from joint-space / RRT trajectories are not a safe MoveL path (TCP re-interpolation can diverge). Warns when `AxisCount ≠ 6`. Controller handoff notes: [AGENTS.md](../AGENTS.md).

JSON export includes `jointNames` when the robot model provides them. Point count is the length of `Times`; duration is the last `Times` value (native Grasshopper list ops).

## Units

Joint inputs: **radians** by default. Right-click the **J** input on **Motus Joint State** and toggle **Degrees** for degree input (persisted in the `.gh`/`.ghx`). Planes and preview geometry use **meters**.
