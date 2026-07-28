# Grasshopper Components

All components live under the **Motus** tab. Motus.Grasshopper is **thin wiring** — solvers, units, and provenance live in [Motus.NET](https://github.com/lasaths/Motus.NET).

## At a glance

| You want… | Use |
|-----------|-----|
| Bundled UR10e + Robotiq | **Motus UR10e Robotiq** |
| Load a URDF / xacro | **Motus Robot** |
| Parametric serial arm / rail | **Motus Serial Chain** |
| Stewart platform (leg lengths in **m**) | **Motus Stewart** → Plan TCP planes |
| Walking / N-leg gait (radians) | **Motus Body** + **Motus Leg** → **Motus Mechanism** → **Motus Walk** (Path/Planes/`Tn` → `Tr`); tip-path from Walk `Rb` without Path |
| Branched tree / tip-path Plan | **Motus Joint Table** (optional **SE2** mobility) |
| Plan one or more goals | **Motus Plan** (nick **Quick**) |
| Obstacles | **Col\*** → **ColScene** → Plan `Collision` |
| Animate | **Motus Preview** (+ optional **Scrub**) |
| Controller handoff | **Motus Waypoints** `Q` (joint MoveJ). JSON/CSV → **Motus Export** |
| PTP/LIN/CIRC program | **Motus Move** → **Motus Program** |
| Author URDF in GH | **Urdf Link / Joint / Assemble / Attach** → optional **Export URDF** |

```
Model ──► Plan ──► Preview
   │         │         │
 Tool    ColScene   Waypoints / Export
```

**Units:** joints default **radians** (toggle ° on Joint State `J`); geometry **meters**. Stewart plan `Q` = meters; serial/legged = radians. See [AGENTS.md](../AGENTS.md) for Family handoff.

## Model

| Component | Inputs | Outputs |
|-----------|--------|---------|
| Motus UR10e Robotiq | *(none)* | Bundled UR10e + Robotiq 2F-85 robot |
| Motus Robot | Path to `.urdf` / `.xacro`; optional BaseLink / TipLink; optional Base plane; optional **Tool** | Robot model with URDF kinematics chain |
| Motus Serial Chain | **Lengths** list (m); optional Base, Home `Q`, **Rail**, Types, TCP | Same Robot goo — parametric serial / rail+arm (concept sizing) |
| Motus Stewart | Optional JSON Path; **Base** / **Plat** (6 points each); classic BaseRadius / PlatformRadius / MinStroke / MaxStroke / PairSep / Name | Same Robot goo — Stewart/Gough hexapod (`Family=stewart`; `Q` = leg lengths in **meters**) |
| Motus Leg | Lengths (m), optional Name / Tip | `Leg` goo — 3R → LegIk3R; longer → numerical IK |
| Motus Body | N / BodyR / BodyZ or custom hip Planes | `Bdy` hip frames → Mechanism |
| Motus Mechanism | Bdy + Leg (clone) or Leg list; Dyn; Tip; stance | `Mech` → Walk (auto `GaitSchedule.Auto`) |
| Motus Body Pose | Mode PathFollow \| TerrainSupport; Clearance | Optional `Pose` → Walk |
| Motus Walk | Required `Mech`, optional Pose, Path/Planes, Speed, Step, Lift, Terrain | Gait `Tr` (full drivers), Robot, Meshes, Support — **not** Stewart |
| Motus Terrain Patch | Origin, Size, Amp | Outdoor heightfield mesh → Walk `Tn` |
| Motus Joint Table | Parent / Child / Type / Ox; optional Oy,Oz, Name, **Tip**, Base, Home, **SE2** (X,Y,yaw) | Same Robot goo — Plan uses **tip path** only; side branches are TreeFK preview only |
| Motus Reach Samples | Robot; optional Count (≤512), Seed | TCP sample points for reach overlay (no building pin) |
| Motus Tool | Name, TCP; **Cap** face dropdown (`None` \| `Robotiq2F85` schema); optional G/L and/or Rd+Bd | Tool definition |
| Motus Tool State | Optional Tool; **Preset** face dropdown; Width (used when Custom); Speed, Force | End-effector state (`EndEffectorStateGoo`) |
| Motus Load Mesh | Path to `.stl`, optional plane | Triangle mesh (wire to Motus Tool `Geometry`) |
| Motus Joint State | Joint list (right-click **J** input → toggle °) | Joint state |
| Motus TCP Pose | Robot, Joint state | TCP plane (FK position + orientation in base frame) |

`Motus UR10e Robotiq` is the zero-config bundled robot (`resources/robots/ur10e_robotiq/`). It previews at the UR10e home pose on placement.

`Motus Robot` loads any serial-chain URDF via `UrdfRobotLoader`. Optional `Base` overrides the robot base frame; optional `Tool` overrides the end-effector. Optional **AllDrivers** promotes tip-path + side-branch drivers into Plan/Joint State DOF (e.g. DKP beside the arm; tip-descendant tool knuckles stay off Plan). Optional **Attach** (Box/Mesh/Brep/…) / **AttachLink** / **AttachOrigin** grafts fixture geometry onto a named parent link (TreeFK + preview) — e.g. a GH Center Box on `turntable_link`. Previews at home when the path resolves (UR10e heuristic or zeros). Right-click **Show TCP** draws the home TCP triad; **Preview collision meshes** shows planning hulls.

**Motus Stewart** builds a Stewart/Gough platform (`Family=stewart`) via Motus.NET. Priority: JSON Path → **Base**+**Plat** (exactly 6 Rhino points each, meters) → classic hex (`Br`/`Pr`/`Sep`). Wire TCP plane goals into **Motus Plan** — IK yields six **leg lengths in meters**. Do **not** hand Stewart `Q` to UR MoveJ. See [ADR 0003](adr/0003-parallel-kinematics-stewart.md). Stewart TCP-LIN now passes the wired collision scene/checker into Motus.NET; collided LIN paths report collision, and when goal IK succeeds Plan can fall back to RRT in leg-length space (not a straight TCP platform path). TCP planes use plate mapping (`FrameConversion.FromPlanePlate`) — Rhino Z is platform normal, not serial tool-approach. Example `08_stewart_tcp_path.ghx` exposes Br/Pr/Lmin/Lmax sliders like Body N on the walking example.

**Motus Body** + **Motus Leg** + **Motus Mechanism** → **Motus Walk** cover N-leg walkers (`Family=legged`, joint `Q` in **radians**) — **not** Stewart. Assemble hips + leg recipe into `Mech`, then Path/Planes + optional Terrain (`Tn` / Motus Terrain Patch) → foot-target gait `Tr` → Preview / Export / Waypoints. Walk runs `LeggedGait.ValidateForPlan` as a Status gate: SSM dips on hills/odd-N stay named **warnings** (trajectory still emits); other validation failures remain errors. Optional **Motus Body Pose** overrides Auto (TerrainSupport if `Tn`, else PathFollow). Motus Hex was removed in 0.13 — see [ADR 0005](adr/0005-general-legged-mechanism.md) and [CHANGELOG](../CHANGELOG.md). Example: `09_walking_hexapod.ghx`.

`Motus Tool` defines the end-effector **TCP** in the flange frame (Z = tool axis, matching KUKA|prc / Robots conventions). **Cap** is an on-component dropdown (`None` \| `Robotiq2F85`) for the **parameter schema** used by Tool State / export (`width` m, `speed`/`force` ratio) — not ToolMode, not mesh choice, not bindings. `None` means no schema (no name-based auto-Cap). Pins stay stable (G/L/Rd/Bd always present, optional) so example wires survive Cap changes; Rd+G both wired → warning, ignore G.

Optional `Geometry` is collision + preview volume in TCP-local coordinates (legacy static tool — jaw-width squash when Bindings empty). Box/sphere tools use the fast collision path; **mesh** tool geometry disables native FCL and falls back to the mesh checker. UR presets with non-zero TCP use numerical IK (analytic IK requires flange-equivalent tool).

Optionally wire a **Description** (`RobotDescription`, e.g. from **Motus Urdf Assemble**) for an *actuated* mechanism instead of a static mesh: **Motus Robot** grafts it onto the arm's kinematic tree at the tip link (`KinematicTree.Attach`, rotation-aware) so TreeFK drives real mechanism links, not a squashed mesh. Leave **TCP** unwired to derive it from the mechanism's `TipTcp()`. **Binding** names the driver joint that Cap's `width` parameter maps to (defaults to `robotiq_left_knuckle` when Cap = Robotiq2F85 and Binding is unwired). This is the fast path when the arm itself is a URDF/xacro load (**Motus Robot**); for composing an arm *and* mechanism from scratch, the Urdf authoring family's **Motus Urdf Attach** + `RobotDescriptionSession.Project` below remains the structural, from-scratch route.

`Motus Tool State` builds an `EndEffectorState` for motion program segments. **Preset** (Open/Closed/Custom) is on-component; **Width** is used when Preset=Custom. Wire **Tool** (or Robot with bundled Cap) — a wired Tool/Robot with Cap=None errors (no silent Robotiq invent). Unwired Tool State warns and assumes Robotiq for zero-config demos. Cap ≠ Motus Move **ToolMode** (Hold/Ramp/Instant export timing).

`Motus Load URDF` was removed; use **Motus Robot** instead.

### Urdf authoring (Link / Joint / Assemble / Explode / Attach)

Use native Grasshopper geometry (e.g. **Center Box**, Mesh, Brep) into **Motus Urdf Link** — there is no Motus geom component.

| Component | Inputs | Outputs |
|-----------|--------|---------|
| Motus Urdf Link | Name; Visual (Box/Mesh/Brep list); optional Collision list | `UrdfLink` |
| Motus Urdf Joint | Name; Type (Revolute/Continuous/Prismatic/Fixed or R/C/P/F); Parent/Child link names; Axis (Line: Start = origin, direction = joint axis); optional Lower/Upper, MimicJoint/Mult/Offset | `UrdfJoint` |
| Motus Urdf Assemble | Name; Links list; Joints list; optional Tip | `RobotDescription` (validated tree; debounced ~120 ms) |
| Motus Urdf Explode | Description | Links list, Joints list |
| Motus Urdf Attach | Parent/Child `RobotDescription`; ParentLink; optional Plane (origin only), JointName | Merged `RobotDescription` |

This is a **typed** authoring path, not a return to bare-number Link×N/Joint×N spaghetti: every
node is a validated Motus.NET goo (`UrdfLinkGoo` → `UrdfJointGoo` → `RobotDescriptionGoo`), and
**Motus.NET owns assemble/attach** (`RobotDescription.TryAssemble` / `.Attach` / `.Explode`) —
Grasshopper only collects per-node inputs and hands them to Motus.NET. See
[ADR 0002](adr/0002-kinematic-tree-in-motus-net.md) for the policy and rationale.

Use this family to author a driven mechanism (gripper, turntable, rail) with **no URDF file on
disk** — e.g. **Motus Urdf Assemble** the tool mechanism, **Motus Urdf Attach** it onto the arm's
description at a parent link, then project to a `KinematicTree` (`RobotDescriptionSession.Project`)
for FK/planning the same way a URDF load or **Motus Serial Chain** would. **Motus Urdf Attach**'s
`Plane` carries origin only — rotate the child's own links/axes for a tilted mount, not the attach
frame.

This is a different, structural role from **Motus Tool** above, which stays a **thin** wrapper
around a TCP frame plus optional collision geometry or an attached mechanism — it never assembles
or validates a kinematic tree itself. When the arm is authored from scratch (not a URDF/xacro file),
wire a driven gripper's actuated fingers through this Urdf authoring family; when the arm is a
URDF/xacro load (**Motus Robot**), wiring the mechanism straight into **Motus Tool**'s `Description`
pin is simpler and grafts onto the loaded tree directly (see above).

`Motus Joint Table` `BaseSE2` stores a `HolonomicSE2` mobile-base goal and also uses the same pose as the preview/base-frame override. Joint goals with `BaseSE2` route through Motus.NET sampling with `PlanningOptions.Mobility`; without `BaseSE2`, Joint Table remains a fixed-base tip-path planner.

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
| Motus RRT Settings | Tune sampling planners (`MaxIter`, `TimeLimit`, `Planner`, `GoalBias`, `Step`) → wire `Settings` to **Motus Plan** `RrtSettings`. `Step` is a config-space step: radians for serial/legged joints, meters for `Family=stewart` leg lengths. Planner dropdown lists algorithms from `SamplingPlannerRegistry.ListAvailable()` (stub builds show managed RRT-Connect only; full native adds RRT*, AORRTC, etc.). See [AGENTS.md](../AGENTS.md). |
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
  - `Goal` is a plane → workspace check, goal IK, then Cartesian LIN (TCP straight line, IK per step, retimed duration in seconds). If LIN fails on collision with a wired scene → IK goal + RRT (uses Motus RRT Settings when present). Stewart plane goals pass collision options into Motus.NET and may RRT in leg-length meters after a collided LIN.
  - `Goal` is joints + `Collision` wired, or Joint Table has `BaseSE2` mobility → sampling planner via **Motus RRT Settings** → `Plan.RrtSettings` (default RRT-Connect).
  - Optional **Motus RRT Settings** → `RrtSettings`: `MaxIter` (default 4000), `TimeLimit` (s, 0 = none), `Planner` (registry `ShortName`, e.g. `RrtConnect`; unavailable planners hidden), `GoalBias` (0–1), `Step` (radians for serial/legged; meters for `Family=stewart`). Ignored for fixed-base free-space joint-linear plans.
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
| Plane goal, Status Success, TCP pierces sphere | LIN validates **link capsules**, not the TCP point | Expected when envelopes clear; if LIN collides Motus Plan RRTs instead (warning on Status) |
| Warning: RRT joint path / joint-space fallback | LIN blocked or failed; path is not a straight TCP line | Accept the joint path, or use a nearer `Start` / clearer corridor for LIN |
| Joint goal, no avoidance | No collision scene on `Plan` | Wire `ColScene`; without it joint goals use joint-linear interpolation only |

**Goal type vs collision behavior**

| Goal | `Collision` wired | Planner behavior |
|------|-------------------|------------------|
| Plane | Yes | TCP-linear (LIN) + validate; on collision failure → IK goal + RRT (not a straight TCP line) |
| Plane | No | TCP-linear in free space |
| Joint State | Yes | RRT-Connect (tries to avoid obstacles) |
| Joint State | No | Joint-linear interpolation |

`Motus Program` LIN segments still validate-only (no RRT reroute).

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

`Motus Preview` outputs optional **ToolState** and **Width** at the playhead. Robotiq finger meshes follow URDF/PickNik joint kinematics from jaw width (not a flattened scale).

**Motus Export** JSON includes `contractVersion`, `diagnostics`, optional `provenance`, and tool metadata (`toolState`, `toolCapabilities`) for downstream consumers.

## Collision

| Component | Notes |
|-----------|-------|
| Motus Collision Sphere | Center point + radius (m) |
| Motus Collision Box | Plane + half extents (m) |
| Motus Collision Plane | Infinite half-space floor/wall; **+Z free**. Default **Offset** 2 mm sinks the plane. Scene auto-ignores proximal `link:-1..1` vs planes (robot+floor at origin). |
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

`Motus Preview` takes a `Trajectory`, optional `ShowStart`, optional **Position** (0–1), and optional **Collision** (same `ColScene` as Plan). Right-click to choose **Override**, **URDF**, or **Custom** viewport mesh colours; expose the hidden **Custom Colours** list input from the menu when using Custom mode (one colour per **Meshes** slot); **Show TCP** draws the playhead TCP triad in the viewport. It outputs link `Meshes` and `Links` at the current playback frame, the full `TCP Path` polyline (FK between waypoints — not a collision-safe sweep), the `State` / `Time` / `Index` at the playhead, and `Invalid` TCP segments (joint/velocity/acceleration limits only). When **Collision** is wired, obstacle hits along the TCP polyline draw in **orange** in the viewport.

`Motus Scrub` is a floating parameter (no inputs) with a single numeric output locked to 0–1. Resize the control horizontally for finer scrub precision on long trajectories. Dragging scrubs preview-only until release; manual scrub **pauses** Play. During Play, the scrub thumb syncs to the current position.

Playback interpolates joint angles between waypoints by elapsed time via `AtTime` (not discrete index stepping). FK uses `KinematicsResolver` (DH presets or URDF chain). Base and tool frames come from the trajectory context (preset or robot overrides).

For URDF robots, preview shows mesh visuals (`.stl` / `.dae`) loaded from the URDF folder. Preset capsule collision is used for planning only, not drawn in the viewport.

## Export

| Component | Output |
|-----------|--------|
| Motus Waypoints | Controller-oriented trees: `Joints` (`Q`) as `{waypoint → q}`, TCP `Planes`, `Times` (default GH plane fans on `P` hidden; path viz on Motus Preview) |
| Motus Export | `Json` and `Csv` strings; warns for `Family=stewart` meters and `Family=legged` non-UR MoveJ handoff |
| Motus Export URDF | `RobotDescription` (Assemble/Attach); Folder; optional Name — click **Write** → Motus.NET `UrdfWriter` |

**Motus Waypoints** reshapes a planned trajectory for live controllers (e.g. UR Write). It does not connect to or command robots.

- `Q` — data tree, **one branch per waypoint**, `AxisCount` joint values (radians). Wire to joint MoveJ-style inputs.
- `P` — TCP planes via FK (same length as `Q` after decimate).
- `Tm` — waypoint times (seconds); metadata for downstream graphs.
- `D` (Decimate) — keep every Nth waypoint; **always keeps first and last**. Default `1` = all points.

Dense Motus paths executed as discrete MoveJ segments are stop-and-go; use Decimate to thin. Prefer `Q` → joint moves for planned path fidelity. Use `P` → linear TCP moves only for Cartesian-intent (LIN) paths — FK planes from joint-space / RRT trajectories are not a safe MoveL path (TCP re-interpolation can diverge). Warns when `AxisCount ≠ 6`. Controller handoff notes: [AGENTS.md](../AGENTS.md).

JSON export includes `jointNames` when the robot model provides them. Point count is the length of `Times`; duration is the last `Times` value (native Grasshopper list ops). `Retime` stays a boolean; optional `Retimer` selects `TotgLite` (default), `Totg`, `SegmentTrapezoid`, or `Bottleneck` when Motus.NET supports it. The algorithms and references are documented in [motus-net/METHODS.md](motus-net/METHODS.md) and [citation-audit.md](citation-audit.md).

**Motus Export URDF** is a thin wrapper over Motus.NET `UrdfWriter.Write`: wire a
`RobotDescription` from **Motus Urdf Assemble** / **Attach**, set Folder (+ optional Name), click
**Write**. Writes `.urdf` plus `meshes/` sidecars when mesh buffers are present. Normal solves only
re-report the last path — no disk IO until Write.

## Units

Joint inputs: **radians** by default. Right-click the **J** input on **Motus Joint State** and toggle **Degrees** for degree input (persisted in the `.gh`/`.ghx`). Planes and preview geometry use **meters**.

## Algorithms & citations

Planner/retimer/family methods and DOIs: [motus-net/METHODS.md](motus-net/METHODS.md), [citation-audit.md](citation-audit.md). Manual Rhino checks: [regression-matrix.md](regression-matrix.md).
