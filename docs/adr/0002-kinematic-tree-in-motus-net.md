# ADR 0002: Kinematic tree ownership in Motus.NET

## Status

Accepted (Waves 0–2). Wave 2 adds `ToolParameterBinding`, `JointTableTrees`, and `MobilityModel.HolonomicSE2` in Motus.NET. Amended below for typed Link/Joint GH goo (structural authoring via `RobotDescription`).

## Context

Motus planning and preview need a single kinematic source of truth that covers serial arms, branched tools (e.g. Robotiq fingers / mimic joints), and future mobile / climbing bases. Today, Grasshopper risks accumulating link×joint “spaghetti” and Motus-only FK that diverges from Motus.NET planners.

Wave 0 freezes the intended Motus.NET surface and ownership so Wave 1 can assemble the same tree from URDF load and from GH **Motus Serial Chain**, without moving algorithms into the plugin.

## Decision

**All kinematic models and algorithms live in Motus.NET.** Motus.Grasshopper is a thin consumer: UI, preview, assemble/debounce, and export handoff — not the home of FK/IK/reach math.

### Target Motus.NET types (Gate 0 intent)

| Type / API | Role |
|------------|------|
| `KinematicTree` | Branched kinematic model (links, joints, mimics). |
| `TreeForwardKinematics` | Tree FK; primary API `ComputeLinkTransformsInto(...)`. |
| Mimic | Mimic-joint resolution on the tree. |
| Fingerprint | Cheap structural identity for cache / invalidate. |
| `ReachSampling` | Bounded TCP reach sampling; fill via `FillTcpPointsInto`. |
| `SerialJointChain` | Tip-group view for existing planners (not a second model). |

Gate 0 freeze intent for the public surface:

- `ComputeLinkTransformsInto` (and related Into-style writers)
- Fingerprint
- Tip extract from the tree / tip-group view
- `ReachSampling.FillTcpPointsInto`
- Tip-chain TCP parity with existing `SerialForwardKinematics`

### Authoring (Wave 1)

URDF load and GH **Motus Serial Chain** both build the **same** Motus.NET `KinematicTree`. Reject Link×N / Joint×N Grasshopper spaghetti as an authoring path.

### Performance budgets

| Path | Budget |
|------|--------|
| Tree FK | < ~50 µs typical for ≤ ~20 links |
| Scrub | < ~2 ms transform-only (no mesh rebuild) |
| Assemble | Debounce 100–150 ms |
| Reach | ≤ 512 samples in < ~16 ms |
| Scrub frames | No `DuplicateMesh` per frame |
| Reach grids | No joint-product reach grids |

### Mobility (Wave 2)

- `MobilityModel.HolonomicSE2` / `Fixed` in Motus.NET — base frame from (x, y, yaw)
- GH **Motus Joint Table** optional `SE2` pin = base pose override only (**not** mobile RRT / SE2 state in the planner)
- Serial Chain keeps plane `Base`
- Nonholonomic / climbing base swaps remain later

### Branching vs Plan (Wave 2)

- Full `KinematicTree` may branch; default **Motus Plan / Joint State** use `ExtractSerialTip` along the Tip link path.
- `AxisCount` and `JointLimits` must match the active Plan DOF (tip path, or tip + side branches when AllDrivers).
- Side-branch drivers are TreeFK/preview-only unless **AllDrivers** is enabled on **Motus Robot** or **Motus Joint Table** (shared `PlanDofComposer` tip-first layout; tip-descendant tool knuckles stay off Plan).

### Out of scope

- Live RTDE / robot execution
- Material / RL concerns
- Motus FK implemented only inside Grasshopper

## Consequences

- Motus.NET owns tree FK, mimic, fingerprint, reach sampling, and tip-chain parity; GH wires and previews only.
- For **serial / open-tree** robots, planners keep using `SerialJointChain` as a tip-group view over the tree.
- **Parallel Stewart/Gough platforms** are a sibling Motus.NET stack (`Family=stewart`) — see [ADR 0003](0003-parallel-kinematics-stewart.md). They do **not** use `ExtractSerialTip` or closed loops inside `KinematicTree` / `RobotDescription`.
- Wave 1 GH assemble must target Motus.NET tree construction; perf budgets constrain scrub/preview and reach UX.
- Gate 0 is an API freeze **intent** for Motus.NET; Grasshopper Wave 0 does not implement Motus.NET kinematics.
- Mobility / SE(2) / climbing remain documented extension points until a later wave.

### Amendment: AllDrivers Plan DOF (Motus Robot + Joint Table)

Optional **AllDrivers** raises Plan `AxisCount` / `JointNames` to tip-path joints first, then non-tip side-branch drivers (excluding tip descendants). Plane/LIN still tip-IK with branches held; joint goals move side branches. Motus.NET planners already key off `AxisCount > tipN` — no new planner stack.

### Amendment: typed Link/Joint GH goo (structural authoring)

The Wave 1 rejection of "Link×N / Joint×N Grasshopper spaghetti" (above) targeted **anonymous**
parameter spaghetti — a robot assembled from bare numbers/planes with no schema, re-derived and
re-validated ad hoc per graph. It did not anticipate a **typed** authoring surface. This amendment
narrows the rule:

- **Allowed:** typed, per-node GH goo — `UrdfLinkGoo`, `UrdfJointGoo`, `RobotDescriptionGoo` —
  wired through dedicated components (**Motus Urdf Link / Joint / Assemble / Explode / Attach**).
  Link visuals are native GH Box/Mesh/Brep (no Motus geometry goo). Each goo wraps a validated
  Motus.NET value type (`UrdfLink`, `UrdfJoint`, `RobotDescription`); GH never invents its own
  link/joint schema.
- **Still rejected:** wiring raw numbers/planes/strings directly into a from-scratch tree builder
  in Grasshopper with no Motus.NET-owned type in between. If a component accepts bare doubles for
  origins/axes/limits and assembles a tree itself, that is the spaghetti this ADR rejects.
- **Motus.NET owns assemble/attach.** Topology validation (single root, no orphan/duplicate
  links, mimic target resolution) and mechanism composition happen in `RobotDescription.TryAssemble`
  / `RobotDescription.Attach` / `RobotDescription.Explode` (Motus.Geometry), and projection to
  `KinematicTree` happens via `RobotDescriptionSession.Project`. **Motus Urdf Assemble** and
  **Motus Urdf Attach** call into these — they do not re-implement tree construction or attach
  math on the GH side.
- **GH stays a thin wrapper.** Grasshopper components collect per-node inputs (geometry dims/mesh,
  parent/child names, axis line, limits) into the typed goo above and hand the assembled
  `RobotDescription` back to Motus.NET (`RobotDescriptionSession.Project`) for FK/planning — the
  same pattern as URDF-file load and **Motus Serial Chain** building the same `KinematicTree`.
- Rationale: this is how a driven mechanism (gripper, turntable, rail) gets authored *without* a
  URDF file on disk, while keeping GH out of the kinematics-implementation business per the
  Decision above. **Motus Export URDF** calls Motus.NET `UrdfWriter` on a `RobotDescription` (no
  GH-local URDF XML) for handoff to other URDF-consuming tools.
