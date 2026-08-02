# ADR 0005: General legged mechanism (N-leg Walk)

## Status

Accepted (Motus.NET owns math; Grasshopper is thin Rhino I/O).

## Context

ADR 0004 delivered N×3R preview gait behind Motus Hex / WalkHex. Users need arbitrary leg count, per-leg serial chains, and pluggable body-pose policy without a Hex special case. NASA bar: SI units, typed failures, DOI methods, no silent NaN. Apple bar: Body + Leg + Mechanism → Walk.

## Decision

1. **`LeggedMechanism`** replaces GH-facing Hex layout: body link, namespaced leg instances (`legName/` on Attach), published `DriverOffsets`, `GaitSchedule`, tip leg, clearance (m along body +Z to support plane).
2. **`GaitSchedule`** — Song & Waldron duty factor `β` + per-leg phase offsets. `FromGroups` sugar. Auto: `G = max(2, ⌈N/(N−3)⌉)`, round-robin on sorted hip yaw → hex tripod `[[0,2,4],[1,3,5]]`, N=4 crawl. **N ≤ 3 rejected** for static gait unless `AllowDynamicGait`.
3. **`ILegIkSolver`** — `TrySolve` + `TryNominalStance` + `Workspace`. Insectoid 3R → `LegIk3R`. Else numerical position/pose with **`TrySolveNear` only**. Typed failure codes.
4. **`IBodyPoseSolver`** — immutable descriptor + `CreateSession()` (EMA safe under GH fan-out). v1: PathFollow, TerrainSupport (stance-weighted plane; `w_min=1` = legacy).
5. **`LeggedGait.TryBuild(mechanism, bodySolver, …)`** — Walk solver math. IK-fail legs excluded from support; degenerate SSM counted; swing-resolved sampling.
6. **Grasshopper thin only:** Motus Body (structure), Motus Leg (recipe), Motus Mechanism (assemble), Motus Walk (gait solver). Optional Motus Body Pose for Custom policy. **No Motus Hex.** No gait/IK/SSM math in GH.
7. **`LeggedLayout.HexMithi` / `QuadSmoke`** remain Motus.NET test factories only (not GH components). QuadSmoke trot requires `AllowDynamicGait`.

## GUID fate (GH 0.13)

| Name | GUID | Fate |
|------|------|------|
| Motus Hex | `c7a02fcb-2562-4540-9f44-5cc9e99293ec` | Removed (optional `IGH_UpgradeObject` → Body+Leg+Mechanism) |
| `Param_MotusHex` | `908aabb4-0e11-4ad1-9ced-ed87671c3499` | Removed |
| Motus Walk Hex → Motus Walk | `236f9a53-c07b-4663-bf27-950e20fb59ab` | Kept; `Hx` → `Mech` |
| Motus Leg | `9a49a661-ff4c-4b96-bb57-c977ee6f9da2` | New |
| Motus Body | `92f0d969-c8ef-47c5-9ec7-514bebbd8441` | New |
| Motus Mechanism | `aa18b783-9a1c-44f8-bd2b-e508c3d372ac` | New |
| Motus Body Pose | `76051f49-2641-4530-8b79-c5635a8e6eaf` | New |
| `Param_MotusLeg` | `b7a9381b-cbce-4df0-8e74-46d7ca62cea1` | New |
| `Param_MotusBody` | `accf652b-0591-4d03-84ac-811a510cb2ef` | New |
| `Param_MotusMechanism` | `4a2b9635-a730-4ee5-9272-266d1ce9bef4` | New |
| `Param_MotusBodyPose` | `03e55c53-15d4-4b46-9927-33803788db85` | New |

## Consequences

- Breaking: old Hex definitions need Body+Leg+Mechanism; Walk `Hx` type changes.
- Build GH with `-UseLocal` until Motus.NET with these types is on `master` and `MotusNetVersion` is bumped.
- Example 09 rebuilt without Hex. Family=legged handoff unchanged (radians, not UR MoveJ).

## Amendment: Motus Plan body-path gait (0.14)

Motus Plan synthesizes full-driver gait when `RobotModelGoo` carries `LeggedMechanism` (Walk always attaches it) and Goal is an all-plane list (≥2). Plane **origins** are the body path (m); orientation ignored; yaw from path tangent. Defaults match Walk (`Spd/St/Lf` via `LeggedGait.Default*`); body pose `PathFollow`; flat Z=0 (no Terrain pin on Plan). API: `LeggedGait.PlanBodyPath` → `TryBuild` → `ValidateForPlan` (**hard SSM** — Walk may still soft-warn + emit `Tr`). Tip-path joint goals and a single plane remain tip LIN/RRT. Walk stays the rich gait UI (Path/Terrain/Speed/Step/Lift/Pose).

## Out of scope

Bretl–Lall wrench LP, friction/climbing, FABRIK cosmetics, articulated torso DOF, Plan Terrain/Speed pins.
