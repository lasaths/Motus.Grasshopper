# ADR 0003: Parallel kinematics — Stewart / Gough platform

## Status

Accepted (Wave 0–3).

## Context

Motus Plan already accepts an ordered list of TCP planes and follows them with Cartesian LIN on a **serial tip chain**. Users need the same UX for Stewart/Gough hexapods (fixed base, moving platform, six prismatic legs): platform pose path → leg-length trajectory.

ADR 0002 keeps `KinematicTree` / `RobotDescription` as **open trees** (no closed loops). Stuffing six platform parents into that graph fights assemble validation and TreeFK. Parallel IK therefore needs a **sibling** Motus.NET stack, not a tip-extract hack.

## Decision

1. **Motus.NET owns Stewart math.** Types live in `Motus.Geometry` (`StewartPlatform`, `StewartInverseKinematics`, `StewartForwardKinematics`, Stewart Cartesian path planner) with Core contracts for units and structured results. Grasshopper is thin wiring only.
2. **`RobotPreset.Family = "stewart"`** identifies the family (reuse existing `Family` field; do not invent a parallel goo taxonomy).
3. **`JointState.Positions[i]` for Stewart = leg length in meters.** Limits use `JointCoordinateUnit.Meters`. Serial arms remain radians.
4. **IK is analytic and unique** for a platform pose: `L_i = ‖ T · P_i − B_i ‖`. No serial-style multi-solution branch picking. Path continuity = pose path staying inside stroke / ΔL thresholds.
5. **FK is numerical** (Newton on pose residual) with documented tolerances, iteration cap, and structured diverge failure. Preview/scrub seed from the previous pose.
6. **Structured reason codes** (`Ok`, `StrokeLimit`, `Singular`, `FkDiverge`, `InvalidInput`, `DeltaLengthJump`) flow to GH Status — not bool-only `TrySolve` alone.
7. **Waypoints / export** gate on `Family`, not `AxisCount == 6`. Stewart `Q` must not be handed to UR MoveJ without an explicit warning.
8. **Closed-loop URDF via `RobotDescription.TryAssemble` remains rejected.** Stewart geometry uses a dedicated model/loader (typed builder or versioned JSON), not multi-parent tree assemble.
9. **Collision / sampling RRT for Stewart** is a later wave (needs unit-correct configuration steps and envelopes). Until then, Plan for Stewart is TCP LIN + IK + limits only.

### Defaults (locked)

| Parameter | Default |
|-----------|---------|
| FK position tolerance | 1e-6 m |
| FK orientation tolerance | 1e-6 rad (axis-angle residual) |
| FK max Newton iterations | 40 |
| Singularity (Jacobian condition) | reject if cond > 1e8 |
| Path max leg ΔL per TCP step | 0.05 m (configurable) |
| LIN TCP step | 0.005 m (same as serial LIN default) |

## Consequences

- ADR 0002 tree ownership unchanged for serial/branched tools; planners may use Stewart APIs when `Family=stewart`.
- Motus.Grasshopper Plan/Preview/Waypoints dispatch on `Family`.
- GH may author base/platform anchors as Rhino point lists on Motus Stewart; Motus.NET still owns `StewartPlatform` validation and solvers.
- Export JSON/CSV must label Stewart joints as meters, not radians.
- Package bump required before NuGet-consuming GH builds; develop with `./build.ps1 -UseLocal`.
