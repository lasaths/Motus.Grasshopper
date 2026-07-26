# ADR 0004: Legged mobile preview gait (N×3R)

## Status

Accepted (Motus.NET owns math).

## Context

WalkHex needs a foot-target duty gait for TreeFK preview along a planar path. Kinematics families belong in Motus.NET (same bar as Stewart / ADR 0003). An early GH-only `WalkingHex*` stack used Rhino types and deferred the port.

## Decision

1. **Motus.NET owns legged math** in `Motus.Geometry`: `LeggedLayout`, `LeggedGait`, `LegIk3R` (N×3R insectoid + flat Z=0). Path input is a planar polyline (`IReadOnlyList<Vec3>`, Z ignored).
2. **`RobotPreset.Family = Units.LeggedFamily` (`"legged"`)**. Gate Waypoints on Family: `Q` is joint **radians**, not Stewart meters and not UR MoveJ for full-driver gait.
3. **`Step` (m)** sets cadence: `cyclesPerPath = max(1, pathLength / stepLength)`.
4. **Swing schedule** is a partition of leg indices (`SwingGroups`); group `g` of `G` swings when cycle phase ∈ `[g/G, (g+1)/G)`.
5. **Grasshopper is thin wiring**: `LeggedGaitRhino` samples Curve/Planes → polyline; WalkHex / preview chrome keep hex naming.
6. **Out of scope:** FABRIK, terrain, n-DOF legs, Motus Plan for full-driver gait, a separate Quadruped GH component.

## Consequences

- Hex is one factory (`LeggedLayout.HexMithi`); other N layouts (e.g. `QuadSmoke`) reuse the same gait/IK in Motus.NET.
- Docs distinguish Stewart (`Family=stewart`, meters) from walking hex (`Family=legged`, radians).
- Example `09_walking_hexapod.ghx` GUID/pins unchanged.
- Build Motus.Grasshopper with `-UseLocal` / `UseMotusNetProjectReference=true` until Motus.NET with these types is published to NuGet.
