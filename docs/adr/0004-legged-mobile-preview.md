# ADR 0004: Legged mobile preview gait (N×3R)

## Status

Accepted (Motus.NET owns math; Grasshopper is thin Rhino I/O).

## Context

WalkHex needs a foot-target duty gait for TreeFK preview along a planar path. Kinematics families belong in Motus.NET (same bar as Stewart / ADR 0003). Methods must be NASA-traceable: named algorithms, peer-reviewed DOIs in code, explicit labels for engineering heuristics.

## Decision

1. **Motus.NET owns legged math** in `Motus.Geometry`:
   - `LegIk3R` — analytic coxa + planar 2R (Lynch & Park, *Modern Robotics*, DOI `10.1017/9781316095072`). **Not FABRIK** for the actuated 3R model; FABRIK remains the cited n-link alternative (Aristidou & Lasenby, DOI `10.1016/j.gmod.2011.05.003`).
   - `LeggedGait` — duty-factor swing groups (Song & Waldron, DOI `10.1177/027836498700600205`) + creeping stance plants (McGhee & Frank, DOI `10.1016/0025-5564(68)90041-2`).
   - Optional `LeggedGait.TerrainHeight` — world Z at (x,y); body base Z = terrain + `BodyZ` clearance; plant/land on sampled height; null = flat Z=0.
   - `StaticStability` — support-polygon CoM / SSM (McGhee & Frank, same DOI); Bretl & Lall (DOI `10.1109/TRO.2008.2001360`) cited as the next step for wrench-feasible equilibrium (not implemented). Horizontal projection only (non-coplanar contacts are a known SSM limit).
   - `LeggedMethodRefs` — central DOI constants + `DescribeStack()` for Status/logs.
2. **`RobotPreset.Family = Units.LeggedFamily` (`"legged"`)**. Gate Waypoints on Family: `Q` is joint **radians**, not Stewart meters.
3. **`Step` (m)** sets cadence: `cyclesPerPath = max(1, pathLength / stepLength)`.
4. **Swing schedule** is a partition of leg indices (`SwingGroups`); group `g` of `G` swings when cycle phase ∈ `[g/G, (g+1)/G)`. Specific hex tripod indices are a **design choice**, not a biology claim.
5. **Grasshopper is thin wiring only**: `LeggedGaitRhino` samples Curve/Planes → polyline; `TerrainHeightRhino` meshes GH Mesh/Brep → downward ray height; WalkHex / chrome keep hex naming. No duplicate IK/gait math in GH.
6. **Heuristics labeled in code**: sinusoidal swing lift above lerped terrain, land bias, drift replant, body-XY as CoM stand-in for SSM, body Z = terrain under body + clearance.
7. **Out of scope:** FABRIK cosmetic chains, Motus Plan for full-driver gait, Bretl–Lall wrench LP, friction cones / climbing.

## Consequences

- Hex is one factory (`LeggedLayout.HexMithi`); other N layouts reuse the same Motus.NET stack.
- Status/Remarks carry `LeggedMethodRefs.DescribeStack()` and min McGhee–Frank SSM.
- Docs distinguish Stewart (`Family=stewart`, meters) from walking hex (`Family=legged`, radians).
- Build Motus.Grasshopper with `-UseLocal` until Motus.NET with these types is published.
