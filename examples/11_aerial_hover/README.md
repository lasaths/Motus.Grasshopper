# Aerial hover (Motus 2.1 foundation)

Generated (`node scripts/generate-examples.mjs --only=11`). Drone-only —
arm pass-off comes later.

## Story

1. Motus Robot loads `assets/aerial/free_flyer_box.urdf` (`Base`/`Tip`=`body`);
   Family → `aerial`.
2. Two WorldXY planes (Z up): Start ≈ (−0.5, 0.35, 0.55), Goal ≈ (0.45, −0.25, 1.05).
3. Motus Plan (Auto Plan) → HolonomicSE3 path → Preview scrub + Export bodyPose +
   Waypoints (warns ≠ MoveJ).

## Wire

| Stage | Components |
|-------|------------|
| Robot | Path panel → Motus Robot |
| Env | Unit Z + Construct Point ×2 → Plane Normal (Start, Goal) |
| Plan | Motus Plan (Robot, Start, Goal) |
| Play | Scrub + Preview + Waypoints + Export |

Logic: Motus.NET HolonomicSE3 / `AerialExportTests`. ADR: GH 0006 · Motus.NET 0001.
