# ADR 0006: Aerial free-flyer example (Motus 2.1)

## Status

Accepted — **drone-only foundation** first. Arm pass-off deferred until Rhino hover is green.

## Context

Motus 2.1 needs a Grasshopper surface for `Family=aerial` HolonomicSE3 (Motus.NET ADR 0001 / [aerial.md](https://github.com/lasaths/Motus.NET/blob/master/docs/aerial.md)). Dual-agent tower pass-off (Motus.NET [ADR 0002](https://github.com/lasaths/Motus.NET/blob/master/docs/adr/0002-aerial-arm-pass-off.md)) is the product end-state, but example 11 ships **drone-only** so Plan → Preview → Export can be verified without Pick Place / One Play complexity.

## Decision

1. **Example 11 = free-flyer hover** — `11_aerial_hover.ghx` via `node scripts/generate-examples.mjs --only=11`. Motus Robot loads `assets/aerial/free_flyer_box.urdf` (`Base`/`Tip`=`body`); Family promotes to `aerial`. Start + Goal are **WorldXY** body planes (Z up) → `FromPlanePlate` → HolonomicSE3. Motus Plan Auto Plan → Preview / Scrub / Export / Waypoints.
2. **No arm in v1** — Do not wire UR / Pick Place / dual Trajectory Preview until the hover path Status + body scrub look correct in Rhino.
3. **Honesty** — Waypoints/Export warn bodyPose ≠ UR MoveJ. Station-hold + Attach span only when Plan `Attach` is wired (pass-off later).
4. **Logic tests** — Motus.NET HolonomicSE3 / `AerialArmPassOffTests` / `AerialExportTests` own contracts; GH qa-smoke covers Family promote + handoff warnings.

## Consequences

- Regression matrix: Rhino row for `11_aerial_hover` (Auto Plan, scrub body, Export bodyPose).
- Pass-off example (drone + example 10 tower) stays a follow-on ADR/example once hover is trusted.
