# ADR 0002: Kinematic tree ownership in Motus.NET

## Status

Accepted (Wave 0 — documentation / Gate 0 API freeze intent). No `MobilityModel` implementation in this wave.

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

### Mobility (hooks only)

This ADR records future hooks — prose only in Wave 0:

- `MobilityModel` (empty type / SE(2) / climbing later)
- Base-frame swap for mobile or climbing setups

**Do not implement** `MobilityModel` (even an empty stub) in Wave 0.

### Out of scope

- Live RTDE / robot execution
- Material / RL concerns
- Motus FK implemented only inside Grasshopper

## Consequences

- Motus.NET owns tree FK, mimic, fingerprint, reach sampling, and tip-chain parity; GH wires and previews only.
- Planners keep using `SerialJointChain` as a tip-group view over the tree, not a parallel kinematics stack.
- Wave 1 GH assemble must target Motus.NET tree construction; perf budgets constrain scrub/preview and reach UX.
- Gate 0 is an API freeze **intent** for Motus.NET; Grasshopper Wave 0 does not implement Motus.NET kinematics.
- Mobility / SE(2) / climbing remain documented extension points until a later wave.
