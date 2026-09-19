## Unreleased

## 2.0.0 — Public product · Yak GA identity — 2026-09-19

Aligned with **Motus.NET 2.0.0** (NuGet publish pending — Prefer `-UseLocal` until nuget.org has **2.0.0**). CI NuGet jobs override to Motus.NET **1.8.0** (last confirmed live index) while **1.9.0** may still be indexing. **Yak Package Manager push is not part of this SemVer cut** — pack path ready; production push needs human auth after NuGet **2.0.0** is live.

### Changed

- Plugin / yak manifest / `MotusNetVersion` pin aligned to **2.0.0**.
- Docs (AGENTS, README, examples, regression matrix, supported-surface) describe **2.0.0** cut + UseLocal until Motus.NET **2.0.0** NuGet publish.
- `verify-motus-net-pin.mjs` EXPECTED **2.0.0**.

### Motus.NET pin

`MotusNetVersion` = **2.0.0** ([`build/MotusNetPackages.props`](build/MotusNetPackages.props)). Prefer UseLocal until nuget.org lists Motus.Core **2.0.0**.

## 1.9.0 — Host product · Pick Place touch gate — 2026-09-19

Aligned with **Motus.NET 1.9.0** ([GitHub v1.9.0](https://github.com/lasaths/Motus.NET/releases/tag/v1.9.0); nuget.org index may lag). Yak still unpublished until production **2.0.0** push.

### Fixed

- Motus Pick Place: empty **Touch** is an Error and emits no `Seg` (fail-closed; avoids Program `Tr` null after Detach-at-place). qa-smoke covers `PickPlaceTouchContract`.
- Motus Program: keep prior `Tr` on planning re-entry and empty Seg; Auto Plan fingerprint includes Group / Attach / Robot / Tool / Prior.
- Motus Robot: experimental Remark for free-flyer / H2 / Go2 (and `Family=urdf` AxisCount=0) URDF loads.
- Motus Waypoints / Export: shared `FamilyHandoffWarnings` (incl. `Family=urdf` AxisCount=0 ≠ MoveJ).

### Changed

- Plugin / yak manifest / `MotusNetVersion` pin aligned to **1.9.0**.
- Docs (AGENTS, README, examples, regression matrix) describe **1.9.0** cut + UseLocal until Motus.NET **1.9.0** NuGet publish.

### Motus.NET pin

`MotusNetVersion` = **1.9.0** ([`build/MotusNetPackages.props`](build/MotusNetPackages.props)). UseLocal until nuget.org lists Motus.Core **1.9.0**.

## 1.8.0 — Trust & polish · SemVer jump — 2026-09-19

Aligned with **Motus.NET 1.8.0** on [nuget.org](https://www.nuget.org/packages/Motus.Core/1.8.0). Public package SemVer jumps **0.17.0 → 1.8.0** (no empty 1.0–1.7 line); Yak still unpublished (Package Manager target **2.0.0**).

### Added

- CI `build-*-nuget` jobs alongside Prefer UseLocal (host-readiness for NuGet-default restore).
- Cap `ToolCapContract.TryValidateBinding` extraction + qa-smoke Cap block; regression matrix points at Motus.NET `RegressionMatrixLogicTests`.
- `scripts/verify-motus-net-pin.mjs` — pin/docs/nuget.org honesty check.

### Changed

- Plugin / yak manifest / `MotusNetVersion` pin aligned to **1.8.0**.
- Docs (AGENTS, README, examples, regression matrix) describe **1.8.0** with NuGet default after publish.

### Motus.NET pin

`MotusNetVersion` = **1.8.0** ([`build/MotusNetPackages.props`](build/MotusNetPackages.props)). Local default = NuGet after publish; Yak `motus` still unpublished (Package Manager target is **2.0.0**).

## 0.17.0 — Pick/place transfers + attachment timelines — 2026-09-19

Aligned with **Motus.NET 0.17.0** on [nuget.org](https://www.nuget.org/packages/Motus.Core/0.17.0). Default Release restores NuGet; `-UseLocal` remains for Motus.NET tip / CI.

### Added

- Pick Place optional `RRT` and `Touch` inputs for sampled hover transfers and explicit gripper contact names. Existing inputs keep their order.
- Program uses RRT-Connect for Transfer segments with the current scene/attached-body checker. Contact edits invalidate cached plans.
- Example **10** sets Pick Place `Touch=robotiq_2f85` so Detach-at-place does not null Program `Tr`.

### Fixed

- Attachment timelines remain on core trajectories during concatenation and export; placed objects detach before retraction.
- Motus Program keeps the previous `Tr` while Auto Plan replans (no null flicker).
- Preview / mesh leak and helper consolidation for pick-place scrub stability.

### Motus.NET pin

`MotusNetVersion` = **0.17.0** ([`build/MotusNetPackages.props`](build/MotusNetPackages.props)). Local default = NuGet; CI continues UseLocal against sibling Motus.NET. Yak `motus` still unpublished (Package Manager target is **2.0.0**).

## 0.16.0 — Tower destack / pick-place cycles

Aligned with **Motus.NET 0.16.0**.

### Added

- **Motus Collision Boxes** — plane list → named box objects.
- **Motus Pick Place** — grasp/place/object lists → LIN/SET/Attach/Detach segments (`PickPlaceCycle`).
- **Motus Program** stamps per-cycle `AttachSpans` from planner attach/detach windows.
- Example **10** tower layout is a **C# Script** on the canvas (not a Motus component).

### Fixed

- Preview keeps placed bricks at Detach `ReleaseWorldPose` while the next cycle is attached.
- Motus Program Auto Plan replans when segment/start/scene fingerprint changes.

### Motus.NET pin

`MotusNetVersion` = **0.16.0** ([`build/MotusNetPackages.props`](build/MotusNetPackages.props)).

## 0.15.0 — Custom tool / kinematics close-out

Aligned with **Motus.NET 0.15.0**.

### Added

- Motus Tool Cap=`Custom` with Wmin/Wmax/Cd pins; Cap-agnostic width→driver bindings.
- Motus Joint Table **AllDrivers** pin (shared `PlanDofComposer` with Motus Robot).
- ToolGoo persists Cap/Bindings/Mechanism via Motus.NET `UrdfWriter` inline XML (TL-009).

### Changed

- Numerical IK Plan Status names `NoConvergence` / `SingularJacobian` / `InvalidInput`; ADR 0002 AllDrivers amendment.

### Motus.NET pin

`MotusNetVersion` = **0.15.0** ([`build/MotusNetPackages.props`](build/MotusNetPackages.props)). CI UseLocal until NuGet publishes 0.15.0.

## 0.14.0 — Legged Plan body-path gait

Aligned with **Motus.NET 0.14.0**.

### Added

- **Motus Plan** synthesizes Family=legged full-driver gait when Walk `Rb` carries `LeggedMechanism` and Goal is ≥2 planes (origins = body path). Uses `LeggedGait.PlanBodyPath` (PathFollow, Walk defaults, hard SSM). Tip-path joint goals and single-plane TCP LIN unchanged.
- Walk always attaches Mechanism + stance angles on emitted `Rb`.
- Status honesty: body-path ≠ TCP LIN; Q full-driver radians ≠ UR MoveJ; mixed plane+joint / no-Mechanism plane goals fail named.

### Motus.NET pin

`MotusNetVersion` = **0.14.0** ([`build/MotusNetPackages.props`](build/MotusNetPackages.props)). CI UseLocal until NuGet publishes 0.14.0.

## 0.13.2 — UR+DKP AllDrivers ceilings

Aligned with **Motus.NET 0.13.2**.

### Added / fixed

- Example `06_turntable_group` (UR + 1-DOF 8-spoke turntable); Motus Robot `AllDrivers` Plan DOF = tip + side branches
- Synced joint goals: arm TCP tracks turntable fixture as `turntable_yaw` rotates
- Planning collision via TreeFK so turntable geometry participates in RRT (not tip-chain only)
- Plane/LIN on AllDrivers: tip-chain IK; side branches held at start and re-embedded
- URDF box/cylinder preview uses `ToPlanePlate` (Motus XYZ ≡ Rhino XYZ)
- **Motus Tool Cap rethink:** Cap is face dropdown schema only (`None` \| `Robotiq2F85`); no name-sneak Cap; Tool State face Preset; wired Cap-less Tool → Error (unwired still warns + Robotiq). Pins stay stable (no VariableParameter morph — GHX wires survive). Examples `03`/`04`/`07` regenerated.

### Motus.NET pin

`MotusNetVersion` = **0.13.2** ([`build/MotusNetPackages.props`](build/MotusNetPackages.props)).

## 0.13.0 — N-leg Walk (ADR 0005)

Aligned with **Motus.NET 0.13.1**.

### Changed

- **Motus Stewart**: optional `Base`/`Plat` (6 points each) and `PairSep`; priority JSON → anchors → classic Br/Pr. Example `08_stewart_tcp_path` exposes Br/Pr/Lmin/Lmax sliders + multi-waypoint TCP path (09-style modular knobs).

### Breaking

- **Motus Hex removed** (`c7a02fcb-2562-4540-9f44-5cc9e99293ec`). `Param_MotusHex` / `HexLayoutGoo` removed.
- **Motus Walk Hex → Motus Walk** (GUID kept `236f9a53-c07b-4663-bf27-950e20fb59ab`). Input `Hx` → required `Mech` (`Param_MotusMechanism`).

### Migration

```
Motus Body (N/Br/Bz) + Motus Leg (L) → Motus Mechanism → Motus Walk (Mech, Path/Planes, Tn)
```

Optional **Motus Body Pose** (`PathFollow` | `TerrainSupport`) → Walk `Pose`. Omit Pose: Auto = TerrainSupport when `Tn` wired, else PathFollow.

### Added

| Component | GUID |
|-----------|------|
| Motus Leg | `9a49a661-ff4c-4b96-bb57-c977ee6f9da2` |
| Motus Body | `92f0d969-c8ef-47c5-9ec7-514bebbd8441` |
| Motus Mechanism | `aa18b783-9a1c-44f8-bd2b-e508c3d372ac` |
| Motus Body Pose | `76051f49-2641-4530-8b79-c5635a8e6eaf` |

Params: `Param_MotusLeg`, `Param_MotusBody`, `Param_MotusMechanism`, `Param_MotusBodyPose`.

See [ADR 0005](docs/adr/0005-general-legged-mechanism.md). Family=legged Waypoints warnings unchanged (radians, not UR MoveJ).

### Motus.NET pin

`MotusNetVersion` = **0.13.1** ([`build/MotusNetPackages.props`](build/MotusNetPackages.props)). CI still UseLocal against sibling Motus.NET; repo variable `MOTUS_NET_REF` (default `master`) selects the checkout ref.
