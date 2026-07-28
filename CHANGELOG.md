# Changelog

## 0.13.2 — UR+DKP AllDrivers ceilings

Aligned with **Motus.NET 0.13.2**.

### Added / fixed

- Example `06_turntable_group` (UR + 1-DOF 8-spoke turntable); Motus Robot `AllDrivers` Plan DOF = tip + side branches
- Synced joint goals: arm TCP tracks turntable fixture as `turntable_yaw` rotates
- Planning collision via TreeFK so turntable geometry participates in RRT (not tip-chain only)
- Plane/LIN on AllDrivers: tip-chain IK; side branches held at start and re-embedded
- URDF box/cylinder preview uses `ToPlanePlate` (Motus XYZ ≡ Rhino XYZ)

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
