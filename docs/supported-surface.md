# Supported vs advanced surface (toward Motus 2.0)

Host-facing tiers for Motus.Grasshopper. SemVer **2.0.0** = first public Yak `motus` Package Manager release and the stability promise for **Supported** components. This document does **not** bump plugin / pin / yak versions.

Library-side detail: Motus.NET [`docs/supported-surface.md`](https://github.com/lasaths/Motus.NET/blob/master/docs/supported-surface.md) (land with companion PR if not yet on master).

## Supported (Yak GA at 2.0.0)

| You want… | Components / path |
|-----------|-------------------|
| Serial UR10e + Robotiq plan | Motus UR10e Robotiq / Robot → Plan (LIN / joint / RRT) → Preview |
| Pick / place | Collision Boxes + Pick Place (`Touch` required) → Program |
| Controller handoff | Waypoints `Q` → MoveJ **serial only** |
| Tool | Cap / Rd / Bd; SET widths as export hints |

Matrix: [regression-matrix.md](regression-matrix.md) Supported / Rhino rows. Example: `10_pick_place.ghx`.

## Advanced (in plugin; honest, not Yak headline)

| Family / feature | Honesty |
|------------------|---------|
| Stewart | `Q` meters; Waypoints/Export warn ≠ UR MoveJ |
| Legged Walk / Plan body-path | Radians; full-driver gait ≠ MoveJ for whole mechanism |
| Joint Table SE2 | HolonomicSE2 base override; preview/Plan DOF rules |

## Experimental (not GA)

| Item | Host rule |
|------|-----------|
| `Family=aerial` | Waypoints/Export warn bodyPose ≠ MoveJ; **no** Aerial GA component |
| H2 / Go2 / free-flyer URDF | Optional Motus Robot load — experimental Status only |
| Catalog Panda smoke | Motus.NET CI; not a GH GA preset |

## Out through 2.0

No RTDE / Session / Run / live commands. No ROS. Yak `motus` **2.0.0** is live on Package Manager ([packaging/yak/README.md](../packaging/yak/README.md)).

## Yak

First production `yak push` is **2.0.0** only. Pack path: `./build.ps1 -Configuration Release -Yak` or `./scripts/pack-yak.sh`. Agents must not interactive `yak login`.
