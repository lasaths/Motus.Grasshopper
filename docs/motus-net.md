# Motus.NET dependency

Sibling library at [Motus.NET](https://github.com/lasaths/Motus.NET) (pinned **0.6.8**). NuGet via [`build/MotusNetPackages.props`](../build/MotusNetPackages.props); local checkout at `../Motus.NET` switches to project refs ([`build/MotusNetLocal.props`](../build/MotusNetLocal.props)).

## Packages

| Package | Role |
|---------|------|
| `Motus.Core` | Models, `IPlanner`, validation, `PlanningContext`, PlanBundle JSON export |
| `Motus.Geometry` | FK/IK, mesh/sphere collision, Cartesian LIN, industrial motion |
| `Motus.OMPL.NET` | Sampling planner façade + registry (`SamplingPlanner`) |
| `Motus.Presets` | UR/KUKA JSON presets, URDF/xacro/SRDF loaders |
| `Motus.Native` / `Motus.OMPL.Native` | Optional P/Invoke to `motus_native` (OMPL + FCL) |

## Always available (managed)

These work **without** full native OMPL — the Rhino/GH default:

- **JointLinearPlanner** — free-space joint interpolation
- **CartesianLinearPathPlanner** — TCP LIN after IK
- **IndustrialMotionPlanner** — PTP / LIN / CIRC + blends
- **Managed RRT-Connect** — only sampling planner with a C# fallback
- **C# mesh/sphere collision** + attach-aware checking
- **PlanningContext** — attach bodies, SRDF groups, locked non-group joints
- **Kinematics** — UR analytic IK; numerical IK for generic chains
- **Presets / URDF** — UR3e–UR30, KUKA KR/LBR, tool/home/viewer presets
- **TrajectoryExport** — PlanBundle JSON for GH / adapters

Plane goals, free-space joints, motion programs, and collision RRT all work with only `RrtConnect` in the Planner dropdown.

## Sampling planners (Motus RRT Settings dropdown)

Source: `SamplingPlannerRegistry.ListAvailable()` — hides planners without managed **or** native support.

| ShortName | Managed? | Needs native? |
|-----------|----------|---------------|
| `RrtConnect` | Yes | Optional |
| `RrtStar` | No | Yes |
| `Aorrtc` | No | Yes (OMPL 2.0+) |
| `Lbkpiece` | No | Yes |
| `AitStar` / `EitStar` / `BlitStar` | No | Yes (compile-gated) |
| `ParallelRace` | Meta (Connect+AORRTC) | Needs native AORRTC |

Stub / NuGet desktop builds → **RrtConnect only**. That is expected, not a missing Motus.NET install.

## Full native (optional)

Build Motus.NET with `scripts/build-native-full.{sh,ps1}` (`MOTUS_USE_OMPL=ON`, `MOTUS_USE_FCL=ON`) to unlock extra sampling planners, native path simplify, and FCL collision.

Current NuGet/release ships **stubs**. Check Motus Plan `Warnings` → `MotusCapabilities.Describe()` for managed vs native status.

## Bottom line

Motus.NET is the whole planning stack (kinematics, collision, LIN/industrial, presets, export). A one-item Planner Value List means only one **sampling** planner is available in this stub build — not that Motus.NET only has one planner.
