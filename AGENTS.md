# AGENTS.md

Notes for AI agents and maintainers working on Motus.Grasshopper.

## Engineering bar (NASA-quality)

Strive for **NASA-grade engineering** on kinematics, planning, and handoff surfaces — not perfection theater, but no silent failures or unit lies.

- **Clear contracts** — explicit units (rad vs m), `RobotPreset.Family`, and Status strings that name the real failure (stroke, singular, FK diverge, invalid input).
- **No silent failures** — reject NaN/Inf at load boundaries; solvers return structured reason codes; never ship garbage poses/meshes.
- **Deterministic solvers** — documented tolerances, iteration caps, seeded numerical FK; verified FK↔IK round-trips before GH wiring.
- **Trust boundaries** — validate limits, schema versions, and size caps on external mechanism descriptions.
- **Docs before new families** — ADR + component reference + example when adding a kinematics family (see [0003](docs/adr/0003-parallel-kinematics-stewart.md) for Stewart, [0004](docs/adr/0004-legged-mobile-preview.md) for legged preview gait).
- Prefer correctness and reviewability over clever shortcuts. Serial UR10e paths must stay green when parallel families land.

## Boundaries

- **Planning / preview / export only** — no RTDE, no live robot commands, no project reference to UR.RTDE.Grasshopper.
- Execution (Session, Run, waits, ServoJ) lives in downstream control plugins.
- User component reference: [docs/grasshopper-components.md](docs/grasshopper-components.md).
- ADR: [docs/adr/0001-urdf-only-robots.md](docs/adr/0001-urdf-only-robots.md) — serial GH robots are URDF-only (path or bundled UR10e Robotiq). [0002](docs/adr/0002-kinematic-tree-in-motus-net.md) — kinematic tree lives in Motus.NET. [0003](docs/adr/0003-parallel-kinematics-stewart.md) — Stewart/Gough (`Family=stewart`) is a Motus.NET sibling stack, not a serial tip chain. [0004](docs/adr/0004-legged-mobile-preview.md) — walking/legged preview (`Family=legged`, radians) in Motus.NET with DOI-cited methods. [0005](docs/adr/0005-general-legged-mechanism.md) — N-leg `LeggedMechanism` + Walk; GH is thin Motus Body + Leg + Mechanism → Walk (+ Terrain Patch); **no Motus Hex**.

## Layout

```
src/Motus.GH/
  Components/     # Plan, Preview, Export (incl. Motus Waypoints), Collision, …
  Data/ Params/   # TrajectoryGoo, Param_Motus*
  Preview/ UI/    # Scrub, ButtonAttributes, FK preview
  Resources/icons/# Phosphor duotone PNGs (embedded)
examples/         # Generated .ghx — never hand-edit; regenerate via node scripts/generate-examples.mjs after Motus GUID/pin/component changes
scripts/          # build helpers, qa-smoke, validate-ghx
```

Build: `./build.sh` (macOS) / `./build.ps1` (Windows). QA: `./scripts/verify-qa.ps1 -Configuration Release -Install`.

After code changes: `graphify update .` (AST graph in `graphify-out/`).

## Motus.NET

Pinned **0.14.0** via [`build/MotusNetPackages.props`](build/MotusNetPackages.props). Default = NuGet (VS-friendly) once published. For close-open-dev / local Motus.NET work, use sibling or in-repo `Motus.NET` via `-p:UseMotusNetProjectReference=true` or `./build.ps1 -UseLocal` ([`build/MotusNetLocal.props`](build/MotusNetLocal.props)); CI checkouts `lasaths/Motus.NET` as a sibling and builds with UseLocal so restore does not depend on nuget.org having the pin yet.

| Package | Role |
|---------|------|
| `Motus.Core` | Models, planners, validation, PlanBundle export |
| `Motus.Geometry` | FK/IK, collision, LIN / industrial motion |
| `Motus.OMPL.NET` | `SamplingPlanner` + registry |
| `Motus.Presets` | URDF/xacro/SRDF loaders |
| `Motus.Native` / `Motus.OMPL.Native` | Optional OMPL/FCL P/Invoke |

Managed (no full native): JointLinear, Cartesian LIN, IndustrialMotion, managed RRT-Connect, C# collision. Stub/NuGet builds often show **only `RrtConnect`** in Motus RRT Settings — expected. Extra sampling planners need Motus.NET native full build. Check Plan `Warnings` → `MotusCapabilities.Describe()`.

Algorithm references and DOI traceability live in Motus.NET [`docs/METHODS.md`](Motus.NET/docs/METHODS.md) (Stewart, legged gait/SSM, mobility, retiming, sampling planners).

## Safety / Plan gate

- Motus Plan defaults to **manual Plan button** (cached re-emit on input edits). **Auto Plan** = debounced replan (~400 ms); verify Status before handing off to controllers.
- `SET` / `WAIT` / `ToolMode` / tool-state = **export hints**, not hardware IO.

## Controller handoff (Motus Waypoints)

Controllers like UR Write MoveJ need `{waypoint → q[n]}` from **Motus Waypoints**.

**Motus Waypoints** (`src/Motus.GH/Components/MotusComponents.cs`):

| Pin | Role |
|-----|------|
| `Tr` | Trajectory (list concatenates) |
| `D` | Decimate every Nth; **keeps first + last** |
| `Q` | Waypoint-major joint tree → MoveJ |
| `P` | FK TCP planes → MoveL only for Cartesian-intent |
| `Tm` | Times (metadata) |

Primary (serial 6R): Plan → Waypoints `Q` → UR Write MoveJ → Run.  
Do **not** MoveL FK planes from joint-space RRT. Gate handoff on **`Preset.Family`**, not bare `AxisCount == 6` — Stewart (`Family=stewart`) `Q` is **leg lengths in meters**, not UR MoveJ radians; legged (`Family=legged`) `Q` is joint **radians** (tip-path or full-driver gait — not UR MoveJ for the whole mechanism). No Play/Session on Motus side.

**Motus Export** JSON/CSV stays for scripts and PlanBundle-style handoff.

## Icons

Phosphor 24×24 duotone PNGs in `src/Motus.GH/Resources/icons/`; tinted in `MotusIcon.cs` / `MotusPalette` by subcategory (Model `#00DB87`, Plan `#787DFA`, Collision peach, Preview lavender, Export `#AFFC41`; chrome `#0A2E33`). Fetch via `.agents/skills/phosphor-icons` CLI:

```bash
node ../phosphor-icons-mcp/dist/cli.js icon path --weight duotone --format png --size 24 --dir src/Motus.GH/Resources/icons
```

Icon name in component ctor maps to `{name}-duotone.png` (e.g. Waypoints → `path`). No Phosphor `mesh` — Collision Mesh uses `polygon`.

## Manual Rhino checks (not covered by qa-smoke)

GitHub-hosted CI **compiles** qa-smoke but **skips the run** (no Rhino 8). Before release / merge of Rhino-touching changes, run locally:

`./scripts/verify-qa.ps1 -Configuration Release -Install`

Also check in Rhino:

- Motus tab visible; Plan button vs Auto Plan; unreachable plane Status
- Preview meshes + Scrub/Play handoff
- Waypoints `Q` tree wires into UR Write MoveJ without GH transpose (serial only)
- Motus Stewart → Plan TCP plane list → Preview scrub; Waypoints warns meters ≠ MoveJ; drag Br/Pr sliders on `08_stewart_tcp_path`
- Joint Table: Tip path Plan works; branching shows warning that side branches are preview-only
- Serial Chain + Reach + Robotiq scrub (TreeFK + ToolParameterBinding)
- `06_turntable_group`: UR beside 1-DOF turntable; GH Center Box → Motus Robot Attach on `turntable_link` (TreeFK); AllDrivers multi-waypoint TCP tracks spoke
- `07_urdf_gripper_tool`: Boxes→ULink→Tool Rd (Cap+Bd)→Robot Tl→PTP Ramp; scrub shows authored fingers pinch
- `09_walking_hexapod`: Body+Leg+Mechanism→Walk; Number Slider `N` (4–12, default 6); Terrain Patch → `Tn`; omit `Tn` = flat Z=0
- Legged Plan gait: Walk `Rb` (Mechanism handle) + Motus Plan ≥2 planes → `PlanBodyPath` full-driver `Tr` (hard SSM); tip joint / 1-plane LIN unchanged
- Example **logic** (not .ghx solve): Motus.NET `Example09_WalkingHexapod_ArcAndBoxTerrain` + qa-smoke “Example 09 walking hex logic”; Three.js stick viz via `Motus.NET/tools/legged-viewer`
