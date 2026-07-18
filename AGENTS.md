# AGENTS.md

Notes for AI agents and maintainers working on Motus.Grasshopper.

## Boundaries

- **Planning / preview / export only** — no RTDE, no live robot commands, no project reference to UR.RTDE.Grasshopper.
- Execution (Session, Run, waits, ServoJ) lives in downstream control plugins.
- User component reference: [docs/grasshopper-components.md](docs/grasshopper-components.md).
- ADR: [docs/adr/0001-urdf-only-robots.md](docs/adr/0001-urdf-only-robots.md) — GH robots are URDF-only (path or bundled UR10e Robotiq).

## Layout

```
src/Motus.GH/
  Components/     # Plan, Preview, Export (incl. Motus Waypoints), Collision, …
  Data/ Params/   # TrajectoryGoo, Param_Motus*
  Preview/ UI/    # Scrub, ButtonAttributes, FK preview
  Resources/icons/# Phosphor duotone PNGs (embedded)
examples/         # Generated .ghx — scripts/generate-examples.mjs
scripts/          # build helpers, qa-smoke, validate-ghx
```

Build: `./build.sh` (macOS) / `./build.ps1` (Windows). QA: `./scripts/verify-qa.ps1 -Configuration Release -Install`.

After code changes: `graphify update .` (AST graph in `graphify-out/`).

## Motus.NET

Pinned **0.6.8** via [`build/MotusNetPackages.props`](build/MotusNetPackages.props). Sibling `../Motus.NET` → project refs ([`build/MotusNetLocal.props`](build/MotusNetLocal.props)).

| Package | Role |
|---------|------|
| `Motus.Core` | Models, planners, validation, PlanBundle export |
| `Motus.Geometry` | FK/IK, collision, LIN / industrial motion |
| `Motus.OMPL.NET` | `SamplingPlanner` + registry |
| `Motus.Presets` | URDF/xacro/SRDF loaders |
| `Motus.Native` / `Motus.OMPL.Native` | Optional OMPL/FCL P/Invoke |

Managed (no full native): JointLinear, Cartesian LIN, IndustrialMotion, managed RRT-Connect, C# collision. Stub/NuGet builds often show **only `RrtConnect`** in Motus RRT Settings — expected. Extra sampling planners need Motus.NET native full build. Check Plan `Warnings` → `MotusCapabilities.Describe()`.

## Safety / Plan gate

- Motus Plan defaults to **manual Plan button** (cached re-emit on input edits). **Auto Plan** = debounced replan (~400 ms); verify Status before handing off to controllers.
- `SET` / `WAIT` / `ToolMode` / tool-state = **export hints**, not hardware IO.

## Controller handoff (Motus Waypoints)

`Motus Trajectory Data` joints are `{axis → waypoints}`. Controllers like UR Write MoveJ need `{waypoint → q[n]}`.

**Motus Waypoints** (`src/Motus.GH/Components/MotusComponents.cs`):

| Pin | Role |
|-----|------|
| `Tr` | Trajectory (list concatenates) |
| `D` | Decimate every Nth; **keeps first + last** |
| `Q` | Waypoint-major joint tree → MoveJ |
| `P` | FK TCP planes → MoveL only for Cartesian-intent |
| `Tm` | Times (metadata) |

Primary: Plan → Waypoints `Q` → UR Write MoveJ → Run.  
Do **not** MoveL FK planes from joint-space RRT. Warns if `AxisCount ≠ 6`. No Play/Session on Motus side.

Trajectory Data / Export JSON-CSV stay for parametric graphs and scripts.

## Icons

Phosphor 24×24 duotone PNGs in `src/Motus.GH/Resources/icons/`; tinted in `MotusIcon.cs` by subcategory (Model teal, Plan sky, Collision orange, Preview purple, Export amber). Fetch example:

```bash
curl -fsSL "https://raw.githubusercontent.com/phosphor-icons/core/main/assets/duotone/path-duotone.svg" -o /tmp/i.svg
rsvg-convert -w 24 -h 24 /tmp/i.svg -o src/Motus.GH/Resources/icons/path-duotone.png
```

Icon name in component ctor maps to `{name}-duotone.png` (e.g. Waypoints → `path`).

## Manual Rhino checks (not covered by qa-smoke)

- Motus tab visible; Plan button vs Auto Plan; unreachable plane Status
- Preview meshes + Scrub/Play handoff
- Waypoints `Q` tree wires into UR Write MoveJ without GH transpose
