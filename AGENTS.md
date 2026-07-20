# AGENTS.md

Notes for AI agents and maintainers working on Motus.Grasshopper.

## Boundaries

- **Planning / preview / export only** — no RTDE, no live robot commands, no project reference to UR.RTDE.Grasshopper.
- Execution (Session, Run, waits, ServoJ) lives in downstream control plugins.
- User component reference: [docs/grasshopper-components.md](docs/grasshopper-components.md).
- ADR: [docs/adr/0001-urdf-only-robots.md](docs/adr/0001-urdf-only-robots.md) — GH robots are URDF-only (path or bundled UR10e Robotiq). [0002](docs/adr/0002-kinematic-tree-in-motus-net.md) — kinematic tree lives in Motus.NET.

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

Pinned **0.7.2** via [`build/MotusNetPackages.props`](build/MotusNetPackages.props). Sibling `../Motus.NET` → project refs ([`build/MotusNetLocal.props`](build/MotusNetLocal.props)).

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

Controllers like UR Write MoveJ need `{waypoint → q[n]}` from **Motus Waypoints**.

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
- Waypoints `Q` tree wires into UR Write MoveJ without GH transpose
- Joint Table: Tip path Plan works; branching shows warning that side branches are preview-only
- Serial Chain + Reach + Robotiq scrub (TreeFK + ToolParameterBinding)
