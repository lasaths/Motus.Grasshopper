# Motus.Grasshopper

Rhino 8 / Grasshopper plugin for [Motus.NET](https://github.com/lasaths/Motus.NET): **thin UI** for robot motion planning, FK preview, and trajectory export.

| This repo | Motus.NET |
|-----------|-----------|
| Grasshopper components, icons, examples | Kinematics, planners, collision, retiming, method DOIs |
| Wires Rhino geometry ↔ Motus types | Host-agnostic .NET libraries (NuGet) |

**Planning and preview only** — no live robot control, no RTDE. MIT ([LICENSE](LICENSE)).

Pins **Motus.NET 0.12.0** from [nuget.org](https://www.nuget.org/profiles/lasaths) (`build/MotusNetPackages.props`). Local Motus.NET work: `./build.ps1 -UseLocal` (sibling checkout). Algorithm catalog: Motus.NET [`docs/METHODS.md`](https://github.com/lasaths/Motus.NET/blob/master/docs/METHODS.md).

## What the Motus tab does

Full pin/behavior reference: **[docs/grasshopper-components.md](docs/grasshopper-components.md)**.

| Palette group | Components | Job |
|---------------|------------|-----|
| **Model** | UR10e Robotiq, Robot, Serial Chain, Stewart, WalkHex, Joint Table, Reach, Tool, Tool State, Load Mesh, Joint State, TCP Pose | Build / load a robot (`Family`: serial, `stewart`, or `legged`) |
| **Urdf** | Link, Joint, Assemble, Explode, Attach, Export URDF | Author a mechanism in GH without a file on disk |
| **Plan** | Plan (Quick), RRT Settings, Move, Program, Planning Group, Attach Body | Solve trajectories (LIN / joint-linear / RRT / motion program) |
| **Collision** | ColSphere, ColBox, ColPlane, ColMesh, ColScene | Obstacles → wire `ColScene` into Plan `Collision` |
| **Preview** | Preview, Scrub | Animate FK meshes / scrub time |
| **Export** | Waypoints, Export | `Q` trees for controllers, or JSON/CSV PlanBundle |

```
Model ──► Plan [Plan] ──► Preview [Play]
              │                │
         ColScene (opt)   Waypoints / Export
```

**First plan:** Motus Robot (or UR10e) → Motus Plan (`Goal` = Plane or Joint State) → click **Plan** → Motus Preview **Play**. Optional: Motus Export / Motus Waypoints.

| Goal type | Planner |
|-----------|---------|
| Plane | TCP-linear LIN (may fall back to RRT if collision blocks LIN) |
| Joint State, no Collision | Joint-linear |
| Joint State + Collision (or SE2 mobility) | Sampling (RRT-Connect by default) |

**Family handoff:** serial `Q` = radians (UR MoveJ OK). Stewart `Q` = **leg lengths in meters** (not MoveJ). Legged `Q` = radians for the mechanism — not a UR arm MoveJ. Details: [AGENTS.md](AGENTS.md).

## Requirements

- Rhino 8.19+ + Grasshopper (Windows or macOS) — RhinoCommon/Grasshopper `8.19.25132.1001`
- .NET 8 SDK

## Install from source

**Windows**

```powershell
./build.ps1                      # Release (NuGet Motus.NET 0.12.0)
./build.ps1 -UseLocal            # sibling Motus.NET project refs
./build.ps1 -Zip                 # dist/Motus.Grasshopper-Release.zip
./build.ps1 -Yak                 # dist/motus-*-rh8_*-any.yak
./build.ps1 -Install             # %APPDATA%\Grasshopper\Libraries\Motus
```

**macOS**

```bash
./build.sh              # Release → src/Motus.GH/bin/Release/net8.0/
INSTALL=1 ./build.sh
```

Libraries folder needs `Motus.GH.gha`, Motus.*.dll, and `resources/robots/`. Verify: `./scripts/verify-install.ps1` (Windows).

| Variable | Purpose |
|----------|---------|
| `RhinoCommonPackageVersion` | RhinoCommon/Grasshopper NuGet floor |
| `Rhino8Dir` / `Rhino8App` | Rhino 8 install hints |
| `MotusNetVersion` | Override NuGet pin (default `0.12.0`) |

## Common workflows

**Collision-aware:** ColSphere/Box/Mesh → **ColScene** → Plan `Collision` (right-click Plan → **Show Collision** if hidden). Prefer ColMesh over raw Mesh into Plan. Example: `examples/02_collision_srdf.ghx`.

**Motion programs:** Motus Move (PTP/LIN/CIRC/SET/WAIT) → Motus Program → Preview/Export. `SET`/`WAIT`/tool-state are **export hints**, not hardware IO. Example: `04_motion_program.ghx`.

**Tools / attach / groups:** Motus Tool on Robot; Attach Body + Planning Group on Plan; SRDF on ColScene. Examples: `03_urdf_tool_frames.ghx`, `02_collision_srdf.ghx`.

**Parallel / walking:** Motus Stewart → Plan TCP planes (`08_stewart_tcp_path.ghx`). Motus WalkHex → gait Trajectory (`09_walking_hexapod.ghx`) — not full-mechanism Motus Plan.

## Examples

Nine generated definitions in [`examples/`](examples/README.md) (**never hand-edit `.ghx`** — edit `scripts/generate-examples.mjs`, then regenerate):

```bash
node scripts/generate-examples.mjs
node scripts/validate-ghx.mjs
```

Before Rhino-touching releases: `./scripts/verify-qa.ps1 -Configuration Release -Install` ([AGENTS.md](AGENTS.md) checklist).

## Docs map

| Doc | Contents |
|-----|----------|
| [docs/grasshopper-components.md](docs/grasshopper-components.md) | Every component, pins, planner rules |
| [examples/README.md](examples/README.md) | Example index + coverage matrix |
| [AGENTS.md](AGENTS.md) | Maintainer / CI / handoff contracts |
| [docs/adr/](docs/adr/) | URDF-only, tree-in-NET, Stewart, legged |

## External plugins & safety

Exports are neutral trajectories. Prefer **Motus Waypoints** `Q` → joint MoveJ for planned paths; `P` → MoveL only for Cartesian-intent LIN. Motus does not connect to or command robots.
