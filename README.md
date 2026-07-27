# Motus.Grasshopper

Rhino 8 / Grasshopper plugin for [Motus.NET](https://github.com/lasaths/Motus.NET): **thin UI** for robot motion planning, FK preview, and trajectory export.

| This repo | Motus.NET |
|-----------|-----------|
| Grasshopper components, icons, examples | Kinematics, planners, collision, retiming, method DOIs |
| Wires Rhino geometry ↔ Motus types | Host-agnostic .NET libraries ([NuGet 0.13.2](https://www.nuget.org/profiles/lasaths)) |

**Planning and preview only** — no live robot control, no RTDE. MIT ([LICENSE](LICENSE)).

## Contents

- [First plan](#first-plan)
- [What the Motus tab does](#what-the-motus-tab-does)
- [Algorithms and references](#algorithms-and-references)
- [Requirements](#requirements)
- [Install from source](#install-from-source)
- [Common workflows](#common-workflows)
- [Examples](#examples)
- [External plugins and safety](#external-plugins-and-safety)
- [Further docs](#further-docs)

## First plan

1. <img src="src/Motus.GH/Resources/icons/robot-duotone.png" width="20" alt="" /> **UR10e Robotiq** (or <img src="src/Motus.GH/Resources/icons/file-duotone.png" width="20" alt="" /> **Robot**)
2. <img src="src/Motus.GH/Resources/icons/flow-arrow-duotone.png" width="20" alt="" /> **Plan** — wire `Goal`, click **Plan**
3. <img src="src/Motus.GH/Resources/icons/eye-duotone.png" width="20" alt="" /> **Preview** — **Play**; optional <img src="src/Motus.GH/Resources/icons/path-duotone.png" width="20" alt="" /> Waypoints / <img src="src/Motus.GH/Resources/icons/export-duotone.png" width="20" alt="" /> Export

```
Model ──► Plan [Plan] ──► Preview [Play]
              │                │
         ColScene (opt)   Waypoints / Export
```

## What the Motus tab does

Pin-level detail: [docs/grasshopper-components.md](docs/grasshopper-components.md). Icons match Grasshopper Phosphor assets (tinted in-app by palette: Model emerald, Plan periwinkle, Collision peach, Preview lavender, Export lime).

<details>
<summary><strong>Model</strong> — robots, tools, joints (click to expand)</summary>

| | Component | What it does |
|:-:|-----------|--------------|
| <img src="src/Motus.GH/Resources/icons/robot-duotone.png" width="24" alt="" /> | **UR10e Robotiq** | Bundled UR10e + Robotiq 2F-85 (zero-config) |
| <img src="src/Motus.GH/Resources/icons/file-duotone.png" width="24" alt="" /> | **Robot** | Load `.urdf` / `.xacro`; optional Base / Tool |
| <img src="src/Motus.GH/Resources/icons/path-duotone.png" width="24" alt="" /> | **Serial Chain** | Parametric serial / rail+arm from link lengths |
| <img src="src/Motus.GH/Resources/icons/stack-duotone.png" width="24" alt="" /> | **Stewart** | Stewart/Gough hexapod (`Family=stewart`; `Q` = leg lengths in **m**) |
| <img src="src/Motus.GH/Resources/icons/polygon-duotone.png" width="24" alt="" /> | **Walking Hex** | Legged hexapod gait (`Family=legged`; Path → `Tr`) |
| <img src="src/Motus.GH/Resources/icons/tree-structure-duotone.png" width="24" alt="" /> | **Joint Table** | Branched tree; Plan = tip path; optional **SE2** mobility |
| <img src="src/Motus.GH/Resources/icons/circles-three-plus-duotone.png" width="24" alt="" /> | **Reach Samples** | TCP reach overlay samples |
| <img src="src/Motus.GH/Resources/icons/wrench-duotone.png" width="24" alt="" /> | **Tool** | TCP + optional geometry / mechanism Description |
| <img src="src/Motus.GH/Resources/icons/sliders-horizontal-duotone.png" width="24" alt="" /> | **Tool State** | Gripper width / Open-Closed for programs |
| <img src="src/Motus.GH/Resources/icons/download-simple-duotone.png" width="24" alt="" /> | **Load Mesh** | STL for Tool geometry |
| <img src="src/Motus.GH/Resources/icons/gear-six-duotone.png" width="24" alt="" /> | **Joint State** | Joint vector (rad; toggle ° on `J`) |
| <img src="src/Motus.GH/Resources/icons/crosshair-duotone.png" width="24" alt="" /> | **TCP Pose** | FK joints → TCP plane |

**Urdf authoring** (same Model palette): Link / Joint / Assemble / Explode / Attach use `stack`, `gear-six`, `tree-structure`, `list-plus`, `paperclip` — build a mechanism without a file on disk; **Export URDF** writes it out.

</details>

<details>
<summary><strong>Plan</strong> — Quick, RRT, Move, Program (click to expand)</summary>

| | Component | What it does |
|:-:|-----------|--------------|
| <img src="src/Motus.GH/Resources/icons/flow-arrow-duotone.png" width="24" alt="" /> | **Plan** (Quick) | Multi-goal planner — Plane = TCP LIN; joints = joint-linear or RRT |
| <img src="src/Motus.GH/Resources/icons/faders-duotone.png" width="24" alt="" /> | **RRT Settings** | MaxIter / Planner / Step → Plan `RrtSettings` |
| <img src="src/Motus.GH/Resources/icons/line-segments-duotone.png" width="24" alt="" /> | **Move** | One PTP / LIN / CIRC / SET / WAIT line |
| <img src="src/Motus.GH/Resources/icons/stack-duotone.png" width="24" alt="" /> | **Program** | Plan a Motus Move list (industrial motion) |
| <img src="src/Motus.GH/Resources/icons/list-plus-duotone.png" width="24" alt="" /> | **Planning Group** | Lock non-group joints (SRDF / manual) |

| Goal type | Planner |
|-----------|---------|
| Plane | TCP-linear LIN (may fall back to RRT if collision blocks LIN) |
| Joint State, no Collision | Joint-linear |
| Joint State + Collision (or SE2 mobility) | Sampling (RRT-Connect by default) |

</details>

<details>
<summary><strong>Collision</strong> — obstacles and attach (click to expand)</summary>

| | Component | What it does |
|:-:|-----------|--------------|
| <img src="src/Motus.GH/Resources/icons/sphere-duotone.png" width="24" alt="" /> | **ColSphere** | Sphere obstacle (m) |
| <img src="src/Motus.GH/Resources/icons/bounding-box-duotone.png" width="24" alt="" /> | **ColBox** | Box obstacle (half extents, m) |
| <img src="src/Motus.GH/Resources/icons/intersect-square-duotone.png" width="24" alt="" /> | **ColPlane** | Floor / wall half-space |
| <img src="src/Motus.GH/Resources/icons/polygon-duotone.png" width="24" alt="" /> | **ColMesh** | Mesh / Brep obstacle |
| <img src="src/Motus.GH/Resources/icons/circles-three-plus-duotone.png" width="24" alt="" /> | **ColScene** | Merge obstacles (+ optional SRDF) → Plan `Collision` |
| <img src="src/Motus.GH/Resources/icons/paperclip-duotone.png" width="24" alt="" /> | **Attach Body** | Grasped volume in TCP frame → Plan `Attach` |

</details>

<details>
<summary><strong>Preview &amp; Export</strong> — animate and hand off (click to expand)</summary>

| | Component | What it does |
|:-:|-----------|--------------|
| <img src="src/Motus.GH/Resources/icons/eye-duotone.png" width="24" alt="" /> | **Preview** | FK animation — **Play** / Scrub |
| <img src="src/Motus.GH/Resources/icons/path-duotone.png" width="24" alt="" /> | **Waypoints** | `Q` joint trees for controllers (MoveJ); `P` planes; `Tm` times |
| <img src="src/Motus.GH/Resources/icons/export-duotone.png" width="24" alt="" /> | **Export** | JSON / CSV PlanBundle |

**Family handoff:** serial `Q` = radians (UR MoveJ OK). Stewart `Q` = **leg lengths in meters** (not MoveJ). Legged `Q` = radians for the mechanism — not a UR arm MoveJ.

</details>

## Algorithms and references

Solvers live in Motus.NET; Grasshopper only selects and wires them. Full catalog: Motus.NET [`docs/METHODS.md`](https://github.com/lasaths/Motus.NET/blob/master/docs/METHODS.md) · [`REFERENCES.bib`](https://github.com/lasaths/Motus.NET/blob/master/docs/REFERENCES.bib) · snapshot [docs/motus-net/METHODS.md](docs/motus-net/METHODS.md).

<details>
<summary><strong>Method catalog</strong> — area, behavior, citations (click to expand)</summary>

| Area | What Motus does | References |
|------|-----------------|------------|
| URDF / xacro | In-process xacro subset → kinematic tree (`LoadTreeXacro`) | Motus.NET `docs/urdf-import.md` |
| Joint LIN | Free-space joint interpolation + optional collision | Deterministic managed |
| Cartesian LIN | TCP-linear path, IK per sample | Deterministic managed |
| PoE FK / numerical IK | Modern Robotics screws + body Jacobian for URDF serial | Lynch & Park, *Modern Robotics* |
| TOTG retime | Managed TOPP-RA-style retiming (`RetimerAlgorithm.Totg`) | Pham & Pham 2018, DOI [10.1109/TRO.2018.2819195](https://doi.org/10.1109/TRO.2018.2819195) |
| Path constraints | MoveIt-shaped position/orientation checks | Sucan et al. 2012, DOI [10.1109/MRA.2012.2205651](https://doi.org/10.1109/MRA.2012.2205651) |
| RRT-Connect | Default sampling planner (managed; native OMPL optional) | Kuffner & LaValle, ICRA 2000; [OMPL](https://ompl.kavrakilab.org/) |
| PRM* | Managed roadmap sampling | Karaman & Frazzoli 2011, DOI [10.1177/0278364911406761](https://doi.org/10.1177/0278364911406761) |
| CHOMP-lite | Post-process smoother | Zucker et al. 2013, DOI [10.1177/0278364913488805](https://doi.org/10.1177/0278364913488805) |
| Stewart / Gough | Leg-length IK/FK, stroke-space collision/RRT (`Family=stewart`, **meters**) | Merlet; Dasgupta & Mruthyunjaya (see METHODS) |
| Group / tree planning | `PlanningGroup` / `GroupMap` over named drivers | ADR [0002](docs/adr/0002-kinematic-tree-in-motus-net.md) |
| Holonomic SE(2) | Base x/y/yaw appended to sampling (`Joint Table` SE2) | LaValle, *Planning Algorithms* (2006) |
| Legged gait | Duty-cycle gait + SSM gate (`Walk`; `Family=legged`, **radians**) | Lynch & Park; Song & Waldron; McGhee & Frank (see METHODS) |

Stub/NuGet builds often list only **RrtConnect** in Motus RRT Settings — expected. Extra planners need Motus.NET native full build; check Plan `Warnings` → `MotusCapabilities.Describe()`.

</details>

## Requirements

- Rhino 8.19+ + Grasshopper (Windows or macOS) — RhinoCommon/Grasshopper `8.19.25132.1001`
- .NET 8 SDK
- Motus.NET **0.13.2** NuGet (default). Local Motus.NET: `./build.ps1 -UseLocal`

## Install from source

**Windows**

```powershell
./build.ps1                      # Release (NuGet Motus.NET 0.13.2)
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
| `MotusNetVersion` | Override NuGet pin (default `0.13.2`) |

## Common workflows

**Collision-aware:** ColSphere/Box/Mesh → **ColScene** → Plan `Collision` (right-click Plan → **Show Collision** if hidden). Prefer ColMesh over raw Mesh into Plan. Example: `examples/02_collision_srdf.ghx`.

**Motion programs:** Motus Move (PTP/LIN/CIRC/SET/WAIT) → Motus Program → Preview/Export. `SET`/`WAIT`/tool-state are **export hints**, not hardware IO. Example: `04_motion_program.ghx`.

**Tools / attach / groups:** Motus Tool on Robot; Attach Body + Planning Group on Plan; SRDF on ColScene. Examples: `03_urdf_tool_frames.ghx`, `02_collision_srdf.ghx`.

**Parallel / walking:** Motus Stewart → Plan TCP planes (`08_stewart_tcp_path.ghx`). Motus Body+Leg+Mechanism → Walk (`09_walking_hexapod.ghx`; Number Slider `N`) — not full-mechanism Motus Plan.

## Examples

Nine generated definitions in [`examples/`](examples/README.md) (**never hand-edit `.ghx`** — edit `scripts/generate-examples.mjs`, then regenerate):

<details>
<summary><strong>Example index</strong> (01–09)</summary>

| File | Demo |
|------|------|
| `01_quick_plan.ghx` | Multi-goal Plan → Preview / Export / Waypoints |
| `02_collision_srdf.ghx` | ColScene + SRDF + Attach + RRT |
| `03_urdf_tool_frames.ghx` | URDF + Tool frames |
| `04_motion_program.ghx` | PTP/LIN/CIRC/SET Program |
| `05_serial_reach.ghx` | Serial Chain + Reach Samples |
| `06_dkp_group.ghx` | UR + DKP: coupled vs Group-locked Plan |
| `07_urdf_gripper_tool.ghx` | Actuated gripper as Tool Description |
| `08_stewart_tcp_path.ghx` | Stewart TCP Plan (meters) |
| `09_walking_hexapod.ghx` | Walk graph + N slider (default 6, range 4–12) |

</details>

```bash
node scripts/generate-examples.mjs
node scripts/validate-ghx.mjs
```

Before Rhino-touching releases: `./scripts/verify-qa.ps1 -Configuration Release -Install` ([AGENTS.md](AGENTS.md) checklist).

## External plugins and safety

Exports are neutral trajectories. Prefer **Motus Waypoints** `Q` → joint MoveJ for planned paths; `P` → MoveL only for Cartesian-intent LIN. Motus does not connect to or command robots.

## Further docs

| Doc | Contents |
|-----|----------|
| [docs/grasshopper-components.md](docs/grasshopper-components.md) | Every component, pins, planner rules |
| [docs/motus-net/METHODS.md](docs/motus-net/METHODS.md) | Full methods table (API + units + DOI) |
| [docs/citation-audit.md](docs/citation-audit.md) | Citation coverage audit |
| [examples/README.md](examples/README.md) | Example coverage matrix |
| [AGENTS.md](AGENTS.md) | Maintainer / CI / handoff contracts |
| [docs/adr/](docs/adr/) | URDF-only, tree-in-NET, Stewart, legged |
