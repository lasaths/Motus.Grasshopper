# Motus.Grasshopper

Rhino 8 / Grasshopper plugin for [Motus.NET](https://github.com/lasaths/Motus.NET): plan robot motion, preview FK, export trajectories.

Planning and preview only — no live robot control. Licensed under [MIT](LICENSE).

## Requirements

- Rhino 8.19+ + Grasshopper (Windows or macOS) — built against RhinoCommon/Grasshopper `8.19.25132.1001` for SR compatibility
- .NET 8 SDK

Pins **Motus.NET 0.6.8** from [nuget.org](https://www.nuget.org/profiles/lasaths) (`build/MotusNetPackages.props`). If `../Motus.NET` is a sibling checkout, the build switches to project references automatically (`build/MotusNetLocal.props`).

What Motus.NET includes (packages, managed planners vs native OMPL, why RRT Settings may list only `RrtConnect`): [docs/motus-net.md](docs/motus-net.md).

## Install from source

**Windows**

```powershell
./build.ps1                      # Release
./build.ps1 -Configuration Debug
./build.ps1 -Zip                 # dist/Motus.Grasshopper-Release.zip
./build.ps1 -Install             # %APPDATA%\Grasshopper\Libraries\Motus
```

**macOS**

```bash
./build.sh              # Release → src/Motus.GH/bin/Release/net8.0/
INSTALL=1 ./build.sh    # ~/Library/Application Support/.../Grasshopper/Libraries/Motus
```

The plugin multi-targets `net8.0-windows` and `net8.0` ([McNeel guidance](https://developer.rhino3d.com/guides/rhinocommon/moving-to-dotnet-core/)). Copy into Libraries/Motus:

- `Motus.GH.gha`
- `Motus.Core.dll`, `Motus.Geometry.dll`, `Motus.Presets.dll`, `Motus.OMPL.NET.dll`
- `resources/robots/`

Verify: `./scripts/verify-install.ps1` (Windows).

| Variable | Purpose |
|----------|---------|
| `RhinoCommonPackageVersion` | NuGet floor for RhinoCommon/Grasshopper (default `8.19.25132.1001`) |
| `Rhino8Dir` | Windows Rhino 8 install (launch / path hints) |
| `Rhino8App` | macOS Rhino 8 `.app` path |
| `MotusNetVersion` | Override NuGet pin (default `0.6.8`) |

## First plan (3 components)

1. **Motus Robot** — pick a preset (e.g. UR10e)
2. **Motus Plan** (nick **Quick**) — wire a Rhino **Plane** or **Motus Joint State** to `Goal` (and optionally `Start`), click **Plan** (`Start` defaults to home). Optional: right-click → **Auto Plan**. Plane goals are TCP LIN only.
3. **Motus Preview** — **Play** to animate; **Motus Export** for JSON/CSV

Component reference: [docs/grasshopper-components.md](docs/grasshopper-components.md). Palette icons: [docs/icons.md](docs/icons.md).

```
Robot ──► Plan [Plan] ──► Preview [Play]
              ▲
         Plane / Joints
```

## Common workflows

### Collision-aware planning

1. **ColSphere** / **ColBox** / **ColMesh** → **ColScene**
2. Wire `ColScene` → **Plan** `Collision` (right-click Plan → **Show Collision** if the pin is hidden)
   - **Plane goal** → TCP LIN + collision validate (no obstacle avoidance)
   - **Joint goal** → sampling planner (RRT-Connect by default); tune with **Motus RRT Settings**

Prefer **ColMesh → ColScene → Plan** over wiring raw Mesh/Brep into Plan. Dense meshes (&gt;20k tris) warn at ColMesh — decimate or use primitives for speed.

Example: `examples/02_collision_srdf.ghx`.

### Motion programs

**Motus Move** (on-component Type dropdown: PTP / LIN / CIRC / SET / WAIT) → **Motus Program** → Preview / Export. Unlike Plan plane goals, Program does not fall back to joint-space when LIN fails. `SET` / `WAIT` / tool-state values are export hints for downstream controllers. See `examples/04_motion_program.ghx`.

### Tools, attach, groups

- **Motus Tool** on Robot for TCP / collision geometry — see `examples/03_urdf_tool_frames.ghx`
- **Attach Body** + **Planning Group** on Plan (enable pins via right-click); SRDF path on **ColScene** for allowed pairs and groups — see `examples/02_collision_srdf.ghx`

### Cartesian vs joint goals

| Type | Motion |
|------|--------|
| Plane goal | TCP-linear LIN |
| Joint State goal | Joint-space (RRT when Collision is wired) |
| Plane start | IK to joints (same multi-seed path as plane goals), then plan |
| Joint State start | Used as-is |

## Examples

Four lean definitions in [`examples/`](examples/README.md) cover planning, collision, SRDF/attach, URDF, tools, and motion programs.

```bash
node scripts/generate-examples.mjs
node scripts/validate-ghx.mjs
```

Before release: `./scripts/verify-qa.ps1 -Configuration Release -Install` and [docs/qa-checklist.md](docs/qa-checklist.md).

## External plugins

Exports are neutral trajectories. Wire them into UR.RTDE.Grasshopper, VirtualRobot, Robots, or your own adapter. Safety IO, retries, and controller transport stay in the control plugin — not Motus.

## Safety

Preview and planning only. See [docs/safety.md](docs/safety.md).
