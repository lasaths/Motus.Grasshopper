# Motus.Grasshopper

Rhino 8 / Grasshopper plugin for [Motus.NET](https://github.com/lasaths/Motus.NET): plan robot motion, preview FK, export trajectories.

Planning and preview only — no live robot control. Licensed under [MIT](LICENSE).

## Requirements

- Rhino 8 + Grasshopper (Windows or macOS)
- .NET 8 SDK

Pins **Motus.NET 0.6.7** from [nuget.org](https://www.nuget.org/profiles/lasaths) (`build/MotusNetPackages.props`). If `../Motus.NET` is a sibling checkout, the build switches to project references automatically (`build/MotusNetLocal.props`).

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
| `Rhino8Dir` | Windows Rhino 8 install (DLL hints) |
| `Rhino8App` | macOS Rhino 8 `.app` path |
| `MotusNetVersion` | Override NuGet pin (default `0.6.7`) |

## First plan (3 components)

1. **Motus Robot** — pick a preset (e.g. UR10e)
2. **Motus Plan** — wire a Rhino **Plane** or **Motus Joint State** to `Goal`, click **Plan** (`Start` defaults to home). Optional: right-click → **Auto Plan**
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

Example: `examples/03_collision_rrt.ghx`.

### Motion programs

**Motion Segment** (PTP / LIN / CIRC) → **Program Plan** → Preview / Export. Unlike Plan plane goals, Program Plan does not fall back to joint-space when LIN fails. `SET` / `WAIT` / tool-state values are export hints for downstream controllers.

### Tools, attach, groups

- **Motus Tool** on Robot for TCP / collision geometry (e.g. Robotiq examples 09–11)
- **Attach Body** + **Planning Group** on Plan (enable pins via right-click); SRDF path on **ColScene** for allowed pairs and groups — see `examples/05_srdf_group_attach.ghx`

### Cartesian vs joint goals

| Goal type | Motion |
|-----------|--------|
| Plane | TCP-linear LIN |
| Joint State | Joint-space (RRT when Collision is wired) |

## Examples

Twelve definitions in [`examples/`](examples/README.md) cover planning, collision, SRDF/attach, URDF, tools, and motion programs.

```bash
node scripts/generate-examples.mjs
node scripts/validate-ghx.mjs
```

Before release: `./scripts/verify-qa.ps1 -Configuration Release -Install` and [docs/qa-checklist.md](docs/qa-checklist.md).

## External plugins

Exports are neutral trajectories. Wire them into UR.RTDE.Grasshopper, VirtualRobot, Robots, or your own adapter. Safety IO, retries, and controller transport stay in the control plugin — not Motus.

## Safety

Preview and planning only. See [docs/safety.md](docs/safety.md).
