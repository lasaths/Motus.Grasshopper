# Motus.Grasshopper

Rhino 8 / Grasshopper plugin for Motus robot motion planning, preview, and export.

Licensed under [MIT](LICENSE).

## Requirements

- Rhino 8 with Grasshopper
- .NET 8 SDK
- [Motus.NET](https://github.com/lasaths/Motus.NET) cloned as sibling: `../Motus.NET`

## Build

Motus.Grasshopper compiles against **pre-built Motus.NET DLLs** (NuGet packaging later).

```powershell
./build.ps1                      # Release (default)
./build.ps1 -Configuration Debug
./build.ps1 -Zip                 # also writes dist/Motus.Grasshopper-Release.zip
```

This builds `../Motus.NET` first, then the plugin.

Output: `src/Motus.GH/bin/Release/net8.0-windows/`

Copy to `%APPDATA%\Grasshopper\Libraries\Motus`:

- `Motus.GH.gha`
- `Motus.Core.dll`, `Motus.Geometry.dll`, `Motus.Presets.dll`, `Motus.OMPL.NET.dll`, `Motus.Rhino.dll`
- `resources/robots/`

Verify: `./scripts/verify-install.ps1`

Install to Grasshopper Libraries (Windows):

```powershell
./build.ps1 -Install
```

Copies to `%APPDATA%\Grasshopper\Libraries\Motus` (e.g. `C:\Users\...\AppData\Roaming\Grasshopper\Libraries\Motus`).

Component icons use [Phosphor Icons](docs/icons.md) (teal `#00c49a`, bold, 24px).

### Environment

| Variable | Default | Purpose |
|----------|---------|---------|
| `Rhino8Dir` | `C:\Program Files\Rhino 8` | Grasshopper DLL hints |
| `MotusNetRoot` | `../Motus.NET/` | Motus.NET DLL location |
| `MotusNetConfiguration` | matches build config | `Debug` or `Release` |

## First workflow

1. **Motus Robot** → pick a preset from the dropdown (e.g. `UR5e`)
2. Drop a Rhino **Plane** for the target → wire to **Motus Plan** `Goal`
3. **Motus Plan** → click the **Plan** button (`Start` defaults to home)
4. **Motus Preview** → click **Play** to animate
5. **Motus Export** → `Json` / `Csv`, or **Motus Trajectory Data** for planes/times/joints

That is three nodes for a working, animated plan. `Goal` also accepts a **Motus Joint State** for joint-space targets.

### Collision-aware planning

1. **Motus Collision Sphere** / **Box** / **Mesh** → **Motus Collision Scene**
2. Wire the scene into **Motus Plan** `Collision`:
   - **Plane goal** → true TCP-linear (LIN) planning with collision validation on the path
   - **Joint goal** → RRT-Connect obstacle avoidance (sphere or mesh checker)

Without a collision scene, joint goals use joint-linear interpolation (free space only).

### Cartesian vs joint goals

- **Plane** `Goal` → TCP-linear LIN motion (`CartesianLinearPathPlanner`).
- **Motus Joint State** `Goal` → joint-space target (RRT when collision scene is wired).

Motus.Grasshopper requires Motus.NET **0.2.0** (see `motus-net.version` in the Motus.NET repo). `./build.ps1` builds the sibling repo first and verifies the version pin.

## Changelog

### 0.3.0 — breaking palette redesign

The component palette was consolidated from ~25 components to 9 for simplicity ("no human error, only bad design"): one `Motus Robot`, one `Motus Plan` (planner inferred from inputs, plane or joint goal, home-default start, Plan button), one `Motus Preview` (with Play), plus `Motus Trajectory Data`, `Motus Export`, the three collision nodes, and `Motus Joint State`. Several component GUIDs changed and the preset/robot/cartesian-pose/validate/play nodes were removed or merged, so **definitions saved against 0.2.x must be rewired**.

## External plugins

Motus outputs neutral trajectories. Wire exports manually into UR.RTDE.Grasshopper, VirtualRobot, Robots, or custom scripts. Motus does not depend on those plugins.

See [docs/external-plugin-workflows.md](docs/external-plugin-workflows.md) and [examples/README.md](examples/README.md).

Before release, run `./scripts/verify-qa.ps1 -Configuration Release -Install` and [docs/qa-checklist.md](docs/qa-checklist.md) in Rhino 8.

## Safety

Planning and preview only — no robot control. See [docs/safety.md](docs/safety.md).