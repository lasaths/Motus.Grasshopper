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

Output: `src/Motus.GH/bin/Release/net8.0/`

Copy to Grasshopper Libraries:

- `Motus.GH.gha`
- `Motus.Core.dll`, `Motus.Geometry.dll`, `Motus.Presets.dll`, `Motus.OMPL.NET.dll`, `Motus.Rhino.dll`
- `resources/robots/`

Verify: `./scripts/verify-install.ps1`

### Environment

| Variable | Default | Purpose |
|----------|---------|---------|
| `Rhino8Dir` | `C:\Program Files\Rhino 8` | Grasshopper DLL hints |
| `MotusNetRoot` | `../Motus.NET/` | Motus.NET DLL location |
| `MotusNetConfiguration` | matches build config | `Debug` or `Release` |

## First workflow

1. **Motus UR Preset** (or KUKA) → model name e.g. `UR5e`
2. **Motus Robot Model** → wrap preset
3. **Motus Joint State** × 2 → start and goal (radians, or set UseDegrees)
4. **Motus Plan Joint Path** → set **Run** = true (or **AutoReplan** for continuous)
5. **Motus Validate Trajectory** / **Motus Trajectory Info**
6. **Motus Preview Robot** / **Motus Preview Trajectory** / **Motus Preview TCP Path**
7. **Motus Trajectory to JSON** / **CSV** / **Joint Lists**

### Collision-aware planning

1. **Motus Collision Sphere** / **Box** → **Motus Collision Scene**
2. **Motus Plan RRT Connect** with scene wired to **Collision**
3. Validate with collision scene on **Motus Validate Trajectory**

### Cartesian goals

1. **Motus Cartesian Pose** from a goal plane
2. **Motus Plan Cartesian Path** with start joint state + pose goal

## External plugins

Motus outputs neutral trajectories. Wire exports manually into UR.RTDE.Grasshopper, VirtualRobot, Robots, or custom scripts. Motus does not depend on those plugins.

See [docs/external-plugin-workflows.md](docs/external-plugin-workflows.md) and [examples/README.md](examples/README.md).

Before release, run [docs/qa-checklist.md](docs/qa-checklist.md) in Rhino 8.

## Safety

Planning and preview only — no robot control. See [docs/safety.md](docs/safety.md).