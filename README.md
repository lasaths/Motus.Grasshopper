# Motus.Grasshopper

Rhino 8 / Grasshopper plugin for Motus robot motion planning, preview, and export.

## Requirements

- Rhino 8 with Grasshopper
- .NET 8 SDK
- Built [Motus.NET](../Motus.NET) sibling repository

## Build

```bash
dotnet build Motus.Grasshopper.slnx
# or on Windows:
./build.ps1
```

Output: `src/Motus.GH/bin/Debug/net8.0/Motus.GH.gha`

Copy `Motus.GH.gha`, `Motus.*.dll`, and the `resources/` folder into your Grasshopper libraries folder, or load via Grasshopper Developer > Manage Grasshopper Libraries.

Grasshopper and GH_IO references point to the default Rhino 8 install path (`C:\Program Files\Rhino 8\...`).

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

## Safety

Planning and preview only — no robot control. See [docs/safety.md](docs/safety.md).
