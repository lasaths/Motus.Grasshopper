# Motus.Grasshopper

Rhino 8 / Grasshopper plugin for Motus robot motion planning, preview, and export.

Licensed under [MIT](LICENSE).

## Requirements

- Rhino 8 with Grasshopper
- .NET 8 SDK

Motus.NET packages (`Motus.Core`, `Motus.Geometry`, `Motus.Presets`, `Motus.OMPL.NET` **0.5.1**) restore from [nuget.org](https://www.nuget.org/profiles/lasaths).

## Build

```powershell
./build.ps1                      # Release (default)
./build.ps1 -Configuration Debug
./build.ps1 -Zip                 # also writes dist/Motus.Grasshopper-Release.zip
```

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

Component icons use [Phosphor Icons](docs/icons.md) (duotone, 24px) with per-subcategory colors (Model teal, Plan sky, Collision orange, Preview purple, Export amber).

### Environment

| Variable | Default | Purpose |
|----------|---------|---------|
| `Rhino8Dir` | `C:\Program Files\Rhino 8` | Grasshopper DLL hints |
| `MotusNetVersion` | `0.5.1` in `build/MotusNetPackages.props` | NuGet package version pin |

## First workflow

1. **Motus Robot** → pick a preset from the dropdown (e.g. `UR5e`)
2. Drop a Rhino **Plane** for the target → wire to **Motus Plan** `Goal`
3. **Motus Plan** → click **Plan** (`Start` defaults to home). Optional: right-click → **Auto Plan** for reactive replanning on input edits.
4. **Motus Preview** → click **Play** to animate
5. **Motus Export** → `Json` / `Csv`, or **Motus Trajectory Data** for planes/times/joints

That is three nodes for a working, animated plan. `Goal` also accepts a **Motus Joint State** for joint-space targets.
For a full component reference, see [docs/grasshopper-components.md](docs/grasshopper-components.md).

### Motion programs (0.6)

1. **Motus Motion Segment** → build PTP / LIN / CIRC segments (Type dropdown)
2. Wire segments into **Motus Program Plan** → click **Plan**
3. **Motus Preview** / **Motus Export** — trajectories include `motionType`, `segmentIndex`, `blendRadiusMeters`

Unlike **Motus Plan** plane goals, **Program Plan** does not fall back to joint-space paths when LIN fails.

### Collision-aware planning

1. **Motus Collision Sphere** / **Box** / **Mesh** → **Motus Collision Scene**
2. Wire the scene into **Motus Plan** `Collision`:
   - **Plane goal** → true TCP-linear (LIN) planning with collision validation on the path
   - **Joint goal** → RRT-Connect with per-link capsules when the preset includes `collisionLinks`

Optional: wire an SRDF file path into **ColScene** `Srdf` for allowed collision pairs (`examples/srdf/table_base.srdf`).
`ColScene` also outputs SRDF groups (`Groups`) and end-effector map (`EndEffectors`) when present.

Without a collision scene, joint goals use joint-linear interpolation (free space only).

### Attach + planning groups (0.4)

- **Motus Attach Body** creates an attached body from a collision object in TCP-local frame.
- **Motus Planning Group** creates or forwards a `PlanningGroup` (manual joints or SRDF-derived).
- Wire both into **Motus Plan**:
  - `Attach` applies `PlanningContext.Attach(...)` behavior (hide source object while attached).
  - `Group` applies `PlanningContext.ForGroup(...)` so non-group joints stay locked.
- Plan warnings include runtime capability probe text from `MotusCapabilities.Describe()` (managed/native OMPL/FCL status).

Quick pattern with SRDF:

1. `ColSphere/ColBox/ColMesh` → `ColScene.Objects`
2. SRDF file path → `ColScene.Srdf`
3. `ColScene.Scene` → `Plan.Collision`
4. `ColScene.Groups` → `Planning Group` → `Plan.Group`
5. `Attach Body` output(s) → `Plan.Attach`

### Cartesian vs joint goals

- **Plane** `Goal` → TCP-linear LIN motion (`CartesianLinearPathPlanner`).
- **Motus Joint State** `Goal` → joint-space target (RRT when collision scene is wired).

Motus.Grasshopper pins Motus.NET **0.5.1** via `build/MotusNetPackages.props` (NuGet).

## Changelog

### 0.6.1 — Motus.NET 0.5.1 (faster obstacle RRT)

Pins Motus.NET **0.5.1** from NuGet. Faster managed collision checks (`FastDhFk`, xyz sphere path) speed up RRT-Connect obstacle planning on Rhino Win/Mac without native OMPL.

### 0.6.0 — Motion programs

- **Motus Motion Segment** — PTP / LIN / CIRC segment builder with Type dropdown
- **Motus Program Plan** — mixed motion programs via `IndustrialMotionPlanner` (collision, group, attach parity with Motus Plan)
- Example `08_motion_program.ghx`; export includes motion metadata per waypoint

### 0.5.0 — Motus.NET 0.5.0 (UR analytic IK fix)

Pins Motus.NET **0.5.0** from NuGet. Fixes UR TCP-linear (LIN) planning from viewer home poses where analytic IK previously disagreed with forward kinematics (~1.37 m error).

### 0.3.0 — breaking palette redesign

The component palette was consolidated from ~25 components to 10 for simplicity ("no human error, only bad design"): one `Motus Robot`, one `Motus Plan` (planner inferred from inputs, plane or joint goal, home-default start, Plan button), one `Motus Preview` (with Play), plus `Motus Trajectory Data`, `Motus Export`, four collision nodes (sphere, box, mesh, scene), and `Motus Joint State`. Several component GUIDs changed and the preset/robot/cartesian-pose/validate/play nodes were removed or merged, so **definitions saved against 0.2.x must be rewired**.

## External plugins

Motus outputs neutral trajectories. Wire exports manually into UR.RTDE.Grasshopper, VirtualRobot, Robots, or custom scripts. Motus does not depend on those plugins.

## Examples

Eight Grasshopper definitions in `examples/` cover every Motus component — joint/Cartesian planning, collision shapes, SRDF/groups/attach, URDF load, frame overrides, and motion programs. See [examples/README.md](examples/README.md).

Regenerate after component changes: `node scripts/generate-examples.mjs`

Before release, run `./scripts/verify-qa.ps1 -Configuration Release -Install` and [docs/qa-checklist.md](docs/qa-checklist.md) in Rhino 8.

## Safety

Planning and preview only — no robot control. See [docs/safety.md](docs/safety.md).