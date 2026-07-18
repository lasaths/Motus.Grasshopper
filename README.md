# Motus.Grasshopper

Rhino 8 / Grasshopper plugin for Motus robot motion planning, preview, and export.

Licensed under [MIT](LICENSE).

## Requirements

- Rhino 8 with Grasshopper
- .NET 8 SDK

Motus.NET packages (`Motus.Core`, `Motus.Geometry`, `Motus.Presets`, `Motus.OMPL.NET` **0.6.7**) restore from [nuget.org](https://www.nuget.org/profiles/lasaths).

## Build

**Windows (PowerShell):**

```powershell
./build.ps1                      # Release (default)
./build.ps1 -Configuration Debug
./build.ps1 -Zip                 # also writes dist/Motus.Grasshopper-Release.zip
./build.ps1 -Install             # copy to %APPDATA%\Grasshopper\Libraries\Motus
```

Output: `src/Motus.GH/bin/Release/net8.0-windows/`

**macOS (bash):**

```bash
./build.sh              # Release
./build.sh Debug
INSTALL=1 ./build.sh    # copy to ~/Library/Application Support/.../Grasshopper/Libraries/Motus
```

Output: `src/Motus.GH/bin/Release/net8.0/`

The plugin multi-targets `net8.0-windows` (Rhino 8 Windows) and `net8.0` (Rhino 8 Mac) per [McNeel cross-platform guidance](https://developer.rhino3d.com/guides/rhinocommon/moving-to-dotnet-core/). Yak/Rhino pick the matching folder automatically when both are packaged.

Copy to Grasshopper Libraries/Motus:

- `Motus.GH.gha`
- `Motus.Core.dll`, `Motus.Geometry.dll`, `Motus.Presets.dll`, `Motus.OMPL.NET.dll`
- `resources/robots/`

Verify: `./scripts/verify-install.ps1` (Windows)

Component icons use [Phosphor Icons](docs/icons.md) (duotone, 24px) with per-subcategory colors (Model teal, Plan sky, Collision orange, Preview purple, Export amber).

### Environment

| Variable | Default | Purpose |
|----------|---------|---------|
| `Rhino8Dir` | `C:\Program Files\Rhino 8` | Grasshopper DLL hints (Windows) |
| `Rhino8App` | `/Applications/Rhino 8.app` | RhinoCommon / Grasshopper hints (macOS) |
| `MotusNetVersion` | `0.6.7` in `build/MotusNetPackages.props` | NuGet package version pin |

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

`SET`, `WAIT`, `ToolMode`, and tool-state values are execution hints in the exported plan payload. They are not robot commands and are interpreted by downstream control adapters.

Unlike **Motus Plan** plane goals, **Program Plan** does not fall back to joint-space paths when LIN fails.

### Collision-aware planning

1. **Motus Collision Sphere** / **Box** / **Mesh** → **Motus Collision Scene**
2. Wire the scene into **Motus Plan** `Collision`:
   - **Plane goal** → true TCP-linear (LIN) planning with collision validation on the path
   - **Joint goal** → sampling planner (default RRT-Connect) via **Motus RRT Settings** when collision is wired

Optional: wire an SRDF file path into **ColScene** `Srdf` for allowed collision pairs (`examples/srdf/table_base.srdf`).
`ColScene` also outputs SRDF groups (`Groups`) and end-effector map (`EndEffectors`) when present.

Without a collision scene, joint goals use joint-linear interpolation (free space only). Red obstacle previews from **Motus Collision Sphere** are **display-only** until `ColScene` is wired into **Motus Plan** `Collision`.

### Troubleshooting: TCP path through a sphere

See [docs/grasshopper-components.md](docs/grasshopper-components.md#troubleshooting-tcp-path-through-a-sphere) for the full checklist. Short version:

1. Wire `ColSphere` → `ColScene` → `Plan.Collision` (and optionally `Preview.Collision` for orange hit highlights).
2. **Plane goals** → LIN + validate (no obstacle avoidance). Use **Joint State** goals + collision for RRT path deformation (`examples/03_collision_rrt.ghx`).
3. **Motus Preview** TCP path is FK only; it does not imply the planner swept the TCP volume against obstacles.

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

Motus.Grasshopper pins Motus.NET **0.6.7** via `build/MotusNetPackages.props` (NuGet).

## Changelog

### 0.6.7 — Motus.NET 0.6.7 (mesh collision perf + Plan UX)

Pins Motus.NET **0.6.7**. Faster managed mesh collision (local meshes, AABB/envelope filters). Plan uses `CollisionCheckerFactory.GetOrCreate`. Manual mode remarks when inputs change without Plan/Auto Plan. ColBrep uses coarser meshing and warns on dense meshes (&gt;20k tris). Generator skips overwriting hand-tuned `examples/03_collision_rrt.ghx`.

### 0.6.3 — Motus.NET 0.6.3 (sampling planner registry)

Pins Motus.NET **0.6.3**. Registry-driven planner selection: **Motus RRT Settings** dropdown from `SamplingPlannerRegistry.ListAvailable()`; **Motus Plan** uses `SamplingPlanner`. Unavailable planners hidden on stub builds; wired settings fall back to managed RRT-Connect with warning.

### 0.6.1 — Motus.NET 0.6.1 (collision UX + tool state)

Pins Motus.NET **0.6.1** from NuGet. Collision planning UX: Plan warnings when `Collision` is unwired, LIN link-envelope disclaimer, joint-fallback viewport remarks. Preview optional `Collision` input with orange TCP hit segments. Troubleshooting docs for TCP-through-sphere behavior.

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
Execution policy (safety IO, retries, controller handshake, command transport) belongs in the downstream control plugin, not Motus.Grasshopper.

## Examples

Eight Grasshopper definitions in `examples/` cover every Motus component — joint/Cartesian planning, collision shapes, SRDF/groups/attach, URDF load, frame overrides, and motion programs. See [examples/README.md](examples/README.md).

Regenerate after component changes: `node scripts/generate-examples.mjs`

Before release, run `./scripts/verify-qa.ps1 -Configuration Release -Install` and [docs/qa-checklist.md](docs/qa-checklist.md) in Rhino 8.

## Safety

Planning and preview only — no robot control. See [docs/safety.md](docs/safety.md).