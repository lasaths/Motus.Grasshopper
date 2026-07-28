# Example Grasshopper Definitions

**Never hand-edit `examples/*.ghx`.** Change `scripts/generate-examples.mjs`, then regenerate. Hand-edited files go stale after GUID/pin/component changes.

```bash
node scripts/generate-examples.mjs
node scripts/validate-ghx.mjs
```

## Prerequisite: Motus.GH installed

Examples target the **current Motus.Grasshopper** build (Motus.NET **0.13.0**). If Grasshopper shows **Unrecognized Objects**, the plugin is not loaded — install, then restart Rhino:

```powershell
.\build.ps1 -Configuration Release -Install
# → %APPDATA%\Grasshopper\Libraries\Motus\Motus.GH.gha
```

macOS: `INSTALL=1 ./build.sh`. Confirm a **Motus** tab appears before opening an example.

Each file uses **Scribble** titles and coloured **Groups**; list inputs go through **Merge**. **Motus Plan** / **Motus Program** examples ship with **Auto Plan** on. Use **Motus Scrub** or Preview **Play** after a trajectory appears.

Component behavior: [docs/grasshopper-components.md](../docs/grasshopper-components.md).

## Example index

| File | What it demonstrates |
|------|----------------------|
| `01_quick_plan.ghx` | Sequential goals (joint + TCP Pose LIN + plane) → Preview, Export, Waypoints |
| `02_collision_srdf.ghx` | ColSphere + ColBox → ColScene (SRDF) + Attach + RRT → Plan |
| `03_urdf_tool_frames.ghx` | Motus Robot URDF + Base + Robotiq Tool (Load Mesh) + Start + Preview ShowStart |
| `04_motion_program.ghx` | PTP + LIN + CIRC + SET → Motus Program → Preview / Export |
| `05_serial_reach.ghx` | Motus Serial Chain → Motus Reach Samples (preview only, no Plan) |
| `06_turntable_group.ghx` | UR10e + turntable: GH fixture box → Robot Attach on turntable_link (TreeFK); TCP tracks spoke |
| `07_urdf_gripper_tool.ghx` | Author Boxes→ULink→Tool Rd (Cap+Bd)→PTP Ramp; scrub fingers pinch |
| `08_stewart_tcp_path.ghx` | Motus Stewart → Plan TCP path (leg lengths in meters) |
| `09_walking_hexapod.ghx` | Body+Leg+Mechanism → Walk; Number Slider `N` (4–12, default 6) |

## Component coverage (01–06 core)

| Component / option | 01 | 02 | 03 | 04 | 05 | 06 |
|--------------------|:--:|:--:|:--:|:--:|:--:|:--:|
| Motus UR10e Robotiq | ✓ | ✓ | | ✓ | | |
| Motus Robot (URDF Path) | | | ✓ | | | ✓ |
| Motus Serial Chain | | | | | ✓ | |
| Motus Reach Samples | | | | | ✓ | |
| Motus Joint State | ✓ | ✓ | ✓ | ✓ | | ✓ |
| Motus TCP Pose | ✓ | | | | | |
| Plane goal (Cartesian LIN) | ✓ | | | ✓ | | |
| Motus Move / Program | | | | ✓ | | |
| Motus Plan — Goal / Start | ✓ | | ✓ | ✓ | | ✓ |
| Motus Plan — Collision / Group / Attach | | ✓ | | | | ✓ |
| Motus RRT Settings | | ✓ | | | | ✓ |
| Motus Collision \* / ColScene | | ✓ | | | | ✓ |
| Motus Tool / Load Mesh | | | ✓ | | | |
| Motus Tool State | | | | ✓ | | |
| Motus Preview / Export / Waypoints | ✓ | ✓ | ✓ | ✓ | | ✓ |

**07–09:** gripper Description tool, Stewart TCP Plan, Body+Leg+Mechanism → Walk gait (+ terrain). See [AGENTS.md](../AGENTS.md) for Rhino manual checks.

**Col Mesh:** wire any Rhino mesh/Brep into **Motus Collision Mesh** the same way **02** wires sphere+box into ColScene.

**Plan advanced pins** (Collision / Group / Attach / RrtSettings) are hidden by default — right-click Motus Plan → Show …, or open **02**.

## Typical flows

### Quick plan (01)

```
UR10e + Start ─┐
Joint State ───┼→ Plan.Goal (list) [Auto Plan] → Preview / Export / Waypoints
TCP Pose ──────┤
Plane ─────────┘
```

### Collision + SRDF (02)

```
ColSphere / ColBox → ColScene (+ SRDF) → Plan.Collision
Joint State → Plan.Goal
Group / Attach / RrtSettings → Plan advanced pins
```

### Stewart (08) / Walk (09)

```
Stewart → Plan (TCP planes) → Preview / Waypoints   # Q = meters
Body + Leg → Mechanism → Walk (Path/Planes [, Terrain]) → Tr → Preview    # not full-body Motus Plan
```

Drag Number Slider `N` (4–12) on **09** to change leg count.

## SRDF / URDF assets

- `examples/srdf/table_base.srdf` — **02** allowed pairs + groups
- `resources/robots/ur10e_robotiq/ur10e_with_turntable.xacro` — **06** (UR prefab + 1-DOF 8-spoke turntable)
- `examples/ur10e/` — run `node scripts/fetch-ur10e-assets.mjs` for meshes

## Editing

**Only:** edit `scripts/generate-examples.mjs`, then regenerate + validate. Layout: band layout, Scribble titles, short Note panels ([CONTEXT.md](../CONTEXT.md) if present).

Controller handoff: [AGENTS.md](../AGENTS.md).
