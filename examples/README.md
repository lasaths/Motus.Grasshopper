# Example Grasshopper Definitions

**Never hand-edit generated `examples/*.ghx`.** Change `scripts/generate-examples.mjs`, then regenerate. Exception: `10_pick_place.ghx` is Cassis-authored (C# tower layout); do not regenerate with `--only=10`. Example `11_aerial_hover.ghx` is generated (`--only=11`).

```bash
node scripts/generate-examples.mjs
node scripts/verify/validate-ghx.mjs
```

## Prerequisite: Motus.GH installed

Examples target the **current Motus.Grasshopper** build (Motus.NET **2.0.0** NuGet pin). If Grasshopper shows **Unrecognized Objects**, the plugin is not loaded — install, then restart Rhino:

```powershell
.\build.ps1 -Configuration Release -Install
# tip / CI: .\build.ps1 -Configuration Release -UseLocal -Install
# → %APPDATA%\Grasshopper\Libraries\Motus\Motus.GH.gha
```

macOS: `INSTALL=1 ./build.sh`. Confirm a **Motus** tab appears before opening an example.

Each file uses **Scribble** titles + note scribbles, coloured **Groups** with size-25 stage headers (**Robot → Env + Traj → Plan → Play**), and list inputs through **Merge**. **Motus Plan** / **Motus Program** examples ship with **Auto Plan** on. Use **Motus Scrub** or Preview **Play** after a trajectory appears.

Layout QA without Rhino: regenerate writes `.cassis-audit/layout/*.svg` and fails on authored Bounds overlaps.
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
| `07_urdf_gripper_tool.ghx` | Author gripper → Tool Rd (Cap=width schema, Bd=j_left) → PTP Ramp pinch |
| `08_stewart_tcp_path.ghx` | Motus Stewart → Plan TCP path (leg lengths in meters) |
| `09_walking_hexapod.ghx` | Body+Leg+Mechanism → Walk; Number Slider `N` (4–12, default 6) |
| `10_pick_place.ghx` | UR10e destack: C# layout (5×4 tower → 5 columns × 4) + Collision Boxes + Pick Place (`Touch=robotiq_2f85`) → one Program; plan ColScene empty; preview ColScene = table+bricks |
| `11_aerial_hover.ghx` | Motus 2.1: free-flyer HolonomicSE3 Start→Goal (WorldXY body); Preview / Export bodyPose; `assets/aerial/free_flyer_box.urdf` (arm pass-off deferred) |

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

Under `examples/assets/` (paths in GHX are relative to `examples/`):

- `assets/srdf/table_base.srdf` — **02** allowed pairs + groups
- `assets/ur10e/` — URDFs; run `node scripts/fetch/fetch-ur10e-assets.mjs` for meshes
- `assets/urdf/turntable_arm.urdf` — turntable-only URDF
- `resources/robots/ur10e_robotiq/ur10e_with_turntable.xacro` — **06** (UR prefab + 1-DOF 8-spoke turntable)

## Editing

**Only:** edit `scripts/generate-examples.mjs`, then regenerate + validate. Layout: pipeline **Robot → Env + Traj → Plan → Play**, size-25 group scribbles, title + note scribbles (see below).

Controller handoff: [AGENTS.md](../AGENTS.md).

## Example layout

**Example definition** — a lean Grasshopper canvas under `examples/` that teaches one Motus workflow end-to-end. Avoid: demo file, sample script, tutorial document.

**Example generator** — `scripts/generate-examples.mjs` is the only source of truth for layout, groups, scribbles, and wires. Hand-edits in Grasshopper are discarded on regenerate (exception: `10_pick_place.ghx`).

**Canvas group** — a coloured GH Group for one stage. Groups must not overlap. Colours match Motus subcategory tints: Model emerald (robot/tool), Plan periwinkle (goals/moves), Collision peach (obstacles/attach/RRT), Preview lavender (plan/program + preview).

**Band / pipeline layout** — stages left→right as **Robot → Env + Traj → Plan → Play**. Each stage is a coloured Group with a size-25 Scribble header. Wires stay short and mostly horizontal. Stage AABBs must not overlap.

**Plan–Scrub–Preview** — Scrub between Plan and Preview (not stacked above Preview). Deltas from Plan origin: Scrub (+120,+88, w=200), Preview (+420,+9). Examples set Motus Preview `SS`/`ShowStart` on, and UR10e / Motus Robot viewport preview off (`Hidden`), so only Preview draws the robot.

**Scribble title** — short canvas title (size 28, X≈0, Y≈−69). **Note scribble** — one-line hint under the title (size 14, X≈0, Y≈−31). Keep sparse; no README-on-canvas.

**Layout QA (no Grasshopper)** — `node scripts/generate-examples.mjs` runs authored Bounds overlap checks and writes SVG maps to `.cassis-audit/layout/*.svg`. Optional Cassis `capture_canvas` / `canvas_snapshot` when Rhino is open.

**Cassis reload** — before regenerating or re-opening an example, close every open `.ghx` except the Cassis host (`ListOpenDocuments` → `CloseDocument` with `saveFirst:false` until only `cassis:true` remains). Stale open tabs keep old InstanceGuids and ignore disk.

**Trajectory** — Motus planned path (joint waypoints + timing) from Motus Plan or Motus Program. **Waypoints export** — controller-oriented joint tree `{waypoint → q[n]}` from Motus Waypoints; do not treat FK TCP planes as MoveL feed for joint-space / RRT paths.
