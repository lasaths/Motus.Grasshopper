# Example Grasshopper Definitions

**Never hand-edit `examples/*.ghx`.** Always change `scripts/generate-examples.mjs`, then regenerate. Hand-edited files go stale after GUID/pin/component changes and get overwritten (or fail validate). This keeps recurring — treat the generator as the only source of truth.

Six lean `.ghx` files cover Motus components and planner inputs. Regenerate after Motus GUID/pin/component changes:

```bash
node scripts/generate-examples.mjs
node scripts/validate-ghx.mjs
```

## Prerequisite: Motus.GH installed

Examples target Motus **0.7.2**. If Grasshopper shows **Unrecognized Objects** for Motus components, the plugin is not loaded (not a bad `.ghx` GUID). Install, then restart Rhino:

```powershell
.\build.ps1 -Configuration Release -Install
# → %APPDATA%\Grasshopper\Libraries\Motus\Motus.GH.gha
```

macOS: `INSTALL=1 ./build.sh`. Confirm a Motus tab appears before reopening an example.

Open any file in Rhino 8 / Grasshopper. Each example uses **Scribble** titles and coloured **Groups**; list inputs are fed through a **Merge** (one wire per pin). **Motus Plan** and **Motus Program** examples ship with **Auto Plan** on. Drag **Motus Scrub** or **Play** on **Motus Preview** after the trajectory appears.

## Example index

| File | What it demonstrates |
|------|----------------------|
| `01_quick_plan.ghx` | Sequential goals via **Merge** (joint + TCP Pose LIN + plane), Preview, Export, Waypoints |
| `02_collision_srdf.ghx` | ColSphere + ColBox via **Merge** → ColScene (SRDF) + Attach + RRT → Plan (Group pin present but unwired; wire Planning Group after Motus.OMPL fix + Rhino restart) |
| `03_urdf_tool_frames.ghx` | Motus Robot URDF + Base + Robotiq Tool (Load Mesh) + Start + Preview ShowStart |
| `04_motion_program.ghx` | PTP + LIN + CIRC + SET via **Merge** → Motus Program → Preview / Export |
| `05_serial_reach.ghx` | Motus Serial Chain (link lengths) → Motus Reach Samples — on-component preview, no Plan |
| `06_turntable_group.ghx` | Turntable+arm URDF: **coupled** Plan (Group unwired) vs **decoupled** Plan (SRDF `arm` group locks turntable) |

## Component coverage

| Component / option | 01 | 02 | 03 | 04 | 05 | 06 |
|--------------------|:--:|:--:|:--:|:--:|:--:|:--:|
| Motus UR10e Robotiq | ✓ | ✓ | | ✓ | | |
| Motus Robot (URDF Path) | | | ✓ | | | ✓ |
| Motus Serial Chain | | | | | ✓ | |
| Motus Reach Samples | | | | | ✓ | |
| Motus Joint Table | | | | | *(docs)* | |
| Motus Joint State | ✓ | ✓ | ✓ | ✓ | | ✓ |
| Motus TCP Pose | ✓ | | | | | |
| Plane goal (Cartesian LIN) | ✓ | | | ✓ | | |
| Motus Move | | | | ✓ | | |
| Motus Program | | | | ✓ | | |
| Motus Plan — Goal list | ✓ | | | | | |
| Motus Plan — Start | ✓ | | ✓ | ✓ | | ✓ |
| Motus Plan — Collision | | ✓ | | | | ✓ |
| Motus Plan — Group | | ✓ | | | | ✓ |
| Motus Plan — Attach | | ✓ | | | | |
| Motus RRT Settings | | ✓ | | | | ✓ |
| Motus Collision Sphere | | ✓ | | | | ✓ |
| Motus Collision Box | | ✓ | | | | |
| Motus Collision Mesh | | *(note)* | | | | |
| Motus Collision Scene | | ✓ | | | | ✓ |
| ColScene SRDF | | ✓ | | | | ✓ |
| Motus Planning Group | | ✓ | | | | ✓ |
| Motus Attach Body | | ✓ | | | | |
| Motus Tool | | | ✓ | | | |
| Motus Load Mesh | | | ✓ | | | |
| Motus Tool State | | | | ✓ | | |
| Motus Preview | ✓ | ✓ | ✓ | ✓ | | ✓ |
| Preview ShowStart | | | ✓ | | | |
| Motus Export | ✓ | | ✓ | ✓ | | |
| Motus Waypoints | ✓ | | | | | |
| Robot Base / Tool override | | | ✓ | | | |

**Col Mesh:** wire any Rhino mesh or Brep into **Motus Collision Mesh** the same way **02** wires sphere + box into **ColScene** `Objects`.

**Degrees** on **Motus Joint State:** right-click the **J** input and toggle **Degrees**; examples use radians by default.

**Plan advanced inputs:** Collision / Group / Attach / RrtSettings are hidden by default. Right-click Motus Plan → Show Collision (etc.), or open **02** which already includes those pins.

**Tool / ToolState:** Motus Tool has an explicit **Cap** dropdown (`None` / `Robotiq2F85`). Motus Tool State accepts **Tool or Robot** on `Tl` (UR10e bundled tool works when you wire the robot). Gripper SET/WAIT uses Motus Program + Motus Move, not Motus Plan.

**Preview:** Trajectory list from Motus Plan concatenates sequential goals. Debug outputs (Index / Invalid / ToolState / Width) are behind right-click → Show debug outputs.

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

### URDF + tool frames (03)

```
URDF Path + Base + Tool (Load Mesh / Cap) → Motus Robot
Start + Goal → Plan [Auto Plan] → Preview (ShowStart) / Export
```

### Motion program (04)

```
Robot + Joint States / Planes / ToolState → Motus Move (PTP/LIN/CIRC/SET ▾) ─┐
                                                                              ├→ Motus Program [Plan] → Preview / Export
Start (optional) ─────────────────────────────────────────────────────────────┘
```

### Serial + Reach (05)

```
Serial Chain (Lengths) → Reach Samples (N≈128)   # on-component Rhino preview; no Plan
```

### Turntable + Group (06)

```
turntable_arm.urdf → Motus Robot
Start / Goal (4 joints: turntable, shoulder, elbow, wrist)
ColSphere (far keep-out) → ColScene → both Plans.Collision   # forces RRT so GroupMap applies
                    ┌→ Plan (no Group)     → Preview   # coupled: turntable moves
Planning Group arm ─┴→ Plan.Group          → Preview   # decoupled: turntable locked
```

Scrub both Previews: coupled rotates the green table; decoupled keeps table fixed while the arm moves.

**Motus Joint Table** is not in these examples (string-list Parent/Child/Type pins). Author in Grasshopper, or extend the generator later.

## SRDF

`examples/srdf/table_base.srdf` disables checks between `link:0` and obstacle `table`, and defines a `manipulator` planning group. In **02**, edit the **Srdf** panel if the relative path does not resolve (ColScene walks up from the Grasshopper working directory, same as Motus Robot Path).

`examples/urdf/turntable_arm.urdf` + `examples/srdf/turntable_arm.srdf` support **06**. The example sets arm joint names on **Motus Planning Group** `J` (Motus `SrdfLoader` still requires a `<chain>` element for SRDF groups).

## URDF preview notes

- `Motus Robot` feeds visual geometry into **Motus Preview** when supported (`box`, `cylinder`, `sphere`, `.stl`).
- `.dae` visuals are skipped; use `*_minimal.urdf` for reliable in-app preview without external meshes.
- URDF assets in `examples/ur10e/` — see that folder’s README. Run `node scripts/fetch-ur10e-assets.mjs` for arm + Robotiq meshes.

## Editing

**Canonical path (only):** edit `scripts/generate-examples.mjs`, then `node scripts/generate-examples.mjs` (+ `validate-ghx`). **Never hand-edit `examples/*.ghx`** — regenerate overwrites them; hand edits go stale after GUID/pin changes.

Layout rules (see also [CONTEXT.md](../CONTEXT.md)): **band layout** (horizontal stages, no overlapping **Groups**); **Scribble** titles in Consolas; **Note panels** stay to one or two lines.

External plugin / controller handoff: [AGENTS.md](../AGENTS.md).
