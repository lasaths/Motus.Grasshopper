# Preview



Motus uses **DH forward kinematics** for Rhino viewport preview when the robot preset has a kinematics profile. URDF-loaded robots use chain FK via `KinematicsResolver`.



## Motus Preview



A single component handles all preview, with a built-in **Play / Stop** button for timed playback. Inputs: a `Trajectory`, optional `ShowStart`, and optional **Position** (0–1) from **Motus Scrub**. Outputs:



| Output | Geometry |

|--------|----------|

| Meshes | URDF/tool triangle meshes only (no link-axis fallback) |

| Links | FK link lines at the current frame |

| TCP Path | TCP polyline along the whole trajectory |

| State / Time / Index | Joint state, elapsed time, waypoint index at the playhead |

| Invalid | TCP segments that fail joint/velocity/acceleration limits |



## Preview colours

Right-click **Motus Preview** to choose how link meshes are tinted in the Rhino viewport:

| Mode | Behaviour |
|------|-----------|
| **Override colours** (default) | Motus teal for the current pose; light gray ghost for **ShowStart** |
| **URDF colours** | Colours from URDF `<material>` / `<color rgba="…">` tags (one per visual). Meshes without URDF materials use neutral gray. |
| **Custom colours** | Per-mesh colours from an optional **Custom Colours** input (Grasshopper colour list). Order matches the **Meshes** output. Short lists repeat the last colour. |

The **Custom Colours** input is hidden by default. Enable it from the component menu: **Show custom colours input**.

Viewport tinting does not modify **Meshes** output geometry (untinted Rhino meshes, same as before).

TCP path and invalid-segment wire colours are unchanged.



## Motus Scrub



**Motus Scrub** is a dedicated canvas parameter (like a Number Slider) locked to **0–1**. Drag the thumb to scrub; **resize wider** for finer control along long trajectories. Wire its output to **Motus Preview → Position**.

When wired to Preview with a solved trajectory, Scrub reads waypoint times from the preview graph and shows **keyframe ticks** on the track. Dragging **magnetically snaps** to nearby keyframes (toggle via right-click **Snap to keyframes**). Use **Previous keyframe** / **Next keyframe** to step discretely.

The header readout shows `65% · 2.40 / 3.70 s · keyframe 4/12` when a trajectory is available.



While dragging, preview updates in the viewport without forcing a full graph solve; releasing the mouse completes the solve and updates outputs.



Right-click **Reset to start (0)** or **Reset to end (1)** for quick jumps.



## Playback



Click **Play** on **Motus Preview** to animate from the current position; outputs (`Meshes`, `Links`, `State`, `Time`, `Index`) follow the playhead. Click **Stop** to pause.



**Manual scrub pauses play.** When play is running, the wired **Motus Scrub** thumb advances in sync. Replacing the trajectory resets position to **0**.



When stopped (and not dragging), frame outputs reflect the current normalized position via continuous `AtTime` interpolation (not discrete waypoint stepping).



## Frames and units



- Base frame from the robot preset or optional `Base` on **Motus Robot**. Tool from preset, or **Motus Tool** wired to `Robot.Tool`.

- Trajectory preview/export uses the **tool snapshot** stored at plan time when present.

- Keep Rhino document units in **meters**.

- For production visualization, export trajectory data to VirtualRobot or Robots.



## URDF visual geometry



- `Motus Robot` forwards URDF `<visual>` geometry to preview on the component and via **Motus Preview**.

- Supported visual types: `box`, `cylinder`, `sphere`, and mesh `.stl`.

- Unsupported visual mesh formats (for example `.dae`) are skipped and preview falls back to simplified FK geometry.

## Manual verification (Rhino)

After loading the plugin, confirm in Grasshopper:

- **Resize**: widen Motus Scrub for finer thumb control on long trajectories.
- **Scrub vs move**: dragging the thumb updates viewport only until mouse-up (full solve on release).
- **Play handoff**: Play advances from current scrub position; dragging scrub while playing pauses Play.
- **Outputs**: `Time` and `Index` match the scrub position when stopped (continuous `AtTime`, not waypoint snapping).
- **Persistence**: save/reload retains scrub value and preview position.


