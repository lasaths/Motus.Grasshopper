# Motus.Grasshopper

Grasshopper-facing planning, preview, and export for Motus robots — no live robot control.

## Language

### Examples

**Example definition**:
A lean Grasshopper canvas under `examples/` that teaches one Motus workflow end-to-end.
_Avoid_: demo file, sample script, tutorial document

**Example generator**:
`scripts/generate-examples.mjs` — the only source of truth for example layout, groups, scribbles, and wires. Hand-edits in Grasshopper are discarded on regenerate.
_Avoid_: saving over `.ghx` from Rhino as the canonical edit path

**Canvas group**:
A coloured GH Group that owns one stage of an example. Groups must not overlap. Colours match Motus subcategory tints: Model emerald (robot/tool), Plan periwinkle (goals/moves), Collision peach (obstacles/attach/RRT), Preview lavender (plan/program + preview).
_Avoid_: cluster, region (unless meaning GH Cluster); one-off group colours per example

**Band layout**:
Example stages stack as horizontal bands with a clear vertical gap; wires still run left→right inside each band. Plan/Preview sit in the rightmost band. Example 04 uses one band (and group) per move type in program order: PTP → LIN → CIRC → SET → Merge → Program.
_Avoid_: overlapping columns, nested groups; one giant Moves blob with long vertical tool-state wires

**Plan–Scrub–Preview**:
Fixed relative spacing in the example generator: Scrub between Plan and Preview (not stacked above Preview). Deltas from Plan origin: Scrub (+132,+76), Preview (+373,+9). Examples set Motus Preview `SS`/`ShowStart` on, and UR10e / Motus Robot viewport preview off (`Hidden`), so only Preview draws the robot.
_Avoid_: Scrub overlapping Plan pins; double robot preview from UR10e + Preview

**Scribble title**:
A short canvas title for an example (mono font). Not a substitute for component tooltips.
_Avoid_: sticky note, annotation panel (those are **Note panel**)

**Note panel**:
A brief yellow Panel with a one- or two-line hint for the user. Keep sparse.
_Avoid_: README-on-canvas, long tutorial text

### Planning handoff

**Trajectory**:
A Motus planned path (joint waypoints + timing) produced by Motus Plan or Motus Program.
_Avoid_: path (ambiguous with TCP path curve), motion program (that is the Move sequence)

**Waypoints export**:
Controller-oriented joint tree `{waypoint → q[n]}` from Motus Waypoints.
_Avoid_: treating FK TCP planes as a MoveL feed for joint-space / RRT paths

## Example dialogue

> **Dev:** Should I nudge the Obstacles group in Rhino so it stops overlapping Attach?
> **Maintainer:** No — fix coordinates in the **example generator**, regenerate the **example definition**, then reopen. Use **band layout**; canvas groups must not overlap.
