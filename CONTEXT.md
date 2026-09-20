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

**Band layout** / **Pipeline layout**:
Example stages run left→right as **Robot → Env + Traj → Plan → Play**. Each stage is a coloured GH Group with a size-25 Scribble header (left-aligned at the top of the group). Wires stay short and mostly horizontal (align Merge Y with Plan Goal). Gaps between stage AABBs must not overlap.
_Avoid_: overlapping columns, nested groups; stacking Plan under Robot with long vertical wires; NickName-only group labels without scribble headers

**Plan–Scrub–Preview**:
Fixed relative spacing in the example generator: Scrub between Plan and Preview (not stacked above Preview). Deltas from Plan origin: Scrub (+120,+88, w=200), Preview (+420,+9) — sized so live Play/Cap chrome does not collide with Scrub. Examples set Motus Preview `SS`/`ShowStart` on, and UR10e / Motus Robot viewport preview off (`Hidden`), so only Preview draws the robot.
_Avoid_: Scrub overlapping Preview Bounds; double robot preview from UR10e + Preview

**Scribble title**:
A short canvas title (size 28, X≈0, Y≈−69). Not a substitute for component tooltips.
_Avoid_: sticky note, annotation panel; title parked at x=40 or positive Y

**Note scribble**:
A brief one-line hint under the title (size 14, X≈0, Y≈−31). Keep sparse.
_Avoid_: README-on-canvas, long tutorial text, yellow Note Panel for the example blurb

**Layout QA (no Grasshopper)**:
`node scripts/generate-examples.mjs` runs authored Bounds overlap checks and writes SVG maps to `.cassis-audit/layout/*.svg`. Optional Cassis `capture_canvas` / `canvas_snapshot` when Rhino is open.
_Avoid_: hand-nudging groups in Rhino as the source of truth

**Cassis reload**:
Before regenerating or re-opening an example, close every open `.ghx` except the Cassis host (`ListOpenDocuments` → `CloseDocument` with `saveFirst:false` until only `cassis:true` remains). Stale open tabs keep old InstanceGuids and ignore disk.
_Avoid_: `LoadDocument` on top of an already-open copy of the same file

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
