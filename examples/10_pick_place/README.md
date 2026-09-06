# Pick-place layout

The layout source embedded in `../10_pick_place.ghx` is mirrored in `layout.cs`.
For an already-open copy of the example, replace the layout C# component source
with `layout.cs`, or reopen the updated GHX after saving any custom edits.
Replan after changing the layout.

The bundled Robotiq closes along Motus TCP Z, which is Rhino TCP plane Y.
Both grasp and place TCP frames therefore align plane Y with the brick short
axis. The previous script aligned plane X instead, placing the jaws along the
80 mm dimension while closing to 40 mm. The corrected quarter-turn roll preserves
the intended brick orientation on placement and leaves the approach axis intact.

Preview: right-click **Show path** to hide the white trajectory line.
Planning: Program's button shows completed/total segments; the message shows the
active segment and elapsed time. The final timing/tool checks have their own phase.
