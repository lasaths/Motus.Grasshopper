# Phosphor component icons

Grasshopper component icons are [Phosphor Icons](https://phosphoricons.com) exported as 24×24 PNG (`duotone`). Each component has a unique icon. Subcategory tint is applied at load time in `MotusIcon.cs` (duotone alpha → primary color).

Source files: `src/Motus.GH/Resources/icons/`

## Subcategory colors

| Subcategory | Color | Hex |
|-------------|-------|-----|
| Model | Teal | `#00c49a` |
| Plan | Sky | `#0ea5e9` |
| Collision | Orange | `#f97316` |
| Preview | Purple | `#a855f7` |
| Export | Amber | `#eab308` |

The **Motus ribbon tab** (16×16) and **plugin assembly** icon (24×24) both use duotone `robot` in brand teal via `MotusIcon.GetCategoryTab()` / `GetAssembly()` in `MotusGhPlugin.cs`.

## Regenerate

```bash
# via phosphor-icons MCP get-icon, or CLI:
# phosphor-icons icon cube export eye --weight duotone --color "#000000" --size 24 --dir src/Motus.GH/Resources/icons
```

Source PNGs are black duotone; `MotusIcon.Recolor` maps alpha to the subcategory primary color.

## Component mapping

| Component | Subcategory | Phosphor icon |
|-----------|-------------|---------------|
| Motus (ribbon tab) | — | robot |
| Motus Robot | Model | file |
| Motus UR10e Robotiq | Model | robot |
| Motus Tool | Model | wrench |
| Motus Load Mesh | Model | download-simple |
| Motus UR10e Robotiq | Model | robot |
| Motus Joint State | Model | gear-six |
| Motus Tool State | Model | sliders-horizontal |
| Motus TCP Pose | Model | crosshair |
| Motus Plan | Plan | flow-arrow |
| Motus Motion Segment | Plan | line-segments |
| Motus Program Plan | Plan | stack |
| Motus Planning Group | Plan | list-plus |
| Motus Collision Sphere | Collision | sphere |
| Motus Collision Box | Collision | bounding-box |
| Motus Collision Mesh | Collision | mesh |
| Motus Collision Scene | Collision | circles-three-plus |
| Motus Attach Body | Collision | paperclip |
| Motus Preview | Preview | eye |
| Motus Scrub (param) | Preview | sliders-horizontal |
| Motus Trajectory Data | Export | grid-four |
| Motus Export | Export | export |
