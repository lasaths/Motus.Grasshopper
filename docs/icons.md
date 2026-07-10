# Phosphor component icons

Grasshopper component icons are [Phosphor Icons](https://phosphoricons.com) exported as 24×24 PNG (`bold`, `#00c49a`). Each component has a unique icon.

Source files: `src/Motus.GH/Resources/icons/`

## Regenerate

```bash
# via phosphor-icons MCP get-icon, or CLI:
# phosphor-icons icon cube export eye --weight bold --color "#00c49a" --size 24 --dir src/Motus.GH/Resources/icons
```

The **Motus ribbon tab** icon (and `M` shortcut letter) is registered in `MotusGhPlugin.cs` via `GH_AssemblyPriority` using the `robot` icon.

## Component mapping

| Component | Phosphor icon |
|-----------|---------------|
| Motus (ribbon tab) | robot |
| Motus Robot | cube |
| Motus Load URDF | file |
| Motus Joint State | gear-six |
| Motus Plan | flow-arrow |
| Motus Collision Sphere | sphere |
| Motus Collision Box | bounding-box |
| Motus Collision Scene | circles-three-plus |
| Motus Collision Mesh | mesh |
| Motus Preview | eye |
| Motus Trajectory Data | grid-four |
| Motus Export | export |
