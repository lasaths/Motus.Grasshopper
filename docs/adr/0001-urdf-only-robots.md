# URDF-only robots in Grasshopper

Grasshopper no longer loads bundled JSON presets or a model dropdown. Every robot is defined by URDF: either **Motus Robot** (user-supplied path) or **Motus UR10e Robotiq** (zero-input bundled `resources/robots/ur10e_robotiq/`). Home pose for UR10e is hardcoded; other URDFs default to zeros. `viewer_presets.json` was removed.

We chose this to align the plugin with the web viewer’s URDF path, ship one OOTB robot with full meshes (~10 MB), and draw robots at home pose on the component (meshes + wire fallback) instead of invisible preset capsules.
