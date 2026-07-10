# UR10e official description (reference)



Kinematics in `ur10e_minimal.urdf` and `ur10e.urdf` are taken from Universal Robots’ published **ur10e** `default_kinematics.yaml` in the official ROS 2 description package.



## Official source



- Repository: [UniversalRobots/Universal_Robots_ROS2_Description](https://github.com/UniversalRobots/Universal_Robots_ROS2_Description)

- License: BSD-3-Clause (see [LICENSE](LICENSE))

- UR10e config: `config/ur10e/default_kinematics.yaml`



Universal Robots ship **xacro**, not a flat URDF. To generate the full model with meshes (for RViz, Isaac, etc.):



```bash

git clone https://github.com/UniversalRobots/Universal_Robots_ROS2_Description.git

cd Universal_Robots_ROS2_Description

# ROS 2 environment required for $(find ur_description)

ros2 launch ur_description view_ur.launch.py ur_type:=ur10e

```



Or with xacro in a sourced ROS workspace:



```bash

xacro urdf/ur.urdf.xacro ur_type:=ur10e name:=ur10e > ur10e.urdf

```



## Files in this folder



| File | Purpose |

|------|---------|

| `ur10e.urdf` | Full model with official visual (DAE) and collision (STL) meshes |

| `ur10e_minimal.urdf` | Small serial chain + sample collision (planning/import demo) |

| `LICENSE` | BSD-3-Clause from Universal Robots description package |

| `README.md` | This file |



## Motus usage



**Meshes:** run `node scripts/fetch-ur10e-assets.mjs` from the repo root (~10 MB visual DAEs + collision STLs).



**Grasshopper:** pick **UR10e** on **Motus Robot** — bundled JSON preset includes approximate link capsules for collision planning.



Wire `ur10e.urdf` or the minimal file into **Motus Load URDF** (`BaseLink` = `base_link`, `TipLink` = `tool0`). `Motus Preview` uses URDF visual geometry when mesh files are present next to the URDF.



**.NET API:**



```csharp

var urdf = UrdfRobotLoader.Load("ur10e.urdf", new UrdfLoadOptions

{

    BaseLink = "base_link",

    TipLink = "tool0",

    ModelName = "UR10e"

});

var model = urdf.ToModel();

```



The full URDF references local `meshes/ur10e/visual/*.dae` and `meshes/ur10e/collision/*.stl`; fetch them with the script above, or use the minimal file / JSON preset for lightweight planning.

