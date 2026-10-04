# ICD/LIS Bamboo Mobile Robot

URDF model of the bamboo mobile brachiation robot developed for collaborative robotic construction with irregular natural materials.

## Project Overview

**Project:** Task and Motion Planning for Collaborative Robotic Construction with Irregular Materials  
**Team:** Nicolas Kubail Kalousdian, Samuel Leder, Achim Menges, Marc Toussaint  
**Institutions:**  
- Institute for Computational Design and Construction (ICD), University of Stuttgart  
- Learning, Intelligent Systems Lab (LIS), University of Tübingen / Max Planck Institute for Intelligent Systems

The robot brachiates on bamboo bundle structures, using learned control policies to transport bamboo bundles and reach goal positions by leveraging elastic bending behavior of the material.

## Preview Without Grasshopper/Rhino

**Static Web Viewer:** Open `../../12_bamboo_mobile_robot_viewer.html` in any web browser to see the robot in 3D.

Features:
- Interactive 3D view (drag to rotate camera)
- Motion scrubber (slide from start pose to goal pose)
- Animate button (3-second loop)
- Real-time joint angle display
- No network required at runtime (self-contained HTML/JS)

```bash
# From repo root, open the viewer:
open examples/12_bamboo_mobile_robot_viewer.html
# or
firefox examples/12_bamboo_mobile_robot_viewer.html
```

The viewer shows the same start-to-goal motion that the Grasshopper example demonstrates.

## Published Kinematics

### 5-DOF Symmetric Chain

Published in RAL 2022 (Kalousdian et al., "Learning Robotic Manipulation of Natural Materials with Variable Properties for Construction Tasks"):

| Joint | Type | Axis | Range | Description |
|-------|------|------|-------|-------------|
| θ1 | Revolute | Z | [-170°, 170°] | Left wrist (axial rotation) |
| θ2 | Revolute | X | [-90°, 90°] | Left elbow |
| θ3 | Revolute | X | [-135°, 135°] | Shoulder (central pivot) |
| θ4 | Revolute | X | [-90°, 90°] | Right elbow |
| θ5 | Revolute | Z | [-170°, 170°] | Right wrist (axial rotation) |

**Morphology:** Symmetrical — two axial joints on the bases (wrists θ1 and θ5) connect to two revolute joints (elbows θ2 and θ4) that meet at a central revolute joint (shoulder θ3).

**Structure:** Claw → Wrist (Z) → Link → Elbow (X) → Link → Shoulder (X) → Link → Elbow (X) → Link → Wrist (Z) → Claw

### Hardware Specifications

From ACADIA 2021 ("Co-Designing Material-Robot Construction Behaviors"):

- **Height:** ~40 cm
- **Mass:** 2.3 kg  
- **Motors:**
  - Shoulder (θ3) and elbows (θ2, θ4): Dynamixel MX-64 (2:1 and 3:1 gear ratios)
  - Wrists (θ1, θ5) and claws: Dynamixel XL430-W250-T (2:1 gear ratio)
- **Claws:** Variable-diameter interlocking fingers with force-feedback control
- **Safety:** Pawl-and-ratchet mechanism to mechanically lock grip on motor failure
- **Sensors:**
  - Internal: IMU (accelerometer, gyroscope, magnetometer) in each end-effector
  - Internal: Joint encoders in servos
  - External: Multi-camera tracking system for root link position
- **Control:** Raspberry Pi microcomputer
- **Cost:** ~€1,800 (prototype)

## URDF Model

`bamboo_mobile_robot.urdf` represents the 5-DOF published kinematic chain.

### Simplifications and Assumptions

1. **Claws:** Modeled as fixed cylindrical geometry. Actual hardware has variable-diameter adaptive grippers with interlocking fingers that conform to irregular bamboo bundle cross-sections.

2. **Link lengths:** Estimated from ~40 cm total height and visual references in papers. Individual link segment lengths are approximations to achieve published joint structure.

3. **Link geometry:** Simplified boxes and cylinders. Actual hardware has 3D-printed Onyx/ABS chassis with aluminum parts in later prototypes.

4. **Mass/Inertia:** Not specified in URDF (visual/collision geometry only). Published 2.3 kg total mass.

5. **Sensors:** Not represented. Robot uses IMU, joint encoders, and external tracking.

6. **Tool frame (tool0):** Placed at right claw tip. Symmetric design means either claw can act as base or end-effector.

## Usage Context

The robot learns control policies via deep reinforcement learning (PPO algorithm) to:
- **Reach:** Bend the bamboo bundle it's holding to reach goal positions
- **Transport:** React to deformations and spring-back while manipulating bamboo elements
- **Locomote:** Walk along bundle length via hard-coded inverse kinematics

Training uses simulation with automatic domain randomization (ADR) and curriculum learning to handle bamboo's variable mechanical properties (elastic modulus, cross-section, wall thickness) before real-world transfer.

## References

1. Kalousdian, N.K., Łochnicki, G., Hartmann, V.N., Leder, S., Oguz, O.S., Menges, A., Toussaint, M. (2022). "Learning Robotic Manipulation of Natural Materials with Variable Properties for Construction Tasks." *IEEE Robotics and Automation Letters*, 7(2), 5749-5756. https://doi.org/10.1109/LRA.2022.3159288  
   PDF: https://argmin.lis.tu-berlin.de/papers/22-kalousdian-RAL.pdf

2. Łochnicki, G., Kubail Kalousdian, N., Leder, S., Maierhofer, M., Wood, D., Menges, A. (2021). "Co-Designing Material-Robot Construction Behaviors: Teaching distributed robotic systems to leverage active bending for light-touch assembly of bamboo bundle structures." *Realignments: Toward Critical Computation, Proceedings of ACADIA 2021*, 470-479. https://doi.org/10.52842/conf.acadia.2021.470  
   PDF: https://papers.cumincad.org/data/works/att/acadia21_470.pdf

3. ICD Project Page: https://www.icd.uni-stuttgart.de/research/research-projects/task-and-motion-planning-for-collaborative-robotic-construction-with-irregular-materials/

## Usage in Grasshopper

See `12_bamboo_mobile_robot.ghx`:
- Load URDF via Motus Robot
- Plan joint-space motion through 5 DOF
- Preview brachiation postures with Scrub

For multi-robot collaborative assembly scenarios, multiple robot instances would coordinate via task/motion planning (Logic-Geometric Programming + RL policies).

## License / Attribution

URDF model created for demonstration purposes based on published research. Robot design © ICD, University of Stuttgart. Research led by Nicolas Kubail Kalousdian and Dr.-Ing. Samuel Leder.
