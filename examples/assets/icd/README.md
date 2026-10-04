# ICD Collective Robotic Construction Actuator

URDF model of the collective robotic construction actuator developed by Samuel Leder at the Institute for Computational Design and Construction (ICD) / IntCDC, University of Stuttgart.

## System Overview

The actuator is a modular single-axis robotic unit with two opposite-facing grippers that clamp standardized timber struts. Multiple actuators and struts combine to form reconfigurable kinematic chains for collective construction of timber structures.

## Specifications

### Actuator Hardware
- **Dimensions (closed)**: 138 (d) × 179 (w) × 309.5 (h) mm³
- **Main axis**: Single continuous rotational joint (unlimited rotation)
- **Grippers**: Two (upper and lower), opposite-facing
- **Gripper function**: Open/close to grip/release struts; slight lifting capability

### Timber Struts
- **Cross-section**: 50 × 50 mm (standardized)
- **Material**: Softwood (e.g., spruce)
- **Minimum length**: 400 mm (for two-actuator kinematic chain)
- **Surface features**: Milled grooves for gripper contact

### Motion Capabilities
- **Rotation**: Continuous, unlimited (modeled with conservative π rad limit for demo)
- **Gripper state**: Open/close (simplified as fixed in URDF)

## URDF Model

`collective_actuator.urdf` represents the **classic version** of the actuator (single revolute + two grippers). Later 2026 variants include active tilt of the upper gripper; not modeled here.

### Simplifications and Assumptions

1. **Grippers**: Modeled as fixed paddles rather than active open/close mechanisms. Full gripper actuation would require additional revolute joints with mimic constraints (similar to example 07).

2. **Rotation limits**: Hardware has unlimited continuous rotation via slip-ring. URDF joint is `type="continuous"` but demo motion uses conservative 0 → π rad for illustration.

3. **Dimensions**: Body and gripper sizes derived from published actuator dimensions (138×179×309.5 mm³). Gripper paddle size (20×50×50 mm) is estimated to match 50×50 mm strut contact surface. Not all internal mechanical details are represented.

4. **Mass/Inertia**: Not specified (URDF visual/collision geometry only). Published mass and torque specs exist in supplementary materials but are not included in this planning-focused model.

5. **Tool frame (tool0)**: Placed at upper gripper contact point. For kinematic-chain examples with struts, attach geometry would represent the gripped strut segment.

## References

- Leder, S., Weber, R., Wall, A., Pettet, A., Guerrero, N., & Menges, A. (2022). "Leveraging Building Material as Part of the In-Plane Robotic Kinematic System for Collective Construction." *Advanced Intelligent Systems*, PMC9404414. https://pmc.ncbi.nlm.nih.gov/articles/PMC9404414/

- Leder, S. (2025). *Co-Design of Collective Robotic Construction Systems in Architecture.* Dissertation, University of Stuttgart.

- Leder, S., Kim, H., Sitti, M., & Menges, A. (2024). "Enhanced co-design and evaluation of a collective robotic construction system for the assembly of large-scale in-plane timber structures." *Automation in Construction*, 162, 105390.

## Usage in Grasshopper

See `12_icd_collective_actuator.ghx`:
- Load URDF via Motus Robot
- Plan joint-linear rotation (0 → π rad)
- Preview actuator motion with Scrub

For multi-actuator kinematic chains, use Motus Robot Attach to add strut geometry at gripper contact frames.

## License / Attribution

URDF model created for demonstration purposes based on published research. Actuator design © ICD, University of Stuttgart. Research led by Dr.-Ing. Samuel Leder.
