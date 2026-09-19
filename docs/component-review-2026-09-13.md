# Grasshopper component review — 2026-09-13

Scope: all 39 concrete component classes under `src/Motus.GH/Components`, plus the Motus Scrub parameter. Reviewed registrations, defaults, relevant solve paths, shared data types, and the component reference. This is a source and workflow review, not an interactive Rhino usability test. Recommendations below are proposed changes; component behavior has not been modified.

## Verdict

The serial workflow is coherent: Robot → Plan for quick goals, or Move/Pick Place → Program for explicit sequences, then Preview and Export/Waypoints. Keep that structure. The main problems are inconsistent contracts between components and unfinished connections between otherwise useful features. Fix these before expanding the catalogue.

## Functional findings

### 1. Program can treat an old trajectory as current after important input changes

Priority: high. `ProgramFingerprint` includes segment summaries, start joints and collision scene, but not the robot/base/tool, Group, Attach, or Prior trajectory. Motion segment summaries also omit ToolState/ToolMode on PTP/LIN/CIRC and attachment geometry. The idle solve path uses this fingerprint to decide whether to warn or trigger Auto Plan. For example, changing only Prior or the robot base can leave the old trajectory displayed without triggering the intended replan.

Evidence: [fingerprint](../src/Motus.GH/Components/MotusMotionProgramComponents.cs#L830), [stale detection](../src/Motus.GH/Components/MotusMotionProgramComponents.cs#L862), [Prior resolved after fingerprint](../src/Motus.GH/Components/MotusMotionProgramComponents.cs#L937).

Recommendation: fingerprint a complete resolved request, including effective start/tool state from Prior. Share request-change semantics with Plan. Invalid replacement inputs should explicitly invalidate the old result.

### 2. Joint State and authored serial robots do not carry mixed units consistently

Priority: high. Joint State exposes an angle input and converts every value when Degrees is enabled, without a robot input to distinguish revolute and prismatic coordinates. Serial Chain creates every limit with the radians-default `JointLimit` constructor, including its advertised rail. Joint Table's tip-path limit construction does the same. A metre-valued rail can therefore be labelled as radians, and a mixed state supplied in degrees/metres will have the rail value converted too.

Evidence: [Joint State](../src/Motus.GH/Components/MotusComponents.cs#L596), [Serial limits](../src/Motus.GH/Components/MotusSerialChainComponent.cs#L79), [Joint Table limits](../src/Motus.GH/Components/MotusJointTableComponent.cs#L270), [constructor contract](../../Motus.NET/src/Motus.Core/JointLimit.cs#L29).

Recommendation: use per-axis units when constructing limits. Let Joint State optionally accept Robot, validate count/order, and convert only angular axes. Until then, its angle-only presentation should not imply universal support for Stewart or rail coordinates.

### 3. Internalized URDF descriptions lose their actual contents on reload

Priority: high. `RobotDescriptionGoo.Write` stores only name and fingerprint; `Read` does not restore `Value`. Upstream Assemble can rebuild a connected definition, but an internalized description with those upstream wires removed cannot recover its links and joints. Tool goo already persists an embedded mechanism as inline URDF, so these closely related data types behave differently.

Evidence: [description persistence](../src/Motus.GH/Data/MotusUrdfGoo.cs#L33), [tool persistence](../src/Motus.GH/Data/MotusGoo.cs#L96).

Recommendation: persist the complete description with the existing bounded URDF serialization path. Add a save/reload test for internalized description data, not just connected recomputation.

### 4. The URDF authoring workflow has no direct canvas route to Plan

Priority: medium. Link → Joint → Assemble/Attach produces RobotDescription, whereas Plan consumes Robot. There is no concrete component invoking `RobotDescriptionSession.Project`; the authoring keywords and documentation point at that C# API. Tool can consume a description as a tool mechanism, but that does not provide a general authored-arm-to-Plan connection. Export URDF → Robot is a possible disk workaround.

Evidence: [Assemble output and next-step hint](../src/Motus.GH/Components/MotusUrdfAuthoringComponents.cs#L205), [Robot path input](../src/Motus.GH/Components/MotusComponents.cs#L290), [Plan robot input](../src/Motus.GH/Components/MotusPlanComponents.cs#L76).

Recommendation: add Description support to Robot or a focused Description → Robot component with tip/base/tool selection. Prefer completing this connection over another robot builder.

### 5. Reach Samples may sample a different endpoint from the selected robot TCP

Priority: medium. Reach hardcodes `tool0`, falls back to the last tree link, uses preset limits by tree-driver index, and samples a link origin without applying the session tool offset. A robot with a custom tip, side-branch layout, or offset tool can get a reach cloud inconsistent with TCP Pose and planning. Its Seed input is explicitly unused.

Evidence: [tip selection and limits](../src/Motus.GH/Components/MotusReachSamplesComponent.cs#L88).

Recommendation: use the robot's selected tip, name-based driver mapping, and effective tool transform. Remove or implement Seed. Describe the result as sampled reach, not a reachability guarantee.

### 6. Planning Group's pass-through behavior overwrites source metadata

Priority: medium. The component starts from an incoming group's name/base/tip, then calls GetData on inputs with persistent defaults (`manipulator`, `base_link`, `tool0`). Those defaults replace the source metadata even when the user merely wires an SRDF group for pass-through; the incoming joint list can remain, yielding mixed metadata.

Evidence: [defaults and solve](../src/Motus.GH/Components/MotusPlanningContextComponents.cs#L26).

Recommendation: separate pass-through from explicit overrides, or use optional unset override fields.

## Workflow and interface findings

- **Plan is overloaded.** For a legged robot, one plane means tip motion while two planes mean body-path gait; Start/Step cease to apply and flat terrain is assumed. Keep Walk as the explicit gait planner, or expose a clear planning mode. Adding a waypoint should not silently change the meaning of every plane. [PlanWorker](../src/Motus.GH/Planning/PlanWorker.cs#L189).
- **Walk and Plan have different validity policies.** Walk emits trajectories on SSM-only validation failures with runtime warnings, while Plan body-path synthesis treats SSM as a hard failure. A shared trajectory wire should carry an explicit validity/preview-only state. [Walk validation](../src/Motus.GH/Components/MotusWalkingHexapodComponent.cs#L289).
- **Program lacks Plan's cancellation and tuning controls.** Its background request does not supply a cancellation callback; transfer planning hardcodes RRT-Connect and seed 42. Expose cancellation and a shared Sampling Settings input, especially for large pick/place jobs. [request construction](../src/Motus.GH/Components/MotusMotionProgramComponents.cs#L1024).
- **Collision is too hidden on Quick Plan.** It is a primary planning concern but starts behind a context menu. Show Scene by default and state visibly whether collision checking is active. Keep Group/Attach/tuning advanced. [Plan inputs/menu](../src/Motus.GH/Components/MotusPlanComponents.cs#L74).
- **Joint Table is less general than its name suggests.** Rows hardcode axis +Z and limits ±π; the UI provides neither axes nor limits. Expose these or clearly position it as a simplified demonstrator. [row construction](../src/Motus.GH/Components/MotusJointTableComponent.cs#L108).
- **Planes have inconsistent meanings.** TCP planes describe orientation; Body and walking path planes use origins only; URDF Attach accepts translation-only orientation. Name point-only inputs as points or support the full frame. Document world/base/local coordinates consistently. [Body](../src/Motus.GH/Components/MotusLeggedComponents.cs#L147), [Attach](../src/Motus.GH/Components/MotusUrdfAuthoringComponents.cs#L376).
- **URDF Link cannot represent a geometry-free link or explicitly empty collision geometry.** Visual geometry is required and empty Collision means reuse Visual. That obstructs frame-only links and visual-only decorations. Provide an explicit collision policy. [Link](../src/Motus.GH/Components/MotusUrdfAuthoringComponents.cs#L52).
- **Waypoints is a geometric extractor, not a complete program handoff.** It exports Q/P/time but omits tool state, attachment events and primitive type. Decimation preserves endpoints only, so it can remove dwell or segment boundaries. Call this out for Program input, preserve essential events, and recommend the complete export for program consumers. [Waypoints](../src/Motus.GH/Components/MotusComponents.cs#L728).
- **Pick Place is specifically a parallel-jaw, vertical-approach recipe.** Fixed world +Z approach and width-only Open/Close are useful defaults but not generic gripper behavior. Accept Tool State inputs and an approach direction while retaining a simple preset. Touch body names need discoverable robot metadata rather than guessed strings. [Pick Place](../src/Motus.GH/Components/MotusPickPlaceComponents.cs#L110).
- **Default terrain and gait settings disagree.** Terrain Patch Amp defaults to 0.04 m; Walk Lift defaults to 0.02 m despite the tooltips advising Lift ≥ Amp. Align the defaults. [Terrain](../src/Motus.GH/Components/MotusTerrainPatchComponent.cs#L34), [Walk](../src/Motus.GH/Components/MotusWalkingHexapodComponent.cs#L63).
- **Documentation is stale in consequential places.** The reference says example 10 uses a table-only planning scene while AGENTS says empty; it describes RRT TimeLimit as 0 while registration is 30; it mentions a Robot input on Joint State which does not exist. Preview Scene/debug outputs and Export's Prepared Trajectory also need an accurate reference.

## Complete component inventory

“Keep” means the component has a clear role, not that every implementation path is verified.

| Component | Assessment / proposed direction |
|---|---|
| Robot | Keep as main loader; also accept authored Description. Expose frame and driver metadata clearly. |
| UR10e Robotiq | Keep as a convenient example preset; label it as such. |
| Serial Chain | Keep for concept modelling; fix prismatic units and clarify assumed axes/limits. |
| Joint Table | Keep under advanced modelling; expose axes/limits or narrow its advertised scope. |
| Stewart | Keep as a distinct family; make metre-valued state construction straightforward. |
| Joint State | Keep; make count/order/units robot-aware. |
| TCP Pose | Keep; FK is useful independently. Correct base/world wording to match effective base transforms. |
| Tool | Keep; static geometry and actuated description modes are justified but need clearer labels for Cap/Binding/frames. |
| Tool State | Keep; use “Jaw width schema” language and make the unwired Robotiq fallback explicit. |
| Load Mesh | Keep as a small import utility, at secondary exposure. |
| Leg | Keep in a Legged category; clarify analytic 3R versus numerical recipe behavior. |
| Body | Keep; use Points if orientation is intentionally ignored. |
| Mechanism | Keep as walker assembly; “Legged Mechanism” is less ambiguous. |
| Body Pose | Keep advanced; expose modes as discoverable choices and explain clearance relative to automatic mode. |
| Walk | Keep as the explicit gait planner; expose validity and separate robot construction from path-dependent output dimensionality. |
| Terrain Patch | Useful demo utility; secondary exposure and compatible defaults. |
| Reach Samples | Keep after correcting TCP/driver selection; remove unused Seed. |
| URDF Link | Keep; support geometry-free links and explicit collision omission. |
| URDF Joint | Keep; default revolute limits 0..0 create a locked joint. Require or clarify limits; retain axis-line convention. |
| URDF Assemble | Keep; complete persistence and connect output to Robot. |
| URDF Explode | Keep; natural inverse of Assemble. |
| URDF Attach | Keep; a full mount frame is more natural than translation-only Plane. |
| Export URDF | Keep; explicit Write action makes sense for filesystem effects. |
| Plan / Quick | Keep; show collision input and make planning intent explicit. Avoid goal-count-driven mode changes. |
| RRT Settings | Rename Sampling Settings because registry options are broader than RRT; reuse in Program. |
| Move | Keep the compact typed instruction builder. Preserve wires across type changes and make precedence of Type pin/dropdown evident. |
| Program | Keep as explicit sequence planner; fix complete request invalidation and add cancellation/tuning. |
| Pick Place | Keep as a task recipe; name its jaw/vertical assumptions and improve contact-body selection. |
| Planning Group | Keep advanced; fix pass-through override semantics. |
| Attach Body | Keep for an already-carried payload; distinguish this from an attach instruction in a program. |
| Collision Sphere | Keep; natural primitive. |
| Collision Box | Keep; rename misleading “axis-aligned” description since an oriented plane is accepted. Label half dimensions consistently. |
| Collision Boxes | Useful bulk naming/list helper; explain how it differs from native list matching on Box. Reject invalid list entries rather than silently dropping them. |
| Collision Plane | Keep; show free-side direction clearly and make automatic proximal-link exemptions discoverable. |
| Collision Mesh | Keep; explain geometry's local frame and applied placement transform. |
| Collision Scene | Keep as shared scene aggregation; SRDF metadata could be an advanced loader rather than part of basic obstacle setup. |
| Preview | Keep; built-in Play and optional Scrub are coherent. Use the same Scene naming as planners and distinguish visual collision feedback from validity. |
| Scrub | Keep; it earns a custom control through snapping and playback synchronization. Explain that display position is spaced by waypoint, not elapsed time. |
| Waypoints | Keep as extraction; avoid implying complete or revalidated controller-ready program output after decimation. |
| Export | Keep; Prepared Trajectory is valuable for matching the exported clock to preview. Make validation state explicit. |

## Recommended order

1. Fix Program request invalidation, per-axis units, and description persistence.
2. Complete Description → Robot, correct Reach, and fix Planning Group pass-through.
3. Make planning mode, collision state, cancellation, and gait validity consistent.
4. Refresh the component reference/examples and simplify labels/categories without breaking saved GUIDs or wires.

Suggested palette: Model, Tool, Plan, Collision, Preview, Export, plus advanced URDF and Legged groups. Do not merge Plan and Program: quick goals versus explicit motion instructions is a useful distinction. Do not remove Stewart or Walk merely because they are specialized; make their distinct contracts visible.

Verification still needed: Rhino save/reopen of internalized data; Auto Plan after changing only Robot/Prior/Tool/Attach; mixed rail-angle states; custom-tip reach versus TCP Pose; Walk versus Plan validity; Program cancellation; data-tree matching and preview interaction. Earlier builds and core tests passing do not cover these UI contracts.
