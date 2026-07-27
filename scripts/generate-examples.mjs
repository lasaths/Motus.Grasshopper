#!/usr/bin/env node
/**
 * Regenerate Motus Grasshopper example .ghx files from graph specs.
 * Run from repo root: node scripts/generate-examples.mjs
 */
import crypto from 'node:crypto';
import fs from 'node:fs';
import path from 'node:path';
import { fileURLToPath } from 'node:url';

const __dirname = path.dirname(fileURLToPath(import.meta.url));
const repoRoot = path.resolve(__dirname, '..');
const outDir = path.resolve(repoRoot, 'examples');
const MOTUS_LIB = 'dc547e55-81a8-c313-e25d-e1468ddecddb';
const csproj = fs.readFileSync(path.resolve(repoRoot, 'src/Motus.GH/Motus.GH.csproj'), 'utf8');
const props = fs.readFileSync(path.resolve(repoRoot, 'build/MotusNetPackages.props'), 'utf8');
const MOTUS_NET_VERSION = props.match(/<MotusNetVersion[^>]*>([^<]+)<\/MotusNetVersion>/)?.[1]?.trim() ?? '0.6.6';
const PLUGIN_VERSION = csproj.match(/<Version>([^<]+)<\/Version>/)?.[1] ?? MOTUS_NET_VERSION;
const PLUGIN_ASSEMBLY_VERSION = `${PLUGIN_VERSION}.0`;
const absPath = (...parts) => path.resolve(repoRoot, ...parts);

const GOAL_JOINTS = [1.2, -1, 1.2, -1.6, -1.5708, 0];
const START_JOINTS = [0, -1.2, 1.2, -1.6, -1.5708, 0];
const MOTION_START = [0, -0.5, 1.0, -1.0, 0.0, 0.0];
/** Walking hex right-middle tip leg: hip, femur, tibia (rad) at default stance. */
const HEX_TIP_START = [-0.1309, 0.5236, -0.5236];
const HEX_TIP_GOAL = [-0.1309, 0.6109, -0.5236];
/** Collision-free home-ish start for obstacle demos (away from table/box). */
const COLLISION_START = [0.0, -1.4, 1.4, -1.7, -1.5708, 0.0];
const COLLISION_GOAL = [1.0, -0.9, 1.0, -1.4, -1.5708, 0.3];
/** ur10e_on_turntable.xacro: [turntable, …UR10e×6] — goal rotates table. */
const TURNTABLE_START = [0.0, ...START_JOINTS];
const TURNTABLE_GOAL = [1.2, ...GOAL_JOINTS];

/** GH / Motus param type GUIDs (ComponentGuid). Required for IGH_VariableParameterComponent ParameterData. */
const PTYPE = {
  generic: '8ec86459-bf01-4409-baee-174d0d2b13d0',
  number: '3e8ca6be-fda8-4aaf-b5c0-3c54c8bb7312',
  integer: '2e3ab970-8545-46bb-836c-1c11e5610bce',
  string: '3ede854e-c753-40eb-84cb-b48008f14fd4',
  boolean: 'cb95db89-6165-43b6-9c41-5702bc5bf137',
  mesh: '1e936df3-0eea-4246-8549-514cb8862b7a',
  line: '8529dbdf-9b6f-42e9-8e1f-c7a2bde56a70',
  curve: 'd5967b9f-e8ee-436b-a8ad-29fdcecf32d5',
  plane: '4f8984c4-7c7a-4d69-b0a2-183cbb330d20',
  point: 'fbac3e32-f100-4292-8692-77240a42fd1a',
  robot: 'a11e8488-943e-426f-b205-e8db5f684901',
  trajectory: 'b22e8488-943e-426f-b205-e8db5f684902',
  jointState: 'c33e8488-943e-426f-b205-e8db5f684903',
  colScene: 'd44e8488-943e-426f-b205-e8db5f684904',
  segment: 'e55e8488-943e-426f-b205-e8db5f684905',
  tool: 'f66e8488-943e-426f-b205-e8db5f684906',
  toolState: 'a77e8488-943e-426f-b205-e8db5f684907',
};

/** Components that implement IGH_VariableParameterComponent — GH loads these via ParameterData only. */
const USE_PARAMETER_DATA = new Set(['plan', 'preview', 'segment']);

const MOTUS = {
  robot: { guid: 'aa3e8488-943e-426f-b205-e8db5f684998', name: 'Motus Robot', nick: 'Robot', w: 74, h: 104,
    inputs: [
      { name: 'Path', nick: 'P', desc: 'Path to .urdf or .xacro file', optional: false, text: '' },
      { name: 'BaseLink', nick: 'B', desc: 'Base link name', optional: true, text: 'base_link' },
      { name: 'TipLink', nick: 'Tip', desc: 'Tip link name', optional: true, text: 'tool0' },
      { name: 'Base', nick: 'Bf', desc: 'Optional base frame override (TCP goals are in this frame)', optional: true, plane: true },
      { name: 'Tool', nick: 'Tl', desc: 'Optional Motus Tool definition', optional: true },
    ],
    outputs: [{ name: 'Robot', nick: 'Rb', desc: 'Robot model with URDF kinematics chain' }] },
  ur10e: { guid: '84b06a7d-8a3d-46ec-968f-25e74c249ad1', name: 'Motus UR10e Robotiq', nick: 'UR10e', w: 74, h: 44,
    inputs: [],
    outputs: [{ name: 'Robot', nick: 'Rb', desc: 'Robot model with URDF kinematics chain' }] },
  tool: { guid: 'b7c4e2a1-9f3d-4b6e-8c1d-2a5f9e0b3d71', name: 'Motus Tool', nick: 'Tool', w: 74, h: 124,
    inputs: [
      { name: 'Name', nick: 'N', desc: 'Tool name', optional: false, text: 'gripper' },
      { name: 'TCP', nick: 'P', desc: 'TCP in flange frame (Z = tool axis); unwired + Description → TipTcp', optional: true, plane: true },
      { name: 'Geometry', nick: 'G', desc: 'Optional static gripper mesh (legacy Cap+STL); ignored when Description wired', optional: true },
      { name: 'GeomPlane', nick: 'L', desc: 'Geometry pose in TCP-local frame', optional: true, plane: true },
      { name: 'Capabilities', nick: 'Cap', desc: 'None or Robotiq2F85 (jaw presets for Motus Tool State)', optional: true, text: 'None' },
      { name: 'Description', nick: 'Rd', desc: 'Optional actuated mechanism (Motus Urdf Assemble) grafted on Motus Robot Tl', optional: true },
      { name: 'Binding', nick: 'Bd', desc: 'Optional driver joint name for Cap width', optional: true, text: '' },
    ],
    outputs: [{ name: 'Tool', nick: 'Tl', desc: 'Tool definition' }] },
  urdfLink: { guid: '2b3c4d5e-6f7a-4b2c-9d3e-4f5a6b7c8d92', name: 'Motus Urdf Link', nick: 'ULink', w: 74, h: 64,
    inputs: [
      { name: 'Name', nick: 'N', desc: 'Link name', optional: false, text: 'link' },
      { name: 'Visual', nick: 'V', desc: 'Rhino geometry (Box/Mesh/Brep/Surface/…, meters)', optional: false, access: 1 },
      { name: 'Collision', nick: 'C', desc: 'Optional collision geometry', optional: true, access: 1 },
    ],
    outputs: [{ name: 'Link', nick: 'L', desc: 'URDF link' }] },
  urdfJoint: { guid: '3c4d5e6f-7a8b-4c3d-ae4f-5a6b7c8d9ea3', name: 'Motus Urdf Joint', nick: 'UJoint', w: 74, h: 164,
    inputs: [
      { name: 'Name', nick: 'N', desc: 'Joint name', optional: false, text: 'joint' },
      { name: 'Type', nick: 'T', desc: 'Revolute / Continuous / Prismatic / Fixed', optional: true, text: 'Revolute' },
      { name: 'Parent', nick: 'Pa', desc: 'Parent link name', optional: false, text: 'palm' },
      { name: 'Child', nick: 'Ch', desc: 'Child link name', optional: false, text: 'finger' },
      { name: 'Axis', nick: 'Ax', desc: 'Origin (Start) + axis (End-Start); default +Z', optional: true },
      { name: 'Lower', nick: 'Lo', desc: 'Lower limit', optional: true, number: 0 },
      { name: 'Upper', nick: 'Up', desc: 'Upper limit', optional: true, number: 0.8 },
      { name: 'MimicJoint', nick: 'Mj', desc: 'Optional mimic target joint name', optional: true, text: '' },
      { name: 'MimicMult', nick: 'Mm', desc: 'Mimic multiplier', optional: true, number: 1 },
      { name: 'MimicOffset', nick: 'Mo', desc: 'Mimic offset', optional: true, number: 0 },
    ],
    outputs: [{ name: 'Joint', nick: 'J', desc: 'URDF joint' }] },
  urdfAssemble: { guid: '4d5e6f7a-8b9c-4d4e-bf5a-6b7c8d9eafb4', name: 'Motus Urdf Assemble', nick: 'UAssemble', w: 74, h: 84,
    inputs: [
      { name: 'Name', nick: 'N', desc: 'Description name', optional: true, text: 'gripper' },
      { name: 'Links', nick: 'L', desc: 'URDF links', optional: false, access: 1 },
      { name: 'Joints', nick: 'J', desc: 'URDF joints', optional: true, access: 1 },
      { name: 'Tip', nick: 'Tip', desc: 'Optional tip link', optional: true, text: 'palm' },
    ],
    outputs: [{ name: 'Description', nick: 'D', desc: 'Assembled robot description' }] },
  urdfExport: { guid: '2f6c1d3a-9b7e-4c5a-8e2d-6a1f4b3c7d90', name: 'Motus Export URDF', nick: 'UrdfExport', w: 74, h: 64,
    inputs: [
      { name: 'Description', nick: 'D', desc: 'RobotDescription from Assemble/Attach', optional: false },
      { name: 'Folder', nick: 'F', desc: 'Output folder for .urdf (+ meshes/)', optional: false, text: '' },
      { name: 'Name', nick: 'N', desc: 'Optional file name override', optional: true, text: '' },
    ],
    outputs: [
      { name: 'Path', nick: 'P', desc: 'Written .urdf path' },
      { name: 'Status', nick: 'Msg', desc: 'Status message' },
    ] },
  loadMesh: { guid: 'c3d4e5f6-a7b8-4901-c234-56789abcdef2', name: 'Motus Load Mesh', nick: 'LoadMesh', w: 74, h: 54,
    inputs: [
      { name: 'Path', nick: 'P', desc: 'Path to .stl file', optional: false, text: '' },
      { name: 'Plane', nick: 'L', desc: 'Mesh pose (origin = local origin)', optional: true, plane: true },
    ],
    outputs: [{ name: 'Mesh', nick: 'M', desc: 'Triangle mesh' }] },
  joints: { guid: '380f17c2-5d5f-4f77-a251-8309f25ef61e', name: 'Motus Joint State', nick: 'Joints', w: 65, h: 44,
    inputs: [
      { name: 'Joints', nick: 'J', desc: 'Joint angles (right-click J input to toggle °)', optional: false, list: true, access: 1, angle: true },
    ],
    outputs: [{ name: 'State', nick: 'Js', desc: 'Joint state' }] },
  tcpPose: { guid: 'f1a2b3c4-d5e6-4789-a123-4567890abcde', name: 'Motus TCP Pose', nick: 'TCP', w: 65, h: 44,
    inputs: [
      { name: 'Robot', nick: 'Rb', desc: 'Robot model', optional: false },
      { name: 'State', nick: 'Js', desc: 'Joint state', optional: false },
    ],
    outputs: [{ name: 'Plane', nick: 'P', desc: 'TCP pose in robot base frame (position + orientation)' }] },
  plan: { guid: '8bb0bae3-527f-4e80-a8a4-c8a88b7276de', name: 'Motus Plan', nick: 'Quick', w: 74, h: 104,
    desc: 'Quick planner: plane=LIN, joint=joint-linear/RRT. For PTP/CIRC/SET/WAIT use Motus Move → Motus Program.',
    inputs: [
      { name: 'Robot', nick: 'Rb', desc: 'Robot model from Motus UR10e or Motus Robot', optional: false, typeId: PTYPE.robot },
      { name: 'Goal', nick: 'G', desc: 'Planes (TCP LIN) or Joint States; list = visit order', optional: false, access: 1, typeId: PTYPE.generic },
      { name: 'Start', nick: 'St0', desc: 'Start as Plane (IK) or Joint State (defaults to home/zeros)', optional: true, typeId: PTYPE.generic },
      { name: 'Step', nick: 'St', desc: 'Plane goals only: TCP LIN step size (m)', optional: true, number: 0.005, typeId: PTYPE.number },
    ],
    advancedInputs: [
      { name: 'Collision', nick: 'C', desc: 'Collision scene; joint goals use RRT; plane goals validate LIN against scene', optional: true, typeId: PTYPE.colScene },
      { name: 'Group', nick: 'Gr', desc: 'Optional planning group (locks non-group joints)', optional: true, typeId: PTYPE.generic },
      { name: 'Attach', nick: 'A', desc: 'Attached bodies for collision checks', optional: true, access: 1, typeId: PTYPE.generic },
      { name: 'RrtSettings', nick: 'Rrt', desc: 'Optional RRT tuning from Motus RRT Settings (joint goals + collision or mobility)', optional: true, typeId: PTYPE.generic },
    ],
    outputs: [
      { name: 'Trajectory', nick: 'Tr', desc: 'Planned trajectories → Motus Preview / Motus Waypoints (one per goal)', access: 1, typeId: PTYPE.trajectory },
      { name: 'Status', nick: 'Msg', desc: 'Status message (read before controller handoff)', typeId: PTYPE.string },
      { name: 'Warnings', nick: 'W', desc: 'Capability / validation warnings', access: 1, typeId: PTYPE.string },
    ] },
  preview: { guid: 'd4a8f1c2-3e5b-4a7d-9c1e-8f2b6d4e0a91', name: 'Motus Preview', nick: 'Preview', w: 74, h: 84,
    inputs: [
      { name: 'Trajectory', nick: 'Tr', desc: 'Motus trajectory from Motus Plan (list concatenates sequential goals)', optional: false, access: 1, typeId: PTYPE.trajectory },
      { name: 'ShowStart', nick: 'SS', desc: 'Also preview the trajectory start pose as a ghost', optional: false, bool: true, typeId: PTYPE.boolean },
      { name: 'Position', nick: 'P', desc: 'Optional normalized playback position 0–1 (Motus Scrub)', optional: true, typeId: PTYPE.number },
    ],
    outputs: [
      { name: 'Meshes', nick: 'M', desc: 'Link meshes at the current frame', access: 1, typeId: PTYPE.mesh },
      { name: 'Links', nick: 'L', desc: 'Link lines at the current frame', access: 1, typeId: PTYPE.line },
      { name: 'TCP Path', nick: 'Path', desc: 'Full TCP polyline via FK', typeId: PTYPE.curve },
      { name: 'State', nick: 'Js', desc: 'Joint state at the current frame', typeId: PTYPE.jointState },
      { name: 'Time', nick: 'Tm', desc: 'Elapsed trajectory time at current frame (seconds)', typeId: PTYPE.number },
    ] },
  export: { guid: '0a443b6f-605b-48e3-843c-cd0a709f8379', name: 'Motus Export', nick: 'Export', w: 74, h: 104,
    inputs: [
      { name: 'Trajectory', nick: 'Tr', desc: 'Motus trajectory (list concatenates sequential goals)', optional: false, access: 1 },
      { name: 'Retime', nick: 'R', desc: 'Apply trajectory retiming before export', optional: true, bool: true },
      { name: 'Validate', nick: 'V', desc: 'Validate limits/velocity after retiming', optional: true, bool: false },
      { name: 'Retimer', nick: 'Rt', desc: 'Retimer algorithm when Retime=true: TotgLite (default), Totg, SegmentTrapezoid, or Bottleneck', optional: true, text: 'TotgLite' },
    ],
    outputs: [
      { name: 'Json', nick: 'J', desc: 'Trajectory JSON' },
      { name: 'Csv', nick: 'C', desc: 'Trajectory CSV' },
      { name: 'Validation', nick: 'Val', desc: 'Validation summary when Validate=true', optional: true },
    ] },
  waypoints: { guid: '133ba1e0-5b0e-46f7-92e8-31aaa7e60a55', name: 'Motus Waypoints', nick: 'Waypoints', w: 74, h: 84,
    inputs: [
      { name: 'Trajectory', nick: 'Tr', desc: 'Motus trajectory (list concatenates sequential goals)', optional: false, access: 1 },
      { name: 'Decimate', nick: 'D', desc: 'Keep every Nth waypoint (always keeps first and last). 1 = all', optional: true, number: 1, typeId: PTYPE.integer },
    ],
    outputs: [
      { name: 'Joints', nick: 'Q', desc: 'Joint tree {waypoint→q[n]} for MoveJ-style controllers (primary handoff)', access: 1 },
      { name: 'Planes', nick: 'P', desc: 'FK TCP planes. Prefer Q→MoveJ for joint paths; P→MoveL only for Cartesian-intent' },
      { name: 'Times', nick: 'Tm', desc: 'Waypoint times (seconds)' },
    ] },
  rrtSettings: { guid: '11d59b15-ffe2-488e-83b8-52eddf772025', name: 'Motus RRT Settings', nick: 'RrtSet', w: 74, h: 104,
    inputs: [
      { name: 'MaxIter', nick: 'Mi', desc: 'Max sampling iterations', optional: false, number: 4000 },
      { name: 'TimeLimit', nick: 'Lim', desc: 'Wall-clock cap in seconds (0 = off)', optional: false, number: 30 },
      { name: 'Planner', nick: 'P', desc: 'Sampling planner from registry', optional: false, text: 'RrtConnect' },
      { name: 'GoalBias', nick: 'Gb', desc: 'Goal bias 0–1', optional: false, number: 0.08 },
      { name: 'Step', nick: 'St', desc: 'Config step (radians for serial/legged; meters for Family=stewart)', optional: false, number: 0.12 },
    ],
    outputs: [{ name: 'Settings', nick: 'Rrt', desc: 'Sampling planner settings for Motus Plan' }] },
  colSphere: { guid: 'c1a2b3c4-d5e6-4789-a012-3456789abcde', name: 'Motus Collision Sphere', nick: 'ColSph', w: 74, h: 64,
    inputs: [
      { name: 'Center', nick: 'C', desc: 'Sphere center', optional: false, point: [0.35, 0.15, 0.35] },
      { name: 'Radius', nick: 'R', desc: 'Radius (m)', optional: false, number: 0.12 },
      { name: 'Name', nick: 'N', desc: 'Obstacle name', optional: false, text: 'sphere' },
    ],
    outputs: [{ name: 'Object', nick: 'O', desc: 'Collision object' }] },
  colBox: { guid: 'd2b3c4d5-e6f7-4890-b123-456789abcdef', name: 'Motus Collision Box', nick: 'ColBox', w: 74, h: 84,
    inputs: [
      { name: 'Plane', nick: 'P', desc: 'Box center/orientation', optional: false, plane: true },
      { name: 'HalfX', nick: 'X', desc: 'Half extent X', optional: false, number: 0.15 },
      { name: 'HalfY', nick: 'Y', desc: 'Half extent Y', optional: false, number: 0.08 },
      { name: 'HalfZ', nick: 'Z', desc: 'Half extent Z', optional: false, number: 0.4 },
      { name: 'Name', nick: 'N', desc: 'Obstacle name', optional: false, text: 'table' },
    ],
    outputs: [{ name: 'Object', nick: 'O', desc: 'Collision object' }] },
  colMesh: { guid: 'f4d5e6f7-a8b9-4012-d345-6789abcdef01', name: 'Motus Collision Mesh', nick: 'ColMesh', w: 74, h: 54,
    inputs: [
      { name: 'Geometry', nick: 'G', desc: 'Triangle mesh or Brep obstacle', optional: false },
      { name: 'Plane', nick: 'P', desc: 'Geometry pose (origin = local origin)', optional: false, plane: true },
      { name: 'Name', nick: 'N', desc: 'Obstacle name', optional: false, text: 'mesh' },
    ],
    outputs: [{ name: 'Object', nick: 'O', desc: 'Collision object' }] },
  colScene: { guid: 'e3c4d5e6-f7a8-4901-c234-56789abcdef0', name: 'Motus Collision Scene', nick: 'ColScene', w: 74, h: 64,
    inputs: [
      { name: 'Objects', nick: 'O', desc: 'Collision objects', optional: false, access: 1 },
      { name: 'Srdf', nick: 'S', desc: 'Optional SRDF file path (disable_collisions pairs)', optional: true, text: '' },
    ],
    outputs: [
      { name: 'Scene', nick: 'Sc', desc: 'Collision scene' },
      { name: 'Groups', nick: 'G', desc: 'Planning groups from SRDF (optional)', access: 1 },
      { name: 'EndEffectors', nick: 'EE', desc: 'End-effector map from SRDF as name=parent_link entries', access: 1 },
    ] },
  group: { guid: '91e2a9db-cfb4-4a6c-99a3-305ba27fdf1e', name: 'Motus Planning Group', nick: 'Group', w: 74, h: 84,
    inputs: [
      { name: 'Group', nick: 'G', desc: 'Optional existing planning group (e.g. from ColScene SRDF output)', optional: true },
      { name: 'Name', nick: 'N', desc: 'Group name', optional: true, text: 'manipulator' },
      { name: 'BaseLink', nick: 'B', desc: 'Base link name', optional: true, text: 'base_link' },
      { name: 'TipLink', nick: 'Tip', desc: 'Tip link name', optional: true, text: 'tool0' },
      { name: 'Joints', nick: 'J', desc: 'Joint names (leave empty to use base..tip shorthand)', optional: true, access: 1 },
    ],
    outputs: [{ name: 'Group', nick: 'G', desc: 'Planning group' }] },
  attach: { guid: '0c464ac8-0e1d-4c7a-9c8c-0a21f1046314', name: 'Motus Attach Body', nick: 'Attach', w: 74, h: 74,
    inputs: [
      { name: 'Object', nick: 'O', desc: 'Collision object geometry to attach', optional: false },
      { name: 'Name', nick: 'N', desc: 'Attached body name', optional: true, text: 'grasp' },
      { name: 'TcpLocal', nick: 'P', desc: 'TCP-local pose of attached geometry', optional: true, plane: true },
      { name: 'SourceName', nick: 'Src', desc: 'Optional scene object name to hide while attached', optional: true, text: '' },
    ],
    outputs: [{ name: 'Attach', nick: 'A', desc: 'Attached body' }] },
  toolState: { guid: 'a1b2c3d4-e5f6-7890-abcd-ef1234567890', name: 'Motus Tool State', nick: 'ToolState', w: 74, h: 84,
    inputs: [
      { name: 'Tool', nick: 'Tl', desc: 'Motus Tool or Robot (uses Robot.Tool / bundled capabilities)', optional: true },
      { name: 'Preset', nick: 'P', desc: 'Open, Closed, or Custom', optional: false, text: 'Open' },
      { name: 'Width', nick: 'W', desc: 'Jaw width (m) when Preset=Custom', optional: true, number: 0.085 },
      { name: 'Speed', nick: 'Sp', desc: 'Grip speed ratio 0–1', optional: true, number: 0.5 },
      { name: 'Force', nick: 'F', desc: 'Grip force ratio 0–1', optional: true, number: 0.5 },
    ],
    outputs: [{ name: 'State', nick: 'Ts', desc: 'End-effector state' }] },
  segment: { guid: '7c4e9a2f-1b3d-4e8a-9f6c-2d8b5a7e9c31', name: 'Motus Move', nick: 'Move', w: 74, h: 100,
    desc: 'One PTP/LIN/CIRC/SET/WAIT program line (Type/ToolMode on-component dropdowns; pins morph by type)',
    inputs: [
      { name: 'Type', nick: 'Ty', desc: 'PTP, LIN, CIRC, SET, or WAIT (prefer on-component dropdown)', optional: false, text: 'PTP', typeId: PTYPE.string },
      { name: 'Goal', nick: 'G', desc: 'PTP: Joint State; LIN/CIRC: Plane (TCP pose)', optional: true, typeId: PTYPE.generic },
      { name: 'Blend', nick: 'B', desc: 'Blend radius (m, default 0)', optional: true, number: 0, typeId: PTYPE.number },
      { name: 'ToolState', nick: 'Ts', desc: 'Tool state (SET required; optional on arm moves)', optional: true, typeId: PTYPE.toolState },
    ],
    typeInputs: {
      LIN: [
        { name: 'Step', nick: 'St', desc: 'LIN only: TCP step size (m)', optional: true, number: 0.005, typeId: PTYPE.number },
      ],
      CIRC: [
        { name: 'Via', nick: 'V', desc: 'CIRC only: arc via point (TCP plane)', optional: true, typeId: PTYPE.plane },
        { name: 'Samples', nick: 'N', desc: 'CIRC only: arc samples (>= 4)', optional: true, number: 16, typeId: PTYPE.integer },
      ],
      SET: [{ name: 'Duration', nick: 'D', desc: 'SET/WAIT duration (s)', optional: true, number: 0, typeId: PTYPE.number }],
      WAIT: [{ name: 'Duration', nick: 'D', desc: 'SET/WAIT duration (s)', optional: true, number: 0, typeId: PTYPE.number }],
    },
    outputs: [{ name: 'Segment', nick: 'Seg', desc: 'Motion segment', typeId: PTYPE.segment }] },
  progPlan: { guid: '8d5f0b3e-2c4e-4f9b-0a7d-3e9c6b8f0d42', name: 'Motus Program', nick: 'Program', w: 74, h: 144,
    desc: 'Plan Motus Move sequence (Auto Plan or click Plan); LIN failures do not fall back to joint paths',
    inputs: [
      { name: 'Robot', nick: 'Rb', desc: 'Robot model', optional: false },
      { name: 'Segments', nick: 'Seg', desc: 'List of Motus Move segments (wire order = program order)', optional: false, access: 1 },
      { name: 'Start', nick: 'St0', desc: 'Start joint state (defaults to home)', optional: true },
      { name: 'Collision', nick: 'C', desc: 'Collision scene', optional: true },
      { name: 'Group', nick: 'Gr', desc: 'Optional planning group (locks non-group joints)', optional: true },
      { name: 'Attach', nick: 'A', desc: 'Optional attached bodies list', optional: true, access: 1 },
    ],
    outputs: [
      { name: 'Trajectory', nick: 'Tr', desc: 'Planned trajectory' },
      { name: 'Status', nick: 'Msg', desc: 'Planning status' },
      { name: 'Warnings', nick: 'W', desc: 'Capability / validation warnings' },
    ] },
  scrub: { guid: 'e1f2a3b4-c5d6-4789-a012-3456789abc01', name: 'Motus Scrub', nick: 'Scrub', w: 220, h: 44 },
  serialChain: {
    guid: 'c8f2a1d0-4e3b-4a7c-9d1e-2b6f8a0c5e71', name: 'Motus Serial Chain', nick: 'Serial', w: 74, h: 124,
    desc: 'Parametric serial / rail+arm from link lengths (same Robot goo as Motus Robot)',
    inputs: [
      { name: 'Lengths', nick: 'L', desc: 'Link lengths (m). With Rail: first = stroke, rest = arm.', optional: false, list: true, access: 1 },
      { name: 'Base', nick: 'B', desc: 'Optional base frame', optional: true, plane: true },
      { name: 'Home', nick: 'Q', desc: 'Optional home joint values (driver order)', optional: true, list: true, access: 1 },
      { name: 'Rail', nick: 'Rail', desc: 'First length is prismatic stroke (+Z); rest revolute', optional: true, bool: false },
      { name: 'Types', nick: 'Types', desc: 'Optional R/P per joint (ignored when Rail)', optional: true, access: 1 },
      { name: 'TCP', nick: 'TCP', desc: 'Optional tip tool frame in last-link frame', optional: true, plane: true },
    ],
    outputs: [{ name: 'Robot', nick: 'Rb', desc: 'Robot model (same as Motus Robot)' }],
  },
  reachSamples: {
    guid: 'a1b2c3d4-5e6f-7081-92a3-b4c5d6e7f809', name: 'Motus Reach Samples', nick: 'Reach', w: 74, h: 64,
    desc: 'Stratified TCP reach samples (capped). Overlay on structure in Rhino.',
    inputs: [
      { name: 'Robot', nick: 'Rb', desc: 'Motus Robot (Serial Chain or URDF)', optional: false },
      { name: 'Count', nick: 'N', desc: 'Max TCP samples (default 512, max 512)', optional: true, number: 128 },
      { name: 'Seed', nick: 'Seed', desc: 'Reserved (Halton; currently unused)', optional: true, number: 0 },
    ],
    outputs: [{ name: 'Points', nick: 'Pts', desc: 'Sampled TCP points in base frame', access: 1 }],
  },
  jointTable: {
    guid: 'd9e3b2c1-5f4a-4b8d-9e2f-3c7a1d0b6f82', name: 'Motus Joint Table', nick: 'JointTbl', w: 74, h: 164,
    desc: 'Joint table → tree. Plan uses Tip path only; side branches are TreeFK preview only.',
    inputs: [
      { name: 'Parent', nick: 'Par', desc: 'Parent link names', optional: false, access: 1 },
      { name: 'Child', nick: 'Ch', desc: 'Child link names', optional: false, access: 1 },
      { name: 'Type', nick: 'Ty', desc: 'R / P / C / F per joint', optional: false, access: 1 },
      { name: 'Ox', nick: 'Ox', desc: 'Joint origin X (m)', optional: false, list: true, access: 1 },
      { name: 'Oy', nick: 'Oy', desc: 'Joint origin Y (m)', optional: true, list: true, access: 1 },
      { name: 'Oz', nick: 'Oz', desc: 'Joint origin Z (m)', optional: true, list: true, access: 1 },
      { name: 'Name', nick: 'N', desc: 'Optional joint names', optional: true, access: 1 },
      { name: 'Tip', nick: 'Tip', desc: 'Tip link for Plan/serial chain (default: last Child)', optional: true, text: '' },
      { name: 'Base', nick: 'B', desc: 'Optional base frame', optional: true, plane: true },
      { name: 'Home', nick: 'Q', desc: 'Optional home q along tip path', optional: true, list: true, access: 1 },
      { name: 'BaseSE2', nick: 'SE2', desc: 'Optional holonomic base goal X, Y, Yaw(rad) — also used as preview base frame', optional: true, list: true, access: 1 },
    ],
    outputs: [{ name: 'Robot', nick: 'Rb', desc: 'Robot model (same as Motus Robot)' }],
  },
  stewart: {
    guid: 'a9e1c3f0-7b2d-4e8a-9c1f-6d4b2a0e8f73', name: 'Motus Stewart', nick: 'Stewart', w: 74, h: 124,
    desc: 'Stewart/Gough hexapod (Family=stewart; Q = leg lengths in meters)',
    inputs: [
      { name: 'Path', nick: 'P', desc: 'Optional Stewart JSON (schemaVersion=1)', optional: true, text: '' },
      { name: 'BaseRadius', nick: 'Br', desc: 'Classic base anchor radius (m)', optional: true, number: 0.5 },
      { name: 'PlatformRadius', nick: 'Pr', desc: 'Classic platform anchor radius (m)', optional: true, number: 0.3 },
      { name: 'MinStroke', nick: 'Lmin', desc: 'Min leg length (m)', optional: true, number: 0.35 },
      { name: 'MaxStroke', nick: 'Lmax', desc: 'Max leg length (m)', optional: true, number: 0.90 },
      { name: 'Name', nick: 'N', desc: 'Model name', optional: true, text: 'stewart_classic' },
    ],
    outputs: [{ name: 'Robot', nick: 'Rb', desc: 'Stewart robot (Family=stewart)' }],
  },
  terrainPatch: {
    guid: '86e87c03-366b-4de3-9448-3b154cd28f24', name: 'Motus Terrain Patch', nick: 'Ground', w: 74, h: 84,
    desc: 'Outdoor heightfield mesh (m) for Motus Walk Terrain',
    inputs: [
      { name: 'Origin', nick: 'O', desc: 'Patch center (m)', optional: true, point: [0.22, 0, 0] },
      { name: 'SizeX', nick: 'Sx', desc: 'Full width X (m)', optional: true, number: 1.0 },
      { name: 'SizeY', nick: 'Sy', desc: 'Full depth Y (m)', optional: true, number: 0.8 },
      { name: 'Amp', nick: 'A', desc: 'Hill amplitude (m)', optional: true, number: 0.012 },
    ],
    outputs: [{ name: 'Mesh', nick: 'M', desc: 'Outdoor ground mesh', typeId: PTYPE.mesh }],
  },
  leg: {
    guid: '9a49a661-ff4c-4b96-bb57-c977ee6f9da2', name: 'Motus Leg', nick: 'Leg', w: 64, h: 72,
    desc: 'Leg lengths (m) → Leg goo for Motus Mechanism',
    inputs: [
      { name: 'Lengths', nick: 'L', desc: 'Link lengths (m)', optional: true, list: true, access: 1 },
      { name: 'Name', nick: 'N', desc: 'Optional leg name', optional: true, text: 'leg' },
      { name: 'Tip', nick: 'Tip', desc: 'Foot link name', optional: true, text: '' },
    ],
    outputs: [{ name: 'Leg', nick: 'Leg', desc: 'Leg recipe → Mechanism' }],
  },
  body: {
    guid: '92f0d969-c8ef-47c5-9ec7-514bebbd8441', name: 'Motus Body', nick: 'Body', w: 64, h: 96,
    desc: 'Radial or custom hip frames → Bdy for Mechanism',
    inputs: [
      { name: 'N', nick: 'N', desc: 'Radial hip count', optional: true },
      { name: 'BodyR', nick: 'Br', desc: 'Body radius (m)', optional: true, number: 0.06 },
      { name: 'BodyZ', nick: 'Bz', desc: 'Body height (m)', optional: true, number: 0.07 },
      { name: 'Planes', nick: 'Pl', desc: 'Optional custom hip planes', optional: true, typeId: PTYPE.plane, access: 1 },
    ],
    outputs: [{ name: 'Body', nick: 'Bdy', desc: 'Hip frames → Mechanism' }],
  },
  mechanism: {
    guid: 'aa18b783-9a1c-44f8-bd2b-e508c3d372ac', name: 'Motus Mechanism', nick: 'Mech', w: 74, h: 140,
    desc: 'Assemble Bdy+Leg → Mech for Motus Walk',
    inputs: [
      { name: 'Body', nick: 'Bdy', desc: 'From Motus Body' },
      { name: 'Leg', nick: 'Leg', desc: 'One Leg or list', access: 1 },
      { name: 'AllowDynamicGait', nick: 'Dyn', desc: 'Allow dynamic gait', optional: true, bool: false },
      { name: 'Tip', nick: 'Tip', desc: 'Tip leg name', optional: true, text: '' },
      { name: 'HipStance', nick: 'Hs', desc: 'Coxa stance (rad)', optional: true, number: 0.1309 },
      { name: 'FemurStance', nick: 'Fs', desc: 'Femur stance (rad)', optional: true, number: 0.5236 },
      { name: 'TibiaStance', nick: 'Ts', desc: 'Tibia stance (rad)', optional: true, number: -0.5236 },
    ],
    outputs: [{ name: 'Mechanism', nick: 'Mech', desc: 'Assembled walker → Walk' }],
  },
  walk: {
    guid: '236f9a53-c07b-4663-bf27-950e20fb59ab', name: 'Motus Walk', nick: 'Walk', w: 80, h: 180,
    desc: 'Walk Mech along Path/Planes + Terrain. Family=legged. NOT Stewart',
    inputs: [
      { name: 'Mechanism', nick: 'Mech', desc: 'From Motus Mechanism' },
      { name: 'Pose', nick: 'Pose', desc: 'Optional body-pose policy', optional: true },
      { name: 'Path', nick: 'P', desc: 'Walk path curve (m)', optional: true, typeId: PTYPE.curve },
      { name: 'Planes', nick: 'Pl', desc: 'Or path as plane origins (m)', optional: true, typeId: PTYPE.plane, access: 1 },
      { name: 'Speed', nick: 'Spd', desc: 'Walk speed (m/s)', optional: true, number: 0.06 },
      { name: 'Step', nick: 'St', desc: 'Step length (m)', optional: true, number: 0.04 },
      { name: 'Lift', nick: 'Lf', desc: 'Swing lift (m)', optional: true, number: 0.02 },
      { name: 'Terrain', nick: 'Tn', desc: 'Optional ground Mesh/Brep (m)', optional: true, access: 1 },
    ],
    outputs: [
      { name: 'Robot', nick: 'Rb', desc: 'Robot (gait=full drivers)' },
      { name: 'State', nick: 'Js', desc: 'Full-driver stance', typeId: PTYPE.jointState },
      { name: 'Trajectory', nick: 'Tr', desc: 'Gait trajectory when Path/Planes wired', typeId: PTYPE.trajectory },
      { name: 'PathCurve', nick: 'Pc', desc: 'Resolved path curve', typeId: PTYPE.curve },
      { name: 'PathPlanes', nick: 'Pp', desc: 'Body planes along path', typeId: PTYPE.plane, access: 1 },
      { name: 'Meshes', nick: 'M', desc: 'Preview meshes', access: 1 },
      { name: 'Support', nick: 'Sp', desc: 'Support polygon', typeId: PTYPE.curve },
    ],
  },
};

const NATIVE = {
  panel: { guid: '59e0b89a-e487-49f8-bab8-b5bab16be14c', name: 'Panel', w: 160, h: 60 },
  // GUIDs verified live against Rhino 8 / Grasshopper (placeholders mean stale GUIDs).
  constructPoint: { guid: '3581f42a-9592-4549-bd6b-1c0fc39d067b', name: 'Construct Point', nick: 'Pt', w: 44, h: 44,
    inputs: ['X', 'Y', 'Z'], outputs: ['Point'] },
  unitZ: { guid: '9103c240-a6a9-4223-9b42-dbd19bf38e2b', name: 'Unit Z', nick: 'Z', w: 44, h: 22, outputs: ['Vector'] },
  unitX: { guid: '79f9fbb3-8f1d-4d9a-88a9-f7961b1012cd', name: 'Unit X', nick: 'X', w: 44, h: 22, outputs: ['Vector'] },
  plane: { guid: 'cfb6b17f-ca82-4f5d-b604-d4f69f569de3', name: 'Plane Normal', nick: 'Pl', w: 44, h: 44,
    inputs: ['Origin', 'Z-Axis'], outputs: ['Plane'] },
  xyPlane: { guid: '17b7152b-d30d-4d50-b9ef-c9fe25576fc2', name: 'XY Plane', nick: 'XY', w: 44, h: 22, outputs: ['Plane'] },
  // SurfaceComponents.gha — Center Box (Base plane + X/Y/Z size → Box)
  centerBox: { guid: '28061aae-04fb-4cb5-ac45-16f3b66bc0a4', name: 'Center Box', nick: 'Box', w: 54, h: 64,
    inputs: ['Base', 'X', 'Y', 'Z'], outputs: ['Box'] },
  // CurveComponents.gha — Line SDL (Start + Direction + Length → Line)
  lineSdl: { guid: '4c619bc9-39fd-4717-82a6-1e07ea237bbe', name: 'Line SDL', nick: 'Ln', w: 44, h: 64,
    inputs: ['Start', 'Direction', 'Length'], outputs: ['Line'] },
  filePath: { guid: '06953bda-1d37-4d58-9b38-4b3c74e54c8f', name: 'File Path', nick: 'Path', w: 50, h: 24 },
  move: { guid: '4f7cd4e3-9b20-41d8-9c00-2940fe7f3aa0', name: 'Move', nick: 'Move', w: 44, h: 44,
    inputs: ['Geometry', 'Motion'], outputs: ['Geometry'] },
  // Verified live against Rhino 8 / Grasshopper.
  merge: { guid: '3cadddef-1e2b-4c09-9390-0e8f78f7609f', name: 'Merge', nick: 'Merge', w: 62, h: 44 },
  scribble: { guid: '7f5c6c55-f846-4a08-9c9a-cfdc285cc6fe', name: 'Scribble' },
  group: { guid: 'c552a431-af5b-46a9-a8a4-0fcbc27ef596', name: 'Group' },
  // Grasshopper.Kernel.Special.GH_NumberSlider (SliderGuid)
  numberSlider: {
    guid: '57da07bd-ecab-415d-9d86-af36d7073abc',
    name: 'Number Slider',
    nick: 'N',
    w: 160,
    h: 24,
  },
};

/**
 * Soft group fills (ARGB α=70) — match MotusPalette / MotusIcon subcategory tints.
 * Same role → same colour in every example.
 */
const GROUP_COLOUR = {
  model: '70;0;219;135',       // MotusPalette.Model #00DB87
  plan: '70;120;125;250',      // MotusPalette.Plan #787DFA
  collision: '70;181;165;154', // MotusPalette.Collision (peach→chrome)
  preview: '70;161;152;202',   // MotusPalette.Preview (lavender→chrome)
};
// Role aliases (keep call sites readable).
GROUP_COLOUR.robot = GROUP_COLOUR.model;   // Robot / URDF / start
GROUP_COLOUR.tool = GROUP_COLOUR.model;    // Tool TCP + mesh (Model tab)
GROUP_COLOUR.goals = GROUP_COLOUR.plan;     // Goal merge → Plan
GROUP_COLOUR.program = GROUP_COLOUR.plan;   // Moves → Program

function id() {
  return crypto.randomUUID();
}

function esc(s) {
  return String(s).replace(/&/g, '&amp;').replace(/</g, '&lt;');
}

function item(name, type, code, value) {
  return `            <item name="${name}" type_name="${type}" type_code="${code}">${value}</item>`;
}

function bounds(x, y, w, h) {
  return `<chunk name="Attributes">
                      <items count="2">
                        ${item('Bounds', 'gh_drawing_rectanglef', '35', `\n                          <X>${x}</X>\n                          <Y>${y}</Y>\n                          <W>${w}</W>\n                          <H>${h}</H>\n                        `)}
                        ${item('Pivot', 'gh_drawing_pointf', '31', `\n                          <X>${x + w / 2}</X>\n                          <Y>${y + h / 2}</Y>\n                        `)}
                      </items>
                    </chunk>`;
}

function persistentNumbers(values) {
  const items = values.map((n, i) => `<chunk name="Item" index="${i}">
                                  <items count="1">
                                    ${item('number', 'gh_double', '6', n)}
                                  </items>
                                </chunk>`).join('\n                                ');
  return `<chunk name="PersistentData">
                          <items count="1">
                            ${item('Count', 'gh_int32', '3', '1')}
                          </items>
                          <chunks count="1">
                            <chunk name="Branch" index="0">
                              <items count="2">
                                ${item('Count', 'gh_int32', '3', String(values.length))}
                                ${item('Path', 'gh_string', '10', '{0}')}
                              </items>
                              <chunks count="${values.length}">
                                ${items}
                              </chunks>
                            </chunk>
                          </chunks>
                        </chunk>`;
}

function persistentText(text) {
  return persistentTexts([text]);
}

function persistentTexts(values) {
  const items = values.map((t, i) => `<chunk name="Item" index="${i}">
                                  <items count="2">
                                    ${item('null_string', 'gh_bool', '1', 'false')}
                                    ${item('string', 'gh_string', '10', esc(t))}
                                  </items>
                                </chunk>`).join('\n                                ');
  return `<chunk name="PersistentData">
                          <items count="1">
                            ${item('Count', 'gh_int32', '3', '1')}
                          </items>
                          <chunks count="1">
                            <chunk name="Branch" index="0">
                              <items count="2">
                                ${item('Count', 'gh_int32', '3', String(values.length))}
                                ${item('Path', 'gh_string', '10', '{0}')}
                              </items>
                              <chunks count="${values.length}">
                                ${items}
                              </chunks>
                            </chunk>
                          </chunks>
                        </chunk>`;
}

function persistentBool(v) {
  return `<chunk name="PersistentData">
                          <items count="1">
                            ${item('Count', 'gh_int32', '3', '1')}
                          </items>
                          <chunks count="1">
                            <chunk name="Branch" index="0">
                              <items count="2">
                                ${item('Count', 'gh_int32', '3', '1')}
                                ${item('Path', 'gh_string', '10', '{0}')}
                              </items>
                              <chunks count="1">
                                <chunk name="Item" index="0">
                                  <items count="1">
                                    ${item('boolean', 'gh_bool', '1', v ? 'true' : 'false')}
                                  </items>
                                </chunk>
                              </chunks>
                            </chunk>
                          </chunks>
                        </chunk>`;
}

function sourceItem(index, guid) {
  return `                        <item name="Source" index="${index}" type_name="gh_guid" type_code="9">${guid}</item>`;
}

function paramInput(def, index, px, py, compW, sources, persistent) {
  const srcs = sources ?? [];
  const srcItems = srcs.map((s, i) => sourceItem(i, s)).join('\n');
  const count = srcs.length;
  const optional = def.optional ? item('Optional', 'gh_bool', '1', 'true') : item('Optional', 'gh_bool', '1', 'false');
  const access = def.access === 1 ? item('Access', 'gh_int32', '3', '1') : '';
  const chunks = [bounds(px + 2, py + 2 + index * 20, 17, 20)];
  if (persistent) chunks.push(persistent);
  if (def.angle) {
    chunks.push(`<chunk name="FixedSettings">
                          <items count="${def.useDegrees ? 2 : 1}">
                            ${item('Angle', 'gh_bool', '1', 'true')}
                            ${def.useDegrees ? item('UseDegrees', 'gh_bool', '1', 'true') : ''}
                          </items>
                        </chunk>`);
  }
  const itemLines = [
    access,
    item('Description', 'gh_string', '10', esc(def.desc ?? def.name)),
    item('InstanceGuid', 'gh_guid', '9', def._guid),
    item('Name', 'gh_string', '10', def.name),
    item('NickName', 'gh_string', '10', def.nick ?? def.name),
    optional,
    srcItems,
    item('SourceCount', 'gh_int32', '3', String(count)),
  ].filter(Boolean).join('\n');
  return `<chunk name="param_input" index="${index}">
                      <items count="${6 + count + (access ? 1 : 0)}">
${itemLines}
                      </items>
                      <chunks count="${chunks.length}">
                        ${chunks.join('\n                        ')}
                      </chunks>
                    </chunk>`;
}

function paramOutput(def, index, px, py, compW) {
  const access = def.access === 1 ? item('Access', 'gh_int32', '3', '1') : '';
  return `<chunk name="param_output" index="${index}">
                      <items count="${6 + (access ? 1 : 0)}">
                        ${access}
                        ${item('Description', 'gh_string', '10', esc(def.desc ?? def.name))}
                        ${item('InstanceGuid', 'gh_guid', '9', def._guid)}
                        ${item('Name', 'gh_string', '10', def.name)}
                        ${item('NickName', 'gh_string', '10', def.nick ?? def.name)}
                        ${item('Optional', 'gh_bool', '1', 'false')}
                        ${item('SourceCount', 'gh_int32', '3', '0')}
                      </items>
                      <chunks count="1">
                        ${bounds(px + compW - 18, py + 2 + index * 20, 16, 20)}
                      </chunks>
                    </chunk>`;
}

function paramAttrBounds(px, py, w = 19, h = 20) {
  return `<chunk name="Attributes">
                              <items count="2">
                                ${item('Bounds', 'gh_drawing_rectanglef', '35', `\n                                  <X>${px}</X>\n                                  <Y>${py}</Y>\n                                  <W>${w}</W>\n                                  <H>${h}</H>\n                                `)}
                                ${item('Pivot', 'gh_drawing_pointf', '31', `\n                                  <X>${px + w / 2}</X>\n                                  <Y>${py + h / 2}</Y>\n                                `)}
                              </items>
                            </chunk>`;
}

function parameterDataChunk(inputs, outputs, x, y, compW, wireMapSafe, options) {
  const idItems = [
    item('InputCount', 'gh_int32', '3', String(inputs.length)),
    ...inputs.map((inp, i) => `                        <item name="InputId" index="${i}" type_name="gh_guid" type_code="9">${inp.typeId}</item>`),
    item('OutputCount', 'gh_int32', '3', String(outputs.length)),
    ...outputs.map((out, i) => `                        <item name="OutputId" index="${i}" type_name="gh_guid" type_code="9">${out.typeId}</item>`),
  ];
  const inChunks = inputs.map((inp, i) => {
    const sources = (wireMapSafe[inp.name] ?? []).map((ref) => ref._guid);
    const srcItems = sources.map((s, si) => sourceItem(si, s));
    const access = inp.access === 1 ? item('Access', 'gh_int32', '3', '1') : '';
    const optional = item('Optional', 'gh_bool', '1', inp.optional ? 'true' : 'false');
    let persistent = null;
    if (options.textList?.[inp.name])
      persistent = persistentTexts(options.textList[inp.name]);
    else if (inp.list && (options.numberList?.[inp.name] ?? (inp.name === 'Lengths' || inp.name === 'Joints' ? options.jointValues : null)))
      persistent = persistentNumbers(options.numberList?.[inp.name] ?? options.jointValues);
    else if (inp.bool !== undefined && !sources.length) persistent = persistentBool(inp.bool);
    else if (inp.number !== undefined && !sources.length) persistent = persistentNumbers([inp.number]);
    else if (inp.text !== undefined && !sources.length) persistent = persistentText(inp.text);
    else if (inp.point && !sources.length) persistent = persistentNumbers(inp.point);
    const nested = [paramAttrBounds(x + 2, y + 2 + i * 20)];
    if (persistent) nested.push(persistent);
    if (inp.angle) {
      nested.push(`<chunk name="FixedSettings">
                              <items count="${inp.useDegrees ? 2 : 1}">
                                ${item('Angle', 'gh_bool', '1', 'true')}
                                ${inp.useDegrees ? item('UseDegrees', 'gh_bool', '1', 'true') : ''}
                              </items>
                            </chunk>`);
    }
    const items = [
      access,
      item('Description', 'gh_string', '10', esc(inp.desc ?? inp.name)),
      item('InstanceGuid', 'gh_guid', '9', inp._guid),
      item('Name', 'gh_string', '10', inp.name),
      item('NickName', 'gh_string', '10', inp.nick ?? inp.name),
      optional,
      ...srcItems,
      item('SourceCount', 'gh_int32', '3', String(sources.length)),
    ].filter(Boolean);
    return `<chunk name="InputParam" index="${i}">
                          <items count="${items.length}">
                            ${items.join('\n                            ')}
                          </items>
                          <chunks count="${nested.length}">
                            ${nested.join('\n                            ')}
                          </chunks>
                        </chunk>`;
  });
  const outChunks = outputs.map((out, i) => {
    const access = out.access === 1 ? item('Access', 'gh_int32', '3', '1') : '';
    const items = [
      access,
      item('Description', 'gh_string', '10', esc(out.desc ?? out.name)),
      item('InstanceGuid', 'gh_guid', '9', out._guid),
      item('Name', 'gh_string', '10', out.name),
      item('NickName', 'gh_string', '10', out.nick ?? out.name),
      item('Optional', 'gh_bool', '1', 'false'),
      item('SourceCount', 'gh_int32', '3', '0'),
    ].filter(Boolean);
    return `<chunk name="OutputParam" index="${i}">
                          <items count="${items.length}">
                            ${items.join('\n                            ')}
                          </items>
                          <chunks count="1">
                            ${paramAttrBounds(x + compW - 18, y + 2 + i * 20, 16, 20)}
                          </chunks>
                        </chunk>`;
  });
  const paramChunks = [...inChunks, ...outChunks];
  return `<chunk name="ParameterData">
                      <items count="${idItems.length}">
                        ${idItems.join('\n                        ')}
                      </items>
                      <chunks count="${paramChunks.length}">
                        ${paramChunks.join('\n                        ')}
                      </chunks>
                    </chunk>`;
}

function motusComponent(key, x, y, wireMap, options = {}) {
  const spec = structuredClone(MOTUS[key]);
  const instance = id();
  const wireMapSafe = wireMap ?? {};
  for (const [pin, refs] of Object.entries(wireMapSafe)) {
    if (Array.isArray(refs) && refs.length > 1) {
      throw new Error(
        `${key}.${pin} has ${refs.length} sources — use nativeMerge() so each input gets one wire`,
      );
    }
  }
  let inputDefs = [...(spec.inputs ?? [])];
  // Auto-include Plan advanced pins when wired (or options.advanced).
  if (spec.advancedInputs?.length) {
    const want = new Set(options.advanced ?? []);
    for (const adv of spec.advancedInputs) {
      if (want.has(adv.name) || (wireMapSafe[adv.name]?.length))
        inputDefs.push(adv);
    }
  }
  // Move type-specific pins (match Motus Move SyncPinsForType morph).
  const segType = (options.segmentType || options.text?.Type || 'PTP').toString().trim().toUpperCase();
  if (key === 'segment') {
    const isArm = segType === 'PTP' || segType === 'LIN' || segType === 'CIRC';
    inputDefs = inputDefs.filter((inp) => {
      if (inp.name === 'Goal' || inp.name === 'Blend') return isArm;
      if (inp.name === 'ToolState') return isArm || segType === 'SET';
      return true;
    });
  }
  if (spec.typeInputs?.[segType])
    inputDefs = [...inputDefs, ...spec.typeInputs[segType]];
  // Adjust height by pin count.
  if (spec.h && inputDefs.length)
    spec.h = Math.max(44, 24 + inputDefs.length * 20);
  const inputs = inputDefs.map((inp) => {
    const copy = { ...inp, _guid: id() };
    if (options.numbers?.[inp.name] !== undefined) copy.number = options.numbers[inp.name];
    if (options.points?.[inp.name] !== undefined) copy.point = options.points[inp.name];
    if (options.text?.[inp.name] !== undefined) copy.text = options.text[inp.name];
    if (options.bools?.[inp.name] !== undefined) copy.bool = options.bools[inp.name];
    if (options.angle?.[inp.name] !== undefined) copy.angle = true;
    if (options.useDegrees?.[inp.name] !== undefined) copy.useDegrees = options.useDegrees[inp.name];
    if (!copy.typeId && USE_PARAMETER_DATA.has(key))
      throw new Error(`missing typeId for ${key}.${inp.name}`);
    return copy;
  });
  const outputs = spec.outputs.map((out) => {
    const copy = { ...out, _guid: id() };
    if (!copy.typeId && USE_PARAMETER_DATA.has(key))
      throw new Error(`missing typeId for ${key} output ${out.name}`);
    return copy;
  });
  const node = { key, instance, inputs, outputs, spec };
  const advancedNames = new Set((spec.advancedInputs ?? []).map((a) => a.name));
  const presentAdvanced = inputs.filter((i) => advancedNames.has(i.name)).map((i) => i.name);
  const planFlags = key === 'plan' ? [
    item('AutoPlan', 'gh_bool', '1', options.autoPlan === false ? 'false' : 'true'),
    item('ShowCollision', 'gh_bool', '1', presentAdvanced.includes('Collision') ? 'true' : 'false'),
    item('ShowGroup', 'gh_bool', '1', presentAdvanced.includes('Group') ? 'true' : 'false'),
    item('ShowAttach', 'gh_bool', '1', presentAdvanced.includes('Attach') ? 'true' : 'false'),
    item('ShowRrtSettings', 'gh_bool', '1', presentAdvanced.includes('RrtSettings') ? 'true' : 'false'),
  ] : [];
  const progFlags = key === 'progPlan' ? [
    item('AutoPlan', 'gh_bool', '1', options.autoPlan === false ? 'false' : 'true'),
  ] : [];
  const moveFlags = key === 'segment' ? [
    item('MotionType', 'gh_string', '10', esc(segType)),
    item('ToolMode', 'gh_string', '10', esc(options.toolMode || 'Hold')),
    // Explicit pivot — Motus Move pin-morph used to wipe Attributes.Pivot on load.
    item('CanvasPivotX', 'gh_double', '6', String(x + spec.w / 2)),
    item('CanvasPivotY', 'gh_double', '6', String(y + spec.h / 2)),
  ] : [];
  // Motus Preview Write() fields — required for Scrub wire restore + ShowStart.
  // Examples default SS/ShowStart on (ghost start pose); pass bools.ShowStart:false to opt out.
  const showStart = key === 'preview'
    ? options.preview?.bools?.ShowStart !== false && options.bools?.ShowStart !== false
    : false;
  const previewPrefix = key === 'preview' ? [item('ColorMode', 'gh_int32', '3', '0')] : [];
  const previewSuffix = key === 'preview' ? [
    item('Position', 'gh_double', '6', '0'),
    item('ShowCustomColors', 'gh_bool', '1', 'false'),
    item('ShowDebugOutputs', 'gh_bool', '1', 'false'),
    item('ShowStart', 'gh_bool', '1', showStart ? 'true' : 'false'),
  ] : [];
  // Hidden = viewport preview off (IGH_PreviewObject); used for UR10e / Motus Robot in examples.
  const hiddenFlag = options.hidden === true
    ? [item('Hidden', 'gh_bool', '1', 'true')]
    : [];
  // AutoPlan before Description so GH_IO custom fields load reliably on Motus Program.
  const containerItems = [
    ...previewPrefix,
    ...progFlags,
    ...planFlags.filter((f) => f.includes('AutoPlan')),
    item('Description', 'gh_string', '10', esc(spec.desc ?? spec.name)),
    ...hiddenFlag,
    item('InstanceGuid', 'gh_guid', '9', instance),
    ...moveFlags,
    item('Name', 'gh_string', '10', spec.name),
    item('NickName', 'gh_string', '10', spec.nick),
    ...planFlags.filter((f) => !f.includes('AutoPlan')),
    ...previewSuffix,
  ];

  let containerChunks;
  if (USE_PARAMETER_DATA.has(key)) {
    containerChunks = `${bounds(x, y, spec.w, spec.h)}
                    ${parameterDataChunk(inputs, outputs, x, y, spec.w, wireMapSafe, options)}`;
  } else {
    const inChunks = inputs.map((inp, i) => {
      const sources = (wireMapSafe[inp.name] ?? []).map((ref) => ref._guid);
      let persistent = null;
      if (options.textList?.[inp.name])
        persistent = persistentTexts(options.textList[inp.name]);
      else if (inp.list && (options.numberList?.[inp.name] ?? (inp.name === 'Lengths' || inp.name === 'Joints' ? options.jointValues : null)))
        persistent = persistentNumbers(options.numberList?.[inp.name] ?? options.jointValues);
      else if (inp.bool !== undefined && !sources.length) persistent = persistentBool(inp.bool);
      else if (inp.number !== undefined && !sources.length) persistent = persistentNumbers([inp.number]);
      else if (inp.text !== undefined && !sources.length) persistent = persistentText(inp.text);
      else if (inp.point && !sources.length) persistent = persistentNumbers(inp.point);
      return paramInput(inp, i, x, y, spec.w, sources, persistent);
    });
    const outChunks = outputs.map((out, i) => paramOutput(out, i, x, y, spec.w));
    containerChunks = `${bounds(x, y, spec.w, spec.h)}
                    ${inChunks.join('\n                    ')}
                    ${outChunks.join('\n                    ')}`;
  }
  const chunkCount = USE_PARAMETER_DATA.has(key) ? 2 : (1 + inputs.length + outputs.length);
  return { xml: `<chunk name="Object" index="PLACEHOLDER">
              <items count="3">
                ${item('GUID', 'gh_guid', '9', spec.guid)}
                ${item('Lib', 'gh_guid', '9', MOTUS_LIB)}
                ${item('Name', 'gh_string', '10', spec.name)}
              </items>
              <chunks count="1">
                <chunk name="Container">
                  <items count="${containerItems.length}">
                    ${containerItems.join('\n                    ')}
                  </items>
                  <chunks count="${chunkCount}">
                    ${containerChunks}
                  </chunks>
                </chunk>
              </chunks>
            </chunk>`, node };
}

/**
 * Native GH Number Slider (integer). Wire outRef(node, 'Number') into Motus Body N.
 * Interval: 0=Float, 1=Integer, 2=Odd, 3=Even (GH_NumberSlider.Write).
 */
function nativeNumberSlider(x, y, { value = 6, min = 4, max = 12, nick = 'N', w = NATIVE.numberSlider.w } = {}) {
  const spec = NATIVE.numberSlider;
  const instance = id();
  const h = spec.h;
  const node = { key: 'numberSlider', instance, outputs: [{ name: 'Number', _guid: instance }] };
  return { xml: `<chunk name="Object" index="PLACEHOLDER">
              <items count="2">
                ${item('GUID', 'gh_guid', '9', spec.guid)}
                ${item('Name', 'gh_string', '10', spec.name)}
              </items>
              <chunks count="1">
                <chunk name="Container">
                  <items count="6">
                    ${item('Description', 'gh_string', '10', 'Numeric slider for a single value')}
                    ${item('InstanceGuid', 'gh_guid', '9', instance)}
                    ${item('Name', 'gh_string', '10', spec.name)}
                    ${item('NickName', 'gh_string', '10', esc(nick))}
                    ${item('Optional', 'gh_bool', '1', 'false')}
                    ${item('SourceCount', 'gh_int32', '3', '0')}
                  </items>
                  <chunks count="2">
                    ${bounds(x, y, w, h)}
                    <chunk name="Slider">
                      <items count="7">
                        ${item('Digits', 'gh_int32', '3', '0')}
                        ${item('GripDisplay', 'gh_int32', '3', '1')}
                        ${item('Interval', 'gh_int32', '3', '1')}
                        ${item('Max', 'gh_double', '6', String(max))}
                        ${item('Min', 'gh_double', '6', String(min))}
                        ${item('SnapCount', 'gh_int32', '3', '0')}
                        ${item('Value', 'gh_double', '6', String(value))}
                      </items>
                    </chunk>
                  </chunks>
                </chunk>
              </chunks>
            </chunk>`, node };
}

function motusScrub(x, y, value = 0, w = MOTUS.scrub.w) {
  const spec = MOTUS.scrub;
  const instance = id();
  const h = spec.h;
  const node = { key: 'scrub', instance, outputs: [{ name: 'Number', _guid: instance }] };
  // Match MotusScrubSlider.Write: ScrubValue + SnapToKeyframes on the container.
  return { xml: `<chunk name="Object" index="PLACEHOLDER">
              <items count="3">
                ${item('GUID', 'gh_guid', '9', spec.guid)}
                ${item('Lib', 'gh_guid', '9', MOTUS_LIB)}
                ${item('Name', 'gh_string', '10', spec.name)}
              </items>
              <chunks count="1">
                <chunk name="Container">
                  <items count="8">
                    ${item('Description', 'gh_string', '10', 'Normalized playback position (0–1) for Motus Preview; resize wider for finer control')}
                    ${item('InstanceGuid', 'gh_guid', '9', instance)}
                    ${item('Name', 'gh_string', '10', spec.name)}
                    ${item('NickName', 'gh_string', '10', spec.nick)}
                    ${item('Optional', 'gh_bool', '1', 'false')}
                    ${item('ScrubValue', 'gh_double', '6', String(value))}
                    ${item('SnapToKeyframes', 'gh_bool', '1', 'false')}
                    ${item('SourceCount', 'gh_int32', '3', '0')}
                  </items>
                  <chunks count="2">
                    ${bounds(x, y, w, h)}
                    <chunk name="Slider">
                      <items count="7">
                        ${item('Digits', 'gh_int32', '3', '3')}
                        ${item('GripDisplay', 'gh_int32', '3', '1')}
                        ${item('Interval', 'gh_int32', '3', '0')}
                        ${item('Max', 'gh_double', '6', '1')}
                        ${item('Min', 'gh_double', '6', '0')}
                        ${item('SnapCount', 'gh_int32', '3', '0')}
                        ${item('Value', 'gh_double', '6', String(value))}
                      </items>
                    </chunk>
                  </chunks>
                </chunk>
              </chunks>
            </chunk>`, node };
}

/**
 * Plan → Scrub → Preview spacing (hand-tuned GH clipboard on 02):
 *   Plan (px,py) → Scrub (+132,+76) → Preview (+373,+9)
 */
const PLAN_SCRUB_DX = 132;
const PLAN_SCRUB_DY = 76;
const PLAN_PREVIEW_DX = 373;
const PLAN_PREVIEW_DY = 9;

function previewWithScrub(planX, planY, trajectoryRef, options = {}) {
  const scrubW = options.scrubWidth ?? 220;
  const scrub = motusScrub(planX + PLAN_SCRUB_DX, planY + PLAN_SCRUB_DY, options.scrubValue ?? 0, scrubW);
  const previewInputs = {
    Trajectory: [trajectoryRef],
    Position: [outRef(scrub.node, 'Number')],
    ...(options.inputs ?? {}),
  };
  // Examples: Motus Preview SS (ShowStart) on by default.
  const previewOpts = {
    ...(options.preview ?? {}),
    bools: { ShowStart: true, ...(options.preview?.bools ?? {}) },
  };
  const preview = motusComponent(
    'preview',
    planX + PLAN_PREVIEW_DX,
    planY + PLAN_PREVIEW_DY,
    previewInputs,
    previewOpts,
  );
  return { scrub, preview };
}

function nativePanel(x, y, text, nick = '', w = NATIVE.panel.w, h = NATIVE.panel.h) {
  const instance = id();
  const node = { key: 'panel', instance, outputs: [{ name: 'Text', _guid: instance }] };
  return { xml: `<chunk name="Object" index="PLACEHOLDER">
              <items count="2">
                ${item('GUID', 'gh_guid', '9', NATIVE.panel.guid)}
                ${item('Name', 'gh_string', '10', 'Panel')}
              </items>
              <chunks count="1">
                <chunk name="Container">
                  <items count="8">
                    ${item('Description', 'gh_string', '10', 'A panel for custom notes and text values')}
                    ${item('InstanceGuid', 'gh_guid', '9', instance)}
                    ${item('Name', 'gh_string', '10', 'Panel')}
                    ${item('NickName', 'gh_string', '10', esc(nick))}
                    ${item('Optional', 'gh_bool', '1', 'false')}
                    ${item('ScrollRatio', 'gh_double', '6', '0')}
                    ${item('SourceCount', 'gh_int32', '3', '0')}
                    ${item('UserText', 'gh_string', '10', esc(text))}
                  </items>
                  <chunks count="2">
                    <chunk name="Attributes">
                      <items count="5">
                        ${item('Bounds', 'gh_drawing_rectanglef', '35', `\n                          <X>${x}</X>\n                          <Y>${y}</Y>\n                          <W>${w}</W>\n                          <H>${h}</H>\n                        `)}
                        ${item('MarginLeft', 'gh_int32', '3', '0')}
                        ${item('MarginRight', 'gh_int32', '3', '0')}
                        ${item('MarginTop', 'gh_int32', '3', '0')}
                        ${item('Pivot', 'gh_drawing_pointf', '31', `\n                          <X>${x}</X>\n                          <Y>${y + 0.60483}</Y>\n                        `)}
                      </items>
                    </chunk>
                    <chunk name="PanelProperties">
                      <items count="7">
                        ${item('Colour', 'gh_drawing_color', '36', '\n                          <ARGB>255;255;250;90</ARGB>\n                        ')}
                        ${item('DrawIndices', 'gh_bool', '1', 'true')}
                        ${item('DrawPaths', 'gh_bool', '1', 'true')}
                        ${item('Multiline', 'gh_bool', '1', 'true')}
                        ${item('SpecialCodes', 'gh_bool', '1', 'false')}
                        ${item('Stream', 'gh_bool', '1', 'false')}
                        ${item('Wrap', 'gh_bool', '1', 'true')}
                      </items>
                    </chunk>
                  </chunks>
                </chunk>
              </chunks>
            </chunk>`, node };
}

function nativeFilePath(x, y, path, filter = '*.urdf|*.urdf|All files|*.*') {
  const instance = id();
  const node = { key: 'filePath', instance, outputs: [{ name: 'Path', _guid: instance }] };
  return { xml: `<chunk name="Object" index="PLACEHOLDER">
              <items count="2">
                ${item('GUID', 'gh_guid', '9', NATIVE.filePath.guid)}
                ${item('Name', 'gh_string', '10', 'File Path')}
              </items>
              <chunks count="1">
                <chunk name="Container">
                  <items count="8">
                    ${item('Description', 'gh_string', '10', 'Contains a collection of file paths')}
                    ${item('ExpireOnFileEvent', 'gh_bool', '1', 'false')}
                    ${item('FileFilter', 'gh_string', '10', esc(filter))}
                    ${item('InstanceGuid', 'gh_guid', '9', instance)}
                    ${item('Name', 'gh_string', '10', 'File Path')}
                    ${item('NickName', 'gh_string', '10', esc(NATIVE.filePath.nick))}
                    ${item('Optional', 'gh_bool', '1', 'false')}
                    ${item('SourceCount', 'gh_int32', '3', '0')}
                  </items>
                  <chunks count="2">
                    ${bounds(x, y, NATIVE.filePath.w, NATIVE.filePath.h)}
                    ${persistentText(path)}
                  </chunks>
                </chunk>
              </chunks>
            </chunk>`, node };
}

function tcpGoalPlane(x, y) {
  const pt = nativeConstructPoint(x, y, [0.45, 0.15, 0.45]);
  const uz = nativeUnitZ(x, y - 60);
  const pl = nativePlane(x + 120, y, pt.node.outputs[0], uz.node.outputs[0]);
  return { pt, uz, node: pl.node, xml: [pt.xml, uz.xml, pl.xml] };
}

function nativeConstructPoint(x, y, coords) {
  const instance = id();
  const outGuid = id();
  const ins = ['X', 'Y', 'Z'].map((name, i) => {
    const g = id();
    return { name, _guid: g, xml: `<chunk name="param_input" index="${i}">
                      <items count="6">
                        ${item('Description', 'gh_string', '10', name + ' coordinate')}
                        ${item('InstanceGuid', 'gh_guid', '9', g)}
                        ${item('Name', 'gh_string', '10', name)}
                        ${item('NickName', 'gh_string', '10', name)}
                        ${item('Optional', 'gh_bool', '1', 'false')}
                        ${item('SourceCount', 'gh_int32', '3', '0')}
                      </items>
                      <chunks count="2">
                        ${bounds(x + 2, y + 2 + i * 14, 15, 14)}
                        ${persistentNumbers([coords[i]])}
                      </chunks>
                    </chunk>` };
  });
  const node = { key: 'constructPoint', instance, outputs: [{ name: 'Point', _guid: outGuid }] };
  return { xml: `<chunk name="Object" index="PLACEHOLDER">
              <items count="2">
                ${item('GUID', 'gh_guid', '9', NATIVE.constructPoint.guid)}
                ${item('Name', 'gh_string', '10', 'Construct Point')}
              </items>
              <chunks count="1">
                <chunk name="Container">
                  <items count="5">
                    ${item('Description', 'gh_string', '10', 'Construct a point from {xyz} coordinates')}
                    ${item('InstanceGuid', 'gh_guid', '9', instance)}
                    ${item('Name', 'gh_string', '10', 'Construct Point')}
                    ${item('NickName', 'gh_string', '10', 'GoalPt')}
                    ${item('SourceCount', 'gh_int32', '3', '0')}
                  </items>
                  <chunks count="5">
                    ${bounds(x, y, 44, 44)}
                    ${ins.map((i) => i.xml).join('\n                    ')}
                    <chunk name="param_output" index="0">
                      <items count="6">
                        ${item('Description', 'gh_string', '10', 'Point coordinate')}
                        ${item('InstanceGuid', 'gh_guid', '9', outGuid)}
                        ${item('Name', 'gh_string', '10', 'Point')}
                        ${item('NickName', 'gh_string', '10', 'Pt')}
                        ${item('Optional', 'gh_bool', '1', 'false')}
                        ${item('SourceCount', 'gh_int32', '3', '0')}
                      </items>
                      <chunks count="1">${bounds(x + 28, y + 14, 14, 14)}</chunks>
                    </chunk>
                  </chunks>
                </chunk>
              </chunks>
            </chunk>`, node };
}

/** Line SDL: Start point + Direction vector + Length → Line (for Motus Urdf Joint Axis). */
function nativeLineSdl(x, y, startRef, dirRef, length = 0.05) {
  const spec = NATIVE.lineSdl;
  const instance = id();
  const inGuids = [id(), id(), id()];
  const outGuid = id();
  const node = {
    key: 'lineSdl',
    instance,
    inputs: spec.inputs.map((n, i) => ({ name: n, _guid: inGuids[i] })),
    outputs: [{ name: 'Line', _guid: outGuid }],
  };
  const sources = [
    [startRef._guid],
    [dirRef._guid],
    [],
  ];
  const persist = [
    null,
    null,
    persistentNumbers([length]),
  ];
  const inChunks = spec.inputs.map((name, i) => {
    const srcItems = (sources[i] ?? []).map((s, si) => sourceItem(si, s)).join('\n');
    const chunks = [bounds(x + 2, y + 2 + i * 20, 17, 20)];
    if (persist[i]) chunks.push(persist[i]);
    return `<chunk name="param_input" index="${i}">
                      <items count="${6 + (sources[i]?.length ?? 0)}">
                        ${item('Description', 'gh_string', '10', name)}
                        ${item('InstanceGuid', 'gh_guid', '9', inGuids[i])}
                        ${item('Name', 'gh_string', '10', name)}
                        ${item('NickName', 'gh_string', '10', name[0])}
                        ${item('Optional', 'gh_bool', '1', 'false')}
                        ${srcItems}
                        ${item('SourceCount', 'gh_int32', '3', String(sources[i]?.length ?? 0))}
                      </items>
                      <chunks count="${chunks.length}">
                        ${chunks.join('\n                        ')}
                      </chunks>
                    </chunk>`;
  });
  const outChunk = `<chunk name="param_output" index="0">
                      <items count="6">
                        ${item('Description', 'gh_string', '10', 'Line')}
                        ${item('InstanceGuid', 'gh_guid', '9', outGuid)}
                        ${item('Name', 'gh_string', '10', 'Line')}
                        ${item('NickName', 'gh_string', '10', 'L')}
                        ${item('Optional', 'gh_bool', '1', 'false')}
                        ${item('SourceCount', 'gh_int32', '3', '0')}
                      </items>
                      <chunks count="1">
                        ${bounds(x + spec.w - 18, y + 2, 16, 20)}
                      </chunks>
                    </chunk>`;
  return {
    xml: `<chunk name="Object" index="PLACEHOLDER">
              <items count="2">
                ${item('GUID', 'gh_guid', '9', spec.guid)}
                ${item('Name', 'gh_string', '10', spec.name)}
              </items>
              <chunks count="1">
                <chunk name="Container">
                  <items count="4">
                    ${item('Description', 'gh_string', '10', 'Line SDL')}
                    ${item('InstanceGuid', 'gh_guid', '9', instance)}
                    ${item('Name', 'gh_string', '10', spec.name)}
                    ${item('NickName', 'gh_string', '10', spec.nick)}
                  </items>
                  <chunks count="4">
                    ${bounds(x, y, spec.w, spec.h)}
                    ${inChunks.join('\n                    ')}
                    ${outChunk}
                  </chunks>
                </chunk>
              </chunks>
            </chunk>`,
    node,
  };
}

function nativeUnitZ(x, y) {
  const instance = id();
  const outGuid = id();
  const node = { key: 'unitZ', instance, outputs: [{ name: 'Vector', _guid: outGuid }] };
  return { xml: `<chunk name="Object" index="PLACEHOLDER">
              <items count="2">
                ${item('GUID', 'gh_guid', '9', NATIVE.unitZ.guid)}
                ${item('Name', 'gh_string', '10', 'Unit Z')}
              </items>
              <chunks count="1">
                <chunk name="Container">
                  <items count="5">
                    ${item('Description', 'gh_string', '10', 'Unit vector along the Z-axis')}
                    ${item('InstanceGuid', 'gh_guid', '9', instance)}
                    ${item('Name', 'gh_string', '10', 'Unit Z')}
                    ${item('NickName', 'gh_string', '10', 'Z')}
                    ${item('SourceCount', 'gh_int32', '3', '0')}
                  </items>
                  <chunks count="2">
                    ${bounds(x, y, 44, 22)}
                    <chunk name="param_output" index="0">
                      <items count="6">
                        ${item('Description', 'gh_string', '10', 'Unit Z vector')}
                        ${item('InstanceGuid', 'gh_guid', '9', outGuid)}
                        ${item('Name', 'gh_string', '10', 'Vector')}
                        ${item('NickName', 'gh_string', '10', 'Z')}
                        ${item('Optional', 'gh_bool', '1', 'false')}
                        ${item('SourceCount', 'gh_int32', '3', '0')}
                      </items>
                      <chunks count="1">${bounds(x + 28, y + 4, 14, 14)}</chunks>
                    </chunk>
                  </chunks>
                </chunk>
              </chunks>
            </chunk>`, node };
}

function nativeUnitX(x, y) {
  const instance = id();
  const outGuid = id();
  const node = { key: 'unitX', instance, outputs: [{ name: 'Vector', _guid: outGuid }] };
  return { xml: `<chunk name="Object" index="PLACEHOLDER">
              <items count="2">
                ${item('GUID', 'gh_guid', '9', NATIVE.unitX.guid)}
                ${item('Name', 'gh_string', '10', 'Unit X')}
              </items>
              <chunks count="1">
                <chunk name="Container">
                  <items count="5">
                    ${item('Description', 'gh_string', '10', 'Unit vector along the X-axis')}
                    ${item('InstanceGuid', 'gh_guid', '9', instance)}
                    ${item('Name', 'gh_string', '10', 'Unit X')}
                    ${item('NickName', 'gh_string', '10', 'X')}
                    ${item('SourceCount', 'gh_int32', '3', '0')}
                  </items>
                  <chunks count="2">
                    ${bounds(x, y, 44, 22)}
                    <chunk name="param_output" index="0">
                      <items count="6">
                        ${item('Description', 'gh_string', '10', 'Unit X vector')}
                        ${item('InstanceGuid', 'gh_guid', '9', outGuid)}
                        ${item('Name', 'gh_string', '10', 'Vector')}
                        ${item('NickName', 'gh_string', '10', 'X')}
                        ${item('Optional', 'gh_bool', '1', 'false')}
                        ${item('SourceCount', 'gh_int32', '3', '0')}
                      </items>
                      <chunks count="1">${bounds(x + 28, y + 4, 14, 14)}</chunks>
                    </chunk>
                  </chunks>
                </chunk>
              </chunks>
            </chunk>`, node };
}

function nativePlane(x, y, originRef, normalRef) {
  const instance = id();
  const outGuid = id();
  const inOrigin = id();
  const inNormal = id();
  const node = { key: 'plane', instance, outputs: [{ name: 'Plane', _guid: outGuid }] };
  return { xml: `<chunk name="Object" index="PLACEHOLDER">
              <items count="2">
                ${item('GUID', 'gh_guid', '9', NATIVE.plane.guid)}
                ${item('Name', 'gh_string', '10', 'Plane Normal')}
              </items>
              <chunks count="1">
                <chunk name="Container">
                  <items count="5">
                    ${item('Description', 'gh_string', '10', 'Create a plane from an origin point and a Z-axis vector')}
                    ${item('InstanceGuid', 'gh_guid', '9', instance)}
                    ${item('Name', 'gh_string', '10', 'Plane Normal')}
                    ${item('NickName', 'gh_string', '10', 'Pl')}
                    ${item('SourceCount', 'gh_int32', '3', '0')}
                  </items>
                  <chunks count="4">
                    ${bounds(x, y, 44, 44)}
                    <chunk name="param_input" index="0">
                      <items count="7">
                        ${item('Description', 'gh_string', '10', 'Origin of plane')}
                        ${item('InstanceGuid', 'gh_guid', '9', inOrigin)}
                        ${item('Name', 'gh_string', '10', 'Origin')}
                        ${item('NickName', 'gh_string', '10', 'O')}
                        ${item('Optional', 'gh_bool', '1', 'false')}
                        ${sourceItem(0, originRef._guid)}
                        ${item('SourceCount', 'gh_int32', '3', '1')}
                      </items>
                      <chunks count="1">${bounds(x + 2, y + 2, 15, 14)}</chunks>
                    </chunk>
                    <chunk name="param_input" index="1">
                      <items count="7">
                        ${item('Description', 'gh_string', '10', 'Z-Axis direction of plane')}
                        ${item('InstanceGuid', 'gh_guid', '9', inNormal)}
                        ${item('Name', 'gh_string', '10', 'Z-Axis')}
                        ${item('NickName', 'gh_string', '10', 'Z')}
                        ${item('Optional', 'gh_bool', '1', 'false')}
                        ${sourceItem(0, normalRef._guid)}
                        ${item('SourceCount', 'gh_int32', '3', '1')}
                      </items>
                      <chunks count="1">${bounds(x + 2, y + 18, 15, 14)}</chunks>
                    </chunk>
                    <chunk name="param_output" index="0">
                      <items count="6">
                        ${item('Description', 'gh_string', '10', 'Plane definition')}
                        ${item('InstanceGuid', 'gh_guid', '9', outGuid)}
                        ${item('Name', 'gh_string', '10', 'Plane')}
                        ${item('NickName', 'gh_string', '10', 'P')}
                        ${item('Optional', 'gh_bool', '1', 'false')}
                        ${item('SourceCount', 'gh_int32', '3', '0')}
                      </items>
                      <chunks count="1">${bounds(x + 28, y + 14, 14, 14)}</chunks>
                    </chunk>
                  </chunks>
                </chunk>
              </chunks>
            </chunk>`, node };
}

function nativeXYPlane(x, y) {
  const instance = id();
  const outGuid = id();
  const node = { key: 'xyPlane', instance, outputs: [{ name: 'Plane', _guid: outGuid }] };
  return { xml: `<chunk name="Object" index="PLACEHOLDER">
              <items count="2">
                ${item('GUID', 'gh_guid', '9', NATIVE.xyPlane.guid)}
                ${item('Name', 'gh_string', '10', 'XY Plane')}
              </items>
              <chunks count="1">
                <chunk name="Container">
                  <items count="5">
                    ${item('Description', 'gh_string', '10', 'World XY plane')}
                    ${item('InstanceGuid', 'gh_guid', '9', instance)}
                    ${item('Name', 'gh_string', '10', 'XY Plane')}
                    ${item('NickName', 'gh_string', '10', 'XY')}
                    ${item('SourceCount', 'gh_int32', '3', '0')}
                  </items>
                  <chunks count="2">
                    ${bounds(x, y, 44, 22)}
                    <chunk name="param_output" index="0">
                      <items count="6">
                        ${item('Description', 'gh_string', '10', 'Plane in world XY')}
                        ${item('InstanceGuid', 'gh_guid', '9', outGuid)}
                        ${item('Name', 'gh_string', '10', 'Plane')}
                        ${item('NickName', 'gh_string', '10', 'P')}
                        ${item('Optional', 'gh_bool', '1', 'false')}
                        ${item('SourceCount', 'gh_int32', '3', '0')}
                      </items>
                      <chunks count="1">${bounds(x + 28, y + 4, 14, 14)}</chunks>
                    </chunk>
                  </chunks>
                </chunk>
              </chunks>
            </chunk>`, node };
}

/** Native Center Box — Base plane + full-extent X/Y/Z (meters). */
function nativeCenterBox(x, y, baseRef, size) {
  const instance = id();
  const outGuid = id();
  const baseIn = id();
  const sizeIns = ['X', 'Y', 'Z'].map((name, i) => {
    const g = id();
    return `<chunk name="param_input" index="${i + 1}">
                      <items count="6">
                        ${item('Description', 'gh_string', '10', `Size of box in {${name.toLowerCase()}} direction.`)}
                        ${item('InstanceGuid', 'gh_guid', '9', g)}
                        ${item('Name', 'gh_string', '10', name)}
                        ${item('NickName', 'gh_string', '10', name)}
                        ${item('Optional', 'gh_bool', '1', 'false')}
                        ${item('SourceCount', 'gh_int32', '3', '0')}
                      </items>
                      <chunks count="2">
                        ${bounds(x + 2, y + 16 + i * 14, 15, 14)}
                        ${persistentNumbers([size[i]])}
                      </chunks>
                    </chunk>`;
  });
  const node = { key: 'centerBox', instance, outputs: [{ name: 'Box', _guid: outGuid }] };
  return {
    xml: `<chunk name="Object" index="PLACEHOLDER">
              <items count="2">
                ${item('GUID', 'gh_guid', '9', NATIVE.centerBox.guid)}
                ${item('Name', 'gh_string', '10', 'Center Box')}
              </items>
              <chunks count="1">
                <chunk name="Container">
                  <items count="5">
                    ${item('Description', 'gh_string', '10', 'Create a box centered on a plane.')}
                    ${item('InstanceGuid', 'gh_guid', '9', instance)}
                    ${item('Name', 'gh_string', '10', 'Center Box')}
                    ${item('NickName', 'gh_string', '10', 'Box')}
                    ${item('SourceCount', 'gh_int32', '3', '0')}
                  </items>
                  <chunks count="6">
                    ${bounds(x, y, NATIVE.centerBox.w, NATIVE.centerBox.h)}
                    <chunk name="param_input" index="0">
                      <items count="7">
                        ${item('Description', 'gh_string', '10', 'Base plane')}
                        ${item('InstanceGuid', 'gh_guid', '9', baseIn)}
                        ${item('Name', 'gh_string', '10', 'Base')}
                        ${item('NickName', 'gh_string', '10', 'B')}
                        ${item('Optional', 'gh_bool', '1', 'false')}
                        ${sourceItem(0, baseRef._guid)}
                        ${item('SourceCount', 'gh_int32', '3', '1')}
                      </items>
                      <chunks count="1">${bounds(x + 2, y + 2, 15, 14)}</chunks>
                    </chunk>
                    ${sizeIns.join('\n                    ')}
                    <chunk name="param_output" index="0">
                      <items count="6">
                        ${item('Description', 'gh_string', '10', 'Resulting box')}
                        ${item('InstanceGuid', 'gh_guid', '9', outGuid)}
                        ${item('Name', 'gh_string', '10', 'Box')}
                        ${item('NickName', 'gh_string', '10', 'B')}
                        ${item('Optional', 'gh_bool', '1', 'false')}
                        ${item('SourceCount', 'gh_int32', '3', '0')}
                      </items>
                      <chunks count="1">${bounds(x + 38, y + 24, 14, 14)}</chunks>
                    </chunk>
                  </chunks>
                </chunk>
              </chunks>
            </chunk>`,
    node,
  };
}

let lastGraphMeta = null;

function buildGraph(objects) {
  lastGraphMeta = objects._meta;
  const chunks = objects.map((o, i) => {
    const xml = typeof o === 'string' ? o : o.xml;
    return xml.replace('index="PLACEHOLDER"', `index="${i}"`);
  });
  const docId = id();
  const { fileName, description } = objects._meta;
  return `<?xml version="1.0" encoding="utf-8" standalone="yes"?>
<Archive name="Root">
  <items count="1">
    <item name="ArchiveVersion" type_name="gh_version" type_code="80">
      <Major>0</Major>
      <Minor>2</Minor>
      <Revision>2</Revision>
    </item>
  </items>
  <chunks count="1">
    <chunk name="Definition">
      <items count="1">
        <item name="plugin_version" type_name="gh_version" type_code="80">
          <Major>1</Major>
          <Minor>0</Minor>
          <Revision>8</Revision>
        </item>
      </items>
      <chunks count="6">
        <chunk name="DocumentHeader">
          <items count="5">
            ${item('DocumentID', 'gh_guid', '9', docId)}
            ${item('Preview', 'gh_string', '10', 'Shaded')}
            ${item('PreviewMeshType', 'gh_int32', '3', '1')}
            ${item('PreviewNormal', 'gh_drawing_color', '36', '\n              <ARGB>150;150;0;0</ARGB>\n            ')}
            ${item('PreviewSelected', 'gh_drawing_color', '36', '\n              <ARGB>150;0;150;0</ARGB>\n            ')}
          </items>
        </chunk>
        <chunk name="DefinitionProperties">
          <items count="4">
            ${item('Date', 'gh_date', '8', String(Date.now() * 10000 + 621355968000000000))}
            ${item('Description', 'gh_string', '10', esc(description))}
            ${item('KeepOpen', 'gh_bool', '1', 'true')}
            ${item('Name', 'gh_string', '10', fileName)}
          </items>
          <chunks count="3">
            <chunk name="Revisions"><items count="1">${item('RevisionCount', 'gh_int32', '3', '0')}</items></chunk>
            <chunk name="Projection">
              <items count="2">
                ${item('Target', 'gh_drawing_point', '30', '\n                  <X>400</X>\n                  <Y>200</Y>\n                ')}
                ${item('Zoom', 'gh_single', '5', '0.75')}
              </items>
            </chunk>
            <chunk name="Views"><items count="1">${item('ViewCount', 'gh_int32', '3', '0')}</items></chunk>
          </chunks>
        </chunk>
        <chunk name="RcpLayout"><items count="1">${item('GroupCount', 'gh_int32', '3', '0')}</items></chunk>
        <chunk name="ValueTable">
          <items count="2">
            ${item('K3DSettings.UnitLength', 'gh_string', '10', 'auto')}
            ${item('K3DSettings.UnitsSystem', 'gh_string', '10', 'SI')}
          </items>
        </chunk>
        <chunk name="GHALibraries">
          <items count="1">${item('Count', 'gh_int32', '3', '2')}</items>
          <chunks count="2">
            <chunk name="Library" index="0">
              <items count="4">
                ${item('Author', 'gh_string', '10', 'Robert McNeel &amp; Associates')}
                ${item('Id', 'gh_guid', '9', '00000000-0000-0000-0000-000000000000')}
                ${item('Name', 'gh_string', '10', 'Grasshopper')}
                ${item('Version', 'gh_string', '10', '8.32.26160.13001')}
              </items>
            </chunk>
            <chunk name="Library" index="1">
              <items count="6">
                ${item('AssemblyFullName', 'gh_string', '10', `Motus.GH, Version=${PLUGIN_ASSEMBLY_VERSION}, Culture=neutral, PublicKeyToken=null`)}
                ${item('AssemblyVersion', 'gh_string', '10', PLUGIN_ASSEMBLY_VERSION)}
                ${item('Author', 'gh_string', '10', 'Motus')}
                ${item('Id', 'gh_guid', '9', MOTUS_LIB)}
                ${item('Name', 'gh_string', '10', 'Motus')}
                ${item('Version', 'gh_string', '10', PLUGIN_VERSION)}
              </items>
            </chunk>
          </chunks>
        </chunk>
        <chunk name="DefinitionObjects">
          <items count="1">${item('ObjectCount', 'gh_int32', '3', String(objects.length))}</items>
          <chunks count="${objects.length}">
            ${chunks.join('\n            ')}
          </chunks>
        </chunk>
      </chunks>
    </chunk>
  </chunks>
</Archive>`;
}

function outRef(node, outputName) {
  const out = node.outputs.find((o) => o.name === outputName || o.nick === outputName);
  if (!out) throw new Error(`Missing output ${outputName} on ${node.key}`);
  return out;
}

function instanceOf(obj) {
  if (obj?.node?.instance) return obj.node.instance;
  if (obj?.instance) return obj.instance;
  throw new Error('object missing InstanceGuid');
}

/** Merge N streams — one wire per Data pin (never multi-source a Motus list pin). */
function nativeMerge(x, y, refs) {
  if (!refs?.length) throw new Error('nativeMerge requires at least one ref');
  const n = refs.length;
  const instance = id();
  const outGuid = id();
  const h = Math.max(44, 24 + n * 20);
  const w = NATIVE.merge.w;
  const inChunks = refs.map((ref, i) => {
    const g = id();
    const src = sourceItem(0, ref._guid);
    return `<chunk name="InputParam" index="${i}">
                          <items count="9">
                            ${item('Access', 'gh_int32', '3', '2')}
                            ${item('Description', 'gh_string', '10', `Data stream ${i + 1}`)}
                            ${item('InstanceGuid', 'gh_guid', '9', g)}
                            ${item('Mutable', 'gh_bool', '1', 'false')}
                            ${item('Name', 'gh_string', '10', `Data ${i + 1}`)}
                            ${item('NickName', 'gh_string', '10', `D${i + 1}`)}
                            ${item('Optional', 'gh_bool', '1', 'true')}
                            ${src}
                            ${item('SourceCount', 'gh_int32', '3', '1')}
                          </items>
                          <chunks count="1">
                            ${paramAttrBounds(x + 2, y + 2 + i * 20, 16, 20)}
                          </chunks>
                        </chunk>`;
  });
  const idItems = [
    item('InputCount', 'gh_int32', '3', String(n)),
    ...refs.map((_, i) => `                        <item name="InputId" index="${i}" type_name="gh_guid" type_code="9">${PTYPE.generic}</item>`),
    item('OutputCount', 'gh_int32', '3', '1'),
    `                        <item name="OutputId" index="0" type_name="gh_guid" type_code="9">${PTYPE.generic}</item>`,
  ];
  const node = { key: 'merge', instance, outputs: [{ name: 'Result', _guid: outGuid }] };
  return { xml: `<chunk name="Object" index="PLACEHOLDER">
              <items count="2">
                ${item('GUID', 'gh_guid', '9', NATIVE.merge.guid)}
                ${item('Name', 'gh_string', '10', 'Merge')}
              </items>
              <chunks count="1">
                <chunk name="Container">
                  <items count="4">
                    ${item('Description', 'gh_string', '10', 'Merge a bunch of data streams')}
                    ${item('InstanceGuid', 'gh_guid', '9', instance)}
                    ${item('Name', 'gh_string', '10', 'Merge')}
                    ${item('NickName', 'gh_string', '10', 'Merge')}
                  </items>
                  <chunks count="2">
                    ${bounds(x, y, w, h)}
                    <chunk name="ParameterData">
                      <items count="${idItems.length}">
                        ${idItems.join('\n                        ')}
                      </items>
                      <chunks count="${n + 1}">
                        ${inChunks.join('\n                        ')}
                        <chunk name="OutputParam" index="0">
                          <items count="7">
                            ${item('Access', 'gh_int32', '3', '2')}
                            ${item('Description', 'gh_string', '10', 'Result of merge')}
                            ${item('InstanceGuid', 'gh_guid', '9', outGuid)}
                            ${item('Name', 'gh_string', '10', 'Result')}
                            ${item('NickName', 'gh_string', '10', 'R')}
                            ${item('Optional', 'gh_bool', '1', 'false')}
                            ${item('SourceCount', 'gh_int32', '3', '0')}
                          </items>
                          <chunks count="1">
                            ${paramAttrBounds(x + w - 14, y + 2, 12, Math.max(20, h - 4))}
                          </chunks>
                        </chunk>
                      </chunks>
                    </chunk>
                  </chunks>
                </chunk>
              </chunks>
            </chunk>`, node };
}

function nativeScribble(x, y, text, size = 22, w = null) {
  const instance = id();
  const tw = w ?? Math.max(120, String(text).length * size * 0.55);
  const th = size * 1.15;
  const node = { key: 'scribble', instance, outputs: [] };
  return { xml: `<chunk name="Object" index="PLACEHOLDER">
              <items count="2">
                ${item('GUID', 'gh_guid', '9', NATIVE.scribble.guid)}
                ${item('Name', 'gh_string', '10', 'Scribble')}
              </items>
              <chunks count="1">
                <chunk name="Container">
                  <items count="13">
                    ${item('Bold', 'gh_bool', '1', 'true')}
                    ${item('Ca', 'gh_drawing_pointf', '31', `\n                      <X>${x}</X>\n                      <Y>${y}</Y>\n                    `)}
                    ${item('Cb', 'gh_drawing_pointf', '31', `\n                      <X>${x + tw}</X>\n                      <Y>${y}</Y>\n                    `)}
                    ${item('Cc', 'gh_drawing_pointf', '31', `\n                      <X>${x + tw}</X>\n                      <Y>${y + th}</Y>\n                    `)}
                    ${item('Cd', 'gh_drawing_pointf', '31', `\n                      <X>${x}</X>\n                      <Y>${y + th}</Y>\n                    `)}
                    ${item('Description', 'gh_string', '10', 'A quick note')}
                    ${item('Font', 'gh_string', '10', 'Consolas')}
                    ${item('InstanceGuid', 'gh_guid', '9', instance)}
                    ${item('Italic', 'gh_bool', '1', 'false')}
                    ${item('Name', 'gh_string', '10', 'Scribble')}
                    ${item('NickName', 'gh_string', '10', 'Scribble')}
                    ${item('Size', 'gh_single', '5', String(size))}
                    ${item('Text', 'gh_string', '10', esc(text))}
                  </items>
                  <chunks count="1">
                    ${bounds(x - 5, y - 5, tw + 10, th + 10)}
                  </chunks>
                </chunk>
              </chunks>
            </chunk>`, node };
}

function nativeGroup(nick, members, colourArgb) {
  const instance = id();
  const ids = members.map((m) => instanceOf(m));
  const idItems = ids.map((g, i) => `                    <item name="ID" index="${i}" type_name="gh_guid" type_code="9">${g}</item>`);
  const node = { key: 'ghGroup', instance, outputs: [] };
  return { xml: `<chunk name="Object" index="PLACEHOLDER">
              <items count="2">
                ${item('GUID', 'gh_guid', '9', NATIVE.group.guid)}
                ${item('Name', 'gh_string', '10', 'Group')}
              </items>
              <chunks count="1">
                <chunk name="Container">
                  <items count="${7 + ids.length}">
                    ${item('Border', 'gh_int32', '3', '1')}
                    ${item('Colour', 'gh_drawing_color', '36', `\n                      <ARGB>${colourArgb}</ARGB>\n                    `)}
                    ${item('Description', 'gh_string', '10', 'A group of Grasshopper objects')}
                    ${idItems.join('\n')}
                    ${item('ID_Count', 'gh_int32', '3', String(ids.length))}
                    ${item('InstanceGuid', 'gh_guid', '9', instance)}
                    ${item('Name', 'gh_string', '10', 'Group')}
                    ${item('NickName', 'gh_string', '10', esc(nick))}
                  </items>
                  <chunks count="1">
                    <chunk name="Attributes" />
                  </chunks>
                </chunk>
              </chunks>
            </chunk>`, node };
}

function ur10eRobot(x, y) {
  // Viewport preview off — Motus Preview owns the robot mesh (avoid double draw).
  return motusComponent('ur10e', x, y, {}, { hidden: true });
}

/** 01 — quick plan: sequential joint + TCP Pose LIN + Export / Waypoints / Preview (was 01+02+12). */
function graph01() {
  // Bands: title → robot | goals → plan/preview (right). Clear Y gaps between groups.
  const title = nativeScribble(40, -60, '01  Quick plan', 28);
  const note = nativePanel(420, -60, 'Auto Plan on. Scrub Preview when Status OK.', 'Note', 260, 40);
  const robot = ur10eRobot(40, 40);
  const start = motusComponent('joints', 40, 180, {}, { jointValues: MOTION_START });
  const goalJoint = motusComponent('joints', 40, 360, {}, { jointValues: GOAL_JOINTS });
  const tcp = motusComponent('tcpPose', 240, 260, {
    Robot: [outRef(robot.node, 'Robot')],
    State: [outRef(goalJoint.node, 'State')],
  });
  const uz = nativeUnitZ(40, 500);
  const ptLin = nativeConstructPoint(40, 560, [0.48, 0.18, 0.48]);
  const plLin = nativePlane(200, 560, ptLin.node.outputs[0], uz.node.outputs[0]);
  const goalsMerge = nativeMerge(360, 400, [
    outRef(goalJoint.node, 'State'),
    outRef(tcp.node, 'Plane'),
    outRef(plLin.node, 'Plane'),
  ]);
  const plan = motusComponent('plan', 560, 200, {
    Robot: [outRef(robot.node, 'Robot')],
    Goal: [outRef(goalsMerge.node, 'Result')],
    Start: [outRef(start.node, 'State')],
  });
  const { scrub, preview } = previewWithScrub(560, 200, outRef(plan.node, 'Trajectory'));
  const waypoints = motusComponent('waypoints', 560 + PLAN_PREVIEW_DX, 380, { Trajectory: [outRef(plan.node, 'Trajectory')] });
  const exp = motusComponent('export', 560 + PLAN_PREVIEW_DX, 540, { Trajectory: [outRef(plan.node, 'Trajectory')] });
  const gRobot = nativeGroup('Robot + start', [robot, start], GROUP_COLOUR.robot);
  const gGoals = nativeGroup('Goals (merge → Plan)', [goalJoint, tcp, { xml: uz.xml, node: uz.node }, ptLin, plLin, goalsMerge], GROUP_COLOUR.goals);
  const gOut = nativeGroup('Plan + preview', [plan, scrub, preview, waypoints, exp], GROUP_COLOUR.preview);
  const objs = [
    title, note, robot, start, goalJoint, tcp,
    { xml: uz.xml }, { xml: ptLin.xml }, { xml: plLin.xml }, goalsMerge,
    plan, scrub, preview, waypoints, exp,
    gRobot, gGoals, gOut,
  ];
  objs._meta = {
    fileName: '01_quick_plan.ghx',
    description: 'Quick plan: sequential Joint State + TCP Pose LIN + Plane goal (via Merge) -> Preview / Export / Waypoints. Auto Plan on; drag Motus Scrub or Play.',
  };
  return buildGraph(objs);
}

/** 02 — collision RRT + shapes + SRDF/group/attach (was 03+04+05). */
function graph02() {
  // Bands (no overlap): robot y40 → obstacles y420 → attach/RRT y760 → plan right.
  const title = nativeScribble(40, -60, '02  Collision + SRDF', 28);
  const note = nativePanel(420, -60, 'RRT detours the sphere. Group pin unwired until OMPL fix.', 'Note', 300, 40);
  const robot = ur10eRobot(40, 40);
  const start = motusComponent('joints', 40, 180, {}, { jointValues: COLLISION_START });
  const goal = motusComponent('joints', 40, 320, {}, { jointValues: COLLISION_GOAL });
  // Blocker must intersect the joint-linear mid path (mesh checker is envelope-tight).
  const sphereCenter = nativeConstructPoint(40, 420, [0.76, 0.50, 0.73]);
  const sphere = motusComponent('colSphere', 220, 420, {
    Center: [outRef(sphereCenter.node, 'Point')],
  }, { text: { Name: 'block' }, numbers: { Radius: 0.18 } });
  const uz = nativeUnitZ(40, 520);
  const boxOrigin = nativeConstructPoint(40, 580, [0.70, 0.20, 0.04]);
  const boxPlane = nativePlane(200, 580, boxOrigin.node.outputs[0], uz.node.outputs[0]);
  const box = motusComponent('colBox', 380, 560, { Plane: [outRef(boxPlane.node, 'Plane')] }, {
    text: { Name: 'table' },
    numbers: { HalfX: 0.25, HalfY: 0.18, HalfZ: 0.02 },
  });
  const obstaclesMerge = nativeMerge(520, 460, [
    outRef(sphere.node, 'Object'),
    outRef(box.node, 'Object'),
  ]);
  const srdfPanel = nativePanel(40, 680, absPath('examples/srdf/table_base.srdf'), 'Srdf', 280, 40);
  const scene = motusComponent('colScene', 680, 420, {
    Objects: [outRef(obstaclesMerge.node, 'Result')],
    Srdf: [outRef(srdfPanel.node, 'Text')],
  });
  const group = motusComponent('group', 860, 420, { Group: [outRef(scene.node, 'Groups')] });
  // Attach + RRT band — entirely below obstacles (gap ~80px).
  const graspCenter = nativeConstructPoint(40, 780, [0, 0, 0.02]);
  const grasp = motusComponent('colSphere', 220, 780, {
    Center: [outRef(graspCenter.node, 'Point')],
  }, { text: { Name: 'grasp' }, numbers: { Radius: 0.03 } });
  const attach = motusComponent('attach', 400, 780, { Object: [outRef(grasp.node, 'Object')] }, { text: { Name: 'grasp' } });
  const rrt = motusComponent('rrtSettings', 620, 760, {});
  // Motus Planning Group stays on canvas. Plan includes an unwired Group pin (ShowGroup)
  // so ParameterData order is Collision→Group→Attach→Rrt (required for GH deserialize).
  // Wire Group → Plan after Motus.OMPL GroupMap fix is loaded + Rhino restart.
  const plan = motusComponent('plan', 1080, 200, {
    Robot: [outRef(robot.node, 'Robot')],
    Goal: [outRef(goal.node, 'State')],
    Start: [outRef(start.node, 'State')],
    Collision: [outRef(scene.node, 'Scene')],
    Attach: [outRef(attach.node, 'Attach')],
    RrtSettings: [outRef(rrt.node, 'Settings')],
  }, { advanced: ['Collision', 'Group', 'Attach', 'RrtSettings'] });
  const { scrub, preview } = previewWithScrub(1080, 200, outRef(plan.node, 'Trajectory'));
  const gRobot = nativeGroup('Robot + joints', [robot, start, goal], GROUP_COLOUR.robot);
  const gCol = nativeGroup('Obstacles + SRDF', [
    sphereCenter, sphere, boxOrigin, { xml: uz.xml, node: uz.node }, boxPlane, box,
    obstaclesMerge, srdfPanel, scene, group,
  ], GROUP_COLOUR.collision);
  const gAttach = nativeGroup('Attach + RRT', [graspCenter, grasp, attach, rrt], GROUP_COLOUR.collision);
  const gOut = nativeGroup('Plan + preview', [plan, scrub, preview], GROUP_COLOUR.preview);
  const objs = [
    title, note, robot, start, goal,
    { xml: sphereCenter.xml }, sphere,
    { xml: boxOrigin.xml }, { xml: uz.xml }, { xml: boxPlane.xml }, box, obstaclesMerge,
    srdfPanel, scene, group,
    { xml: graspCenter.xml }, grasp, attach, rrt, plan, scrub, preview,
    gRobot, gCol, gAttach, gOut,
  ];
  objs._meta = {
    fileName: '02_collision_srdf.ghx',
    description: 'Collision RRT: sphere blocks the joint-linear mid-path so Plan detours. ColSphere+ColBox via Merge → ColScene (SRDF) + Attach + RRT. Auto Plan on; scrub Preview.',
  };
  return buildGraph(objs);
}

/** 03 — URDF load + base/tool frames + Robotiq mesh (was 06+07+09+10). */
function graph03() {
  // Bands in main canvas (x≥40): URDF/base → tool → plan/preview.
  const title = nativeScribble(40, -60, '03  URDF + tool frames', 28);
  const note = nativePanel(420, -60, 'Custom URDF + Tool TCP. Preview ShowStart on.', 'Note', 280, 40);
  const urdfFile = nativeFilePath(40, 40, absPath('resources/robots/ur10e_robotiq/ur10e_robotiq.urdf'));
  const basePl = nativeXYPlane(40, 160);
  const tcpPt = nativeConstructPoint(40, 280, [0, 0, 0.1633]);
  const ux = nativeUnitX(40, 360);
  const tcpPl = nativePlane(220, 280, tcpPt.node.outputs[0], ux.node.outputs[0]);
  const meshPath = nativeFilePath(40, 460, absPath('resources/tools/robotiq_2f85_tcp_local.stl'), '*.stl|*.stl|All files|*.*');
  const loadMesh = motusComponent('loadMesh', 260, 440, {
    Path: [outRef(meshPath.node, 'Path')],
  });
  const tool = motusComponent('tool', 440, 300, {
    TCP: [outRef(tcpPl.node, 'Plane')],
    Geometry: [outRef(loadMesh.node, 'Mesh')],
  }, { text: { Name: 'robotiq_2f85', Capabilities: 'Robotiq2F85' } });
  const robot = motusComponent('robot', 660, 40, {
    Path: [outRef(urdfFile.node, 'Path')],
    Base: [outRef(basePl.node, 'Plane')],
    Tool: [outRef(tool.node, 'Tool')],
  }, { text: { BaseLink: 'base_link', TipLink: 'tool0' }, hidden: true });
  const start = motusComponent('joints', 660, 240, {}, { jointValues: START_JOINTS });
  const goal = motusComponent('joints', 660, 380, {}, { jointValues: GOAL_JOINTS });
  const plan = motusComponent('plan', 880, 200, {
    Robot: [outRef(robot.node, 'Robot')],
    Goal: [outRef(goal.node, 'State')],
    Start: [outRef(start.node, 'State')],
  });
  const { scrub, preview } = previewWithScrub(880, 200, outRef(plan.node, 'Trajectory'));
  const exp = motusComponent('export', 880 + PLAN_PREVIEW_DX, 400, { Trajectory: [outRef(plan.node, 'Trajectory')] });
  const gPaths = nativeGroup('URDF + base', [urdfFile, basePl], GROUP_COLOUR.robot);
  const gTool = nativeGroup('Tool TCP + mesh', [
    tcpPt, { xml: ux.xml, node: ux.node }, tcpPl, meshPath, loadMesh, tool,
  ], GROUP_COLOUR.tool);
  const gPlan = nativeGroup('Plan + preview', [robot, start, goal, plan, scrub, preview, exp], GROUP_COLOUR.preview);
  const objs = [
    title, note, urdfFile, basePl,
    { xml: tcpPt.xml }, { xml: ux.xml }, { xml: tcpPl.xml },
    meshPath, loadMesh, tool, robot, start, goal, plan, scrub, preview, exp,
    gPaths, gTool, gPlan,
  ];
  objs._meta = {
    fileName: '03_urdf_tool_frames.ghx',
    description: 'Motus Robot URDF + Base override + Robotiq Tool (Load Mesh, Cap=Robotiq2F85) + Start + Preview ShowStart. Auto Plan on.',
  };
  return buildGraph(objs);
}

/** 04 — motion program: PTP + LIN + CIRC + SET gripper (was 08+11). */
function graph04() {
  // One horizontal row per move (top→bottom = program order). Short wires only.
  const title = nativeScribble(40, -60, '04  Motion program', 28);
  const note = nativePanel(420, -60, 'One row per move → Merge → Program. Scrub when Status OK.', 'Note', 320, 40);

  // Robot column (left)
  const robot = ur10eRobot(40, 40);
  const start = motusComponent('joints', 40, 160, {}, { jointValues: MOTION_START });

  // Row 1 — PTP
  const ptpGoal = motusComponent('joints', 40, 320, {}, { jointValues: GOAL_JOINTS });
  const stateOpen = motusComponent('toolState', 220, 320, {
    Tool: [outRef(robot.node, 'Robot')],
  }, { text: { Preset: 'Open' } });
  const segPtp = motusComponent('segment', 420, 300, {
    Goal: [outRef(ptpGoal.node, 'State')],
    ToolState: [outRef(stateOpen.node, 'State')],
  }, { text: { Type: 'PTP' } });

  // Row 2 — LIN
  const uz = nativeUnitZ(40, 460);
  const ptLin = nativeConstructPoint(40, 520, [0.45, 0.15, 0.45]);
  const plLin = nativePlane(200, 520, ptLin.node.outputs[0], uz.node.outputs[0]);
  const segLin = motusComponent('segment', 420, 500, {
    Goal: [outRef(plLin.node, 'Plane')],
  }, { text: { Type: 'LIN' } });

  // Row 3 — CIRC
  const ptVia = nativeConstructPoint(40, 660, [0.453, 0.152, 0.45]);
  const plVia = nativePlane(200, 660, ptVia.node.outputs[0], uz.node.outputs[0]);
  const ptGoal = nativeConstructPoint(40, 780, [0.45, 0.154, 0.45]);
  const plGoal = nativePlane(200, 780, ptGoal.node.outputs[0], uz.node.outputs[0]);
  const segCirc = motusComponent('segment', 420, 700, {
    Goal: [outRef(plGoal.node, 'Plane')],
    Via: [outRef(plVia.node, 'Plane')],
  }, { text: { Type: 'CIRC' } });

  // Row 4 — SET
  const stateClosed = motusComponent('toolState', 40, 920, {
    Tool: [outRef(robot.node, 'Robot')],
  }, { text: { Preset: 'Closed' } });
  const segSet = motusComponent('segment', 420, 900, {
    ToolState: [outRef(stateClosed.node, 'State')],
  }, { text: { Type: 'SET' }, numbers: { Duration: 0.2 } });

  // Sequence column
  const segsMerge = nativeMerge(620, 520, [
    outRef(segPtp.node, 'Segment'),
    outRef(segLin.node, 'Segment'),
    outRef(segCirc.node, 'Segment'),
    outRef(segSet.node, 'Segment'),
  ]);
  const progPlan = motusComponent('progPlan', 820, 480, {
    Robot: [outRef(robot.node, 'Robot')],
    Segments: [outRef(segsMerge.node, 'Result')],
    Start: [outRef(start.node, 'State')],
  });
  const { scrub, preview } = previewWithScrub(820, 480, outRef(progPlan.node, 'Trajectory'));
  const exp = motusComponent('export', 820 + PLAN_PREVIEW_DX, 680, { Trajectory: [outRef(progPlan.node, 'Trajectory')] });

  const gRobot = nativeGroup('Robot + start', [robot, start], GROUP_COLOUR.robot);
  const gPtp = nativeGroup('1 PTP', [ptpGoal, stateOpen, segPtp], GROUP_COLOUR.plan);
  const gLin = nativeGroup('2 LIN', [
    { xml: uz.xml, node: uz.node }, ptLin, plLin, segLin,
  ], GROUP_COLOUR.plan);
  const gCirc = nativeGroup('3 CIRC', [ptVia, plVia, ptGoal, plGoal, segCirc], GROUP_COLOUR.plan);
  const gSet = nativeGroup('4 SET', [stateClosed, segSet], GROUP_COLOUR.plan);
  const gSeq = nativeGroup('Merge → Program', [segsMerge, progPlan], GROUP_COLOUR.preview);
  const gOut = nativeGroup('Preview + export', [scrub, preview, exp], GROUP_COLOUR.preview);

  const flat = [
    title, note, robot, start,
    ptpGoal, stateOpen, segPtp,
    { xml: uz.xml }, { xml: ptLin.xml }, { xml: plLin.xml }, segLin,
    { xml: ptVia.xml }, { xml: plVia.xml }, { xml: ptGoal.xml }, { xml: plGoal.xml }, segCirc,
    stateClosed, segSet, segsMerge, progPlan, scrub, preview, exp,
    gRobot, gPtp, gLin, gCirc, gSet, gSeq, gOut,
  ];
  flat._meta = {
    fileName: '04_motion_program.ghx',
    description: 'Motion program: PTP + LIN + CIRC + SET gripper (via Merge) -> Motus Program -> Preview / Export. Auto Plan on; drag Motus Scrub or Play.',
  };
  return buildGraph(flat);
}

/** 05 — Serial Chain + Reach Samples (on-component preview; no Plan). */
function graph05() {
  // Bands: title → serial chain → reach samples. Preview is on-component.
  const title = nativeScribble(40, -60, '05  Serial + Reach', 28);
  const note = nativePanel(420, -60, 'No Plan — Serial/Reach draw in Rhino. Edit Lengths.', 'Note', 300, 40);
  const chain = motusComponent('serialChain', 40, 40, {}, {
    jointValues: [0.15, 0.35, 0.30, 0.20, 0.15, 0.10],
  });
  const reach = motusComponent('reachSamples', 280, 40, {
    Robot: [outRef(chain.node, 'Robot')],
  });
  const gChain = nativeGroup('Serial Chain', [chain], GROUP_COLOUR.robot);
  const gReach = nativeGroup('Reach Samples', [reach], GROUP_COLOUR.preview);
  const objs = [title, note, chain, reach, gChain, gReach];
  objs._meta = {
    fileName: '05_serial_reach.ghx',
    description: 'Serial Chain (link lengths) → Reach Samples (N=128). On-component preview; no Plan.',
  };
  return buildGraph(objs);
}

/**
 * 06 — Turntable (1-DOF) + arm: coupled vs decoupled via Planning Group.
 * Group locking applies on the RRT path (joint-linear ignores GroupMap), so both
 * Plans share a far keep-out sphere to force RRT.
 */
function graph06() {
  const title = nativeScribble(40, -60, '06  Turntable + Group', 28);
  const note = nativePanel(
    420,
    -60,
    'Prefab UR10e on turntable. Coupled: Group off. Decoupled: arm Group locks turntable. Scrub both.',
    'Note',
    420,
    40,
  );
  // Prefab ur10e_robotiq.urdf via thin turntable xacro (meshes stay next to bundled URDF).
  const urdfFile = nativeFilePath(
    40,
    40,
    absPath('resources/robots/ur10e_robotiq/ur10e_on_turntable.xacro'),
    '*.xacro;*.urdf|*.xacro;*.urdf|All files|*.*',
  );
  const robot = motusComponent('robot', 280, 40, {
    Path: [outRef(urdfFile.node, 'Path')],
  }, { text: { BaseLink: 'world', TipLink: 'tool0' }, hidden: true });
  const start = motusComponent('joints', 40, 180, {}, { jointValues: TURNTABLE_START });
  const goal = motusComponent('joints', 40, 300, {}, { jointValues: TURNTABLE_GOAL });

  // Far sphere: SceneHasObstacles → RRT (required for GroupMap lock). Not on the path.
  const keepCenter = nativeConstructPoint(40, 420, [2.0, 2.0, 0.4]);
  const keep = motusComponent('colSphere', 220, 420, {
    Center: [outRef(keepCenter.node, 'Point')],
  }, { text: { Name: 'keepout' }, numbers: { Radius: 0.08 } });
  const scene = motusComponent('colScene', 420, 420, {
    Objects: [outRef(keep.node, 'Object')],
  });
  const group = motusComponent('group', 620, 420, {}, {
    text: { Name: 'arm', BaseLink: 'base_link', TipLink: 'tool0' },
    textList: {
      Joints: [
        'shoulder_pan_joint',
        'shoulder_lift_joint',
        'elbow_joint',
        'wrist_1_joint',
        'wrist_2_joint',
        'wrist_3_joint',
      ],
    },
  });
  const rrt = motusComponent('rrtSettings', 800, 420, {});

  const planCoupled = motusComponent('plan', 40, 680, {
    Robot: [outRef(robot.node, 'Robot')],
    Goal: [outRef(goal.node, 'State')],
    Start: [outRef(start.node, 'State')],
    Collision: [outRef(scene.node, 'Scene')],
    RrtSettings: [outRef(rrt.node, 'Settings')],
  }, { advanced: ['Collision', 'RrtSettings'] });
  const coupled = previewWithScrub(40, 680, outRef(planCoupled.node, 'Trajectory'));

  const planDecoupled = motusComponent('plan', 40, 980, {
    Robot: [outRef(robot.node, 'Robot')],
    Goal: [outRef(goal.node, 'State')],
    Start: [outRef(start.node, 'State')],
    Collision: [outRef(scene.node, 'Scene')],
    Group: [outRef(group.node, 'Group')],
    RrtSettings: [outRef(rrt.node, 'Settings')],
  }, { advanced: ['Collision', 'Group', 'RrtSettings'] });
  const decoupled = previewWithScrub(40, 980, outRef(planDecoupled.node, 'Trajectory'));

  const gRobot = nativeGroup('Prefab UR10e + turntable', [urdfFile, robot, start, goal], GROUP_COLOUR.robot);
  const gScene = nativeGroup('Scene + arm group', [
    keepCenter, keep, scene, group, rrt,
  ], GROUP_COLOUR.collision);
  const gCoupled = nativeGroup('Coupled (no Group)', [planCoupled, coupled.scrub, coupled.preview], GROUP_COLOUR.plan);
  const gDecoupled = nativeGroup('Decoupled (arm Group)', [planDecoupled, decoupled.scrub, decoupled.preview], GROUP_COLOUR.preview);

  const objs = [
    title, note,
    urdfFile, robot, start, goal,
    { xml: keepCenter.xml }, keep, scene, group, rrt,
    planCoupled, coupled.scrub, coupled.preview,
    planDecoupled, decoupled.scrub, decoupled.preview,
    gRobot, gScene, gCoupled, gDecoupled,
  ];
  objs._meta = {
    fileName: '06_turntable_group.ghx',
    description:
      'Prefab UR10e+Robotiq on 1-DOF turntable: coupled Plan vs arm Group (locks turntable). Shared RRT scene; scrub both Previews.',
  };
  return buildGraph(objs);
}

/**
 * 07 — Author gripper → Tool Rd → Robot Tl → Program PTP (ToolMode Ramp open→closed) → Preview.
 * Cap=Robotiq2F85 supplies width schema/DefaultState(Open); Bd=j_left drives the authored joint.
 */
function graph07() {
  const title = nativeScribble(40, -60, '07  URDF gripper Tool', 28);
  const note = nativePanel(
    420,
    -60,
    'Arm = ur10e_minimal (no Robotiq). Boxes → ULink → Tool Rd → Tl. Cap=Robotiq2F85, Bd=j_left.',
    'Note',
    520,
    40,
  );

  const xy = nativeXYPlane(40, 0);
  const palmBox = nativeCenterBox(40, 40, outRef(xy.node, 'Plane'), [0.08, 0.06, 0.03]);
  const leftBox = nativeCenterBox(40, 160, outRef(xy.node, 'Plane'), [0.02, 0.01, 0.06]);
  const rightBox = nativeCenterBox(40, 280, outRef(xy.node, 'Plane'), [0.02, 0.01, 0.06]);

  const palm = motusComponent('urdfLink', 220, 40, {
    Visual: [outRef(palmBox.node, 'Box')],
  }, { text: { Name: 'palm' } });
  const left = motusComponent('urdfLink', 220, 160, {
    Visual: [outRef(leftBox.node, 'Box')],
  }, { text: { Name: 'L' } });
  const right = motusComponent('urdfLink', 220, 280, {
    Visual: [outRef(rightBox.node, 'Box')],
  }, { text: { Name: 'R' } });

  const uz = nativeUnitZ(40, 420);
  const leftOrigin = nativeConstructPoint(40, 500, [0, 0.035, 0]);
  const rightOrigin = nativeConstructPoint(40, 580, [0, -0.035, 0]);
  const leftAxis = nativeLineSdl(220, 460, leftOrigin.node.outputs[0], uz.node.outputs[0], 0.05);
  const rightAxis = nativeLineSdl(220, 580, rightOrigin.node.outputs[0], uz.node.outputs[0], 0.05);

  const jLeft = motusComponent('urdfJoint', 420, 200, {
    Axis: [outRef(leftAxis.node, 'Line')],
  }, {
    text: { Name: 'j_left', Type: 'Revolute', Parent: 'palm', Child: 'L' },
    numbers: { Lower: 0, Upper: 0.8 },
  });
  const jRight = motusComponent('urdfJoint', 420, 400, {
    Axis: [outRef(rightAxis.node, 'Line')],
  }, {
    text: {
      Name: 'j_right', Type: 'Revolute', Parent: 'palm', Child: 'R', MimicJoint: 'j_left',
    },
    numbers: { Lower: 0, Upper: 0.8, MimicMult: -1, MimicOffset: 0 },
  });

  const linksMerge = nativeMerge(420, 40, [
    outRef(palm.node, 'Link'),
    outRef(left.node, 'Link'),
    outRef(right.node, 'Link'),
  ]);
  const jointsMerge = nativeMerge(600, 280, [
    outRef(jLeft.node, 'Joint'),
    outRef(jRight.node, 'Joint'),
  ]);
  const assemble = motusComponent('urdfAssemble', 780, 120, {
    Links: [outRef(linksMerge.node, 'Result')],
    Joints: [outRef(jointsMerge.node, 'Result')],
  }, { text: { Name: 'demo_gripper', Tip: 'palm' } });

  // Cap supplies Open/Closed width schema; Bd maps width → authored driver (not robotiq_* names).
  const tool = motusComponent('tool', 980, 120, {
    Description: [outRef(assemble.node, 'Description')],
  }, { text: { Name: 'demo_gripper', Capabilities: 'Robotiq2F85', Binding: 'j_left' } });

  // Arm-only primitives — ur10e.urdf/.robotiq need mesh assets; minimal always previews.
  const urdfFile = nativeFilePath(
    780,
    320,
    absPath('examples/ur10e/ur10e_minimal.urdf'),
  );
  const robot = motusComponent('robot', 980, 320, {
    Path: [outRef(urdfFile.node, 'Path')],
    Tool: [outRef(tool.node, 'Tool')],
  }, { text: { BaseLink: 'base_link', TipLink: 'tool0' }, hidden: true });

  const start = motusComponent('joints', 1180, 200, {}, { jointValues: START_JOINTS });
  const goal = motusComponent('joints', 1180, 340, {}, { jointValues: GOAL_JOINTS });
  const stateClosed = motusComponent('toolState', 1180, 480, {
    Tool: [outRef(tool.node, 'Tool')],
  }, { text: { Preset: 'Closed' } });
  // InitialToolState = Cap DefaultState (Open); Ramp lerps width → Closed over the PTP.
  const segPtp = motusComponent('segment', 1380, 280, {
    Goal: [outRef(goal.node, 'State')],
    ToolState: [outRef(stateClosed.node, 'State')],
  }, { text: { Type: 'PTP' }, toolMode: 'Ramp' });
  const prog = motusComponent('progPlan', 1580, 240, {
    Robot: [outRef(robot.node, 'Robot')],
    Segments: [outRef(segPtp.node, 'Segment')],
    Start: [outRef(start.node, 'State')],
  });
  const { scrub, preview } = previewWithScrub(1580, 240, outRef(prog.node, 'Trajectory'));

  const exportFolder = nativePanel(
    780,
    480,
    absPath('examples'),
    'Folder',
    260,
    40,
  );
  const urdfExport = motusComponent('urdfExport', 1080, 480, {
    Description: [outRef(assemble.node, 'Description')],
    Folder: [outRef(exportFolder.node, 'Text')],
  }, { text: { Name: 'demo_gripper' } });

  const gAuthor = nativeGroup('Box / Link / Joint', [
    { xml: xy.xml, node: xy.node },
    palmBox, leftBox, rightBox, palm, left, right,
    { xml: uz.xml, node: uz.node },
    { xml: leftOrigin.xml, node: leftOrigin.node },
    { xml: rightOrigin.xml, node: rightOrigin.node },
    leftAxis, rightAxis,
    jLeft, jRight,
  ], GROUP_COLOUR.model);
  const gTool = nativeGroup('Assemble → Tool Rd', [
    linksMerge, jointsMerge, assemble, tool,
  ], GROUP_COLOUR.tool);
  const gPlan = nativeGroup('Robot Tl + Program', [
    urdfFile, robot, start, goal, stateClosed, segPtp, prog, scrub, preview, exportFolder, urdfExport,
  ], GROUP_COLOUR.preview);

  const objs = [
    title, note,
    { xml: xy.xml }, palmBox, leftBox, rightBox, palm, left, right,
    { xml: uz.xml }, { xml: leftOrigin.xml }, { xml: rightOrigin.xml }, leftAxis, rightAxis,
    jLeft, jRight, linksMerge, jointsMerge, assemble, tool,
    urdfFile, robot, start, goal, stateClosed, segPtp, prog, scrub, preview,
    exportFolder, urdfExport,
    gAuthor, gTool, gPlan,
  ];
  objs._meta = {
    fileName: '07_urdf_gripper_tool.ghx',
    description:
      'ur10e_minimal (no prefab gripper) + Center Box → Motus Urdf Link/Joint/Assemble → Tool Rd (Cap=Robotiq2F85, Bd=j_left) → Robot Tl → Program PTP Ramp → Preview. Wire any Rhino geometry into ULink V. Export URDF Write on Assemble D.',
  };
  return buildGraph(objs);
}

function graph08() {
  const title = nativeScribble(40, 40, '08 · Stewart TCP path', 28);
  const note = nativePanel(
    40,
    80,
    'Motus Stewart (classic hex) → Plan with TCP planes → Preview. Q = leg lengths (m), not UR MoveJ.',
    'Note',
    420,
    60,
  );
  const stewart = motusComponent('stewart', 40, 200, {});
  const startPt = nativeConstructPoint(280, 160, [0, 0, 0.625]);
  const goalPt = nativeConstructPoint(280, 280, [0.01, 0, 0.625]);
  const uz = nativeUnitZ(280, 100);
  const startPl = nativePlane(400, 160, startPt.node.outputs[0], uz.node.outputs[0]);
  const goalPl = nativePlane(400, 280, goalPt.node.outputs[0], uz.node.outputs[0]);
  const plan = motusComponent('plan', 560, 200, {
    Robot: [outRef(stewart.node, 'Robot')],
    Goal: [outRef(goalPl.node, 'Plane')],
    Start: [outRef(startPl.node, 'Plane')],
  });
  const { scrub, preview } = previewWithScrub(760, 200, outRef(plan.node, 'Trajectory'));
  const waypoints = motusComponent('waypoints', 980, 200, {
    Trajectory: [outRef(plan.node, 'Trajectory')],
  });

  const gModel = nativeGroup('Stewart', [stewart], GROUP_COLOUR.model);
  const gPlan = nativeGroup('Plan TCP', [
    { xml: startPt.xml, node: startPt.node },
    { xml: goalPt.xml, node: goalPt.node },
    { xml: uz.xml, node: uz.node },
    { xml: startPl.xml, node: startPl.node },
    { xml: goalPl.xml, node: goalPl.node },
    plan, scrub, preview, waypoints,
  ], GROUP_COLOUR.plan);

  const objs = [
    title, note, stewart,
    { xml: startPt.xml }, { xml: goalPt.xml }, { xml: uz.xml },
    { xml: startPl.xml }, { xml: goalPl.xml },
    plan, scrub, preview, waypoints,
    gModel, gPlan,
  ];
  objs._meta = {
    fileName: '08_stewart_tcp_path.ghx',
    description:
      'Motus Stewart classic hex → Plan Start/Goal TCP planes → Preview scrub + Waypoints (leg lengths in meters). Requires Motus.NET ≥ 0.9.0 (-UseLocal until NuGet publish).',
  };
  return buildGraph(objs);
}

/**
 * Shared Walk graph (Body+Leg+Mechanism + Ground + arc → Walk → Preview).
 * Logic asserted by Motus.NET Example09 + qa-smoke for N=6. N is the only structural knob.
 */
function graphWalking({ n, label, fileName, description }) {
  const title = nativeScribble(40, 40, label, 28);
  const note = nativePanel(
    40,
    80,
    'Slider N → Body → Leg → Mechanism → Walk Mech; Ground → Tn; Planes arc → gait Tr → Preview. Drag N (4–12) for hex/octo/… Green rings = planted feet.',
    'Note',
    560,
    88,
  );
  const uz = nativeUnitZ(40, 180);
  const nSlider = nativeNumberSlider(200, 140, { value: n, min: 4, max: 12, nick: 'N', w: 180 });
  const groundOrigin = nativeConstructPoint(200, 200, [0.22, 0, 0]);
  const ground = motusComponent('terrainPatch', 400, 200, {
    Origin: [outRef(groundOrigin.node, 'Point')],
  }, { numbers: { Amp: 0.02 } });
  const body = motusComponent('body', 560, 140, {
    N: [outRef(nSlider.node, 'Number')],
  });
  const leg = motusComponent('leg', 560, 300, {});
  const mech = motusComponent('mechanism', 700, 200, {
    Body: [outRef(body.node, 'Body')],
    Leg: [outRef(leg.node, 'Leg')],
  });
  const arcParts = [];
  const planeRefs = [];
  const ARC_N = 9;
  for (let i = 0; i < ARC_N; i++) {
    const a = Math.PI - (i / (ARC_N - 1)) * Math.PI;
    const px = 0.22 + 0.18 * Math.cos(a);
    const py = 0.18 * Math.sin(a);
    // XY path only — Walk samples terrain height under the body.
    const pt = nativeConstructPoint(40, 300 + i * 44, [px, py, 0]);
    const pl = nativePlane(140, 300 + i * 44, outRef(pt.node, 'Point'), outRef(uz.node, 'Vector'));
    arcParts.push({ xml: pt.xml, node: pt.node }, { xml: pl.xml, node: pl.node });
    planeRefs.push(outRef(pl.node, 'Plane'));
  }
  const planesMerge = nativeMerge(280, 460, planeRefs);
  const walk = motusComponent('walk', 40, 720, {
    Mechanism: [outRef(mech.node, 'Mechanism')],
    Planes: [outRef(planesMerge.node, 'Result')],
    Terrain: [outRef(ground.node, 'Mesh')],
  }, { numbers: { Lift: 0.06 } });
  const { scrub, preview } = previewWithScrub(560, 720, outRef(walk.node, 'Trajectory'));
  const gModel = nativeGroup('Walking path', [
    { xml: uz.xml, node: uz.node },
    nSlider,
    groundOrigin,
    ground,
    body,
    leg,
    mech,
    ...arcParts,
    planesMerge,
    walk,
  ], GROUP_COLOUR.model);
  const gPreview = nativeGroup('Gait Preview', [scrub, preview], GROUP_COLOUR.preview);

  const objs = [
    title, note, { xml: uz.xml },
    nSlider, groundOrigin, ground, body, leg, mech,
    ...arcParts, planesMerge, walk, scrub, preview, gModel, gPreview,
  ];
  objs._meta = { fileName, description };
  return buildGraph(objs);
}

function graph09() {
  return graphWalking({
    n: 6,
    label: '09 · Walking hexapod',
    fileName: '09_walking_hexapod.ghx',
    description:
      'Body N slider (default 6, range 4–12) + Leg + Mechanism + Ground + arc → Walk → Preview. Drag N for other leg counts.',
  });
}

const graphs = [graph01, graph02, graph03, graph04, graph05, graph06, graph07, graph08, graph09];
const legacy = [
  '01_basic_planning.ghx',
  '02_collision_planning.ghx',
  '01_joint_planning.ghx',
  '02_cartesian_planning.ghx',
  '03_collision_rrt.ghx',
  '04_collision_shapes.ghx',
  '05_srdf_group_attach.ghx',
  '06_urdf_load.ghx',
  '07_frames_and_start.ghx',
  '08_motion_program.ghx',
  '09_tool_tcp.ghx',
  '10_robotiq_tool.ghx',
  '11_gripper_motion_program.ghx',
  '12_sequential_goals.ghx',
  '10_funky_octopod.ghx',
];

for (const name of legacy) {
  const p = path.join(outDir, name);
  if (fs.existsSync(p)) fs.unlinkSync(p);
}

for (const buildFn of graphs) {
  const xml = buildFn();
  const meta = lastGraphMeta;
  if (!meta?.fileName) throw new Error(`missing meta for ${buildFn.name}`);
  const outPath = path.join(outDir, meta.fileName);
  fs.writeFileSync(outPath, xml, 'utf8');
  console.log('wrote', meta.fileName);
}

console.log('Done.');
