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
const PLUGIN_VERSION = csproj.match(/<Version>([^<]+)<\/Version>/)?.[1]?.trim() ?? MOTUS_NET_VERSION;
/** Prefer csproj AssemblyVersion (4-part) so .ghx meta matches the built .gha. */
const PLUGIN_ASSEMBLY_VERSION =
  csproj.match(/<AssemblyVersion>([^<]+)<\/AssemblyVersion>/)?.[1]?.trim() ??
  ( /^\d+\.\d+\.\d+$/.test(PLUGIN_VERSION) ? `${PLUGIN_VERSION}.0` : PLUGIN_VERSION);
/** Portable path written into .ghx (POSIX). Prefer repo-relative resources/… (bundled)
 * or examples/-sibling paths (resolved beside the .ghx via UrdfPathResolver). */
const repoRel = (...parts) => parts.join('/');
/** Absolute only for local script I/O (never written into example graphs). */
const absPath = (...parts) => path.resolve(repoRoot, ...parts);

/** Panel that feeds Motus Path/Srdf pins — avoids GH File Path baking machine absolutes. */
function pathPanel(x, y, relPath, nick = 'Path', w = 200, h = 36) {
  return nativePanel(x, y, relPath, nick, w, h);
}

const GOAL_JOINTS = [1.2, -1, 1.2, -1.6, -1.5708, 0];
const START_JOINTS = [0, -1.2, 1.2, -1.6, -1.5708, 0];
const MOTION_START = [0, -0.5, 1.0, -1.0, 0.0, 0.0];
/** UR10e+Robotiq fingers-down home for example 10 (rad). Rhino TCP Z = world +Z → Motus X = +Z → fingers world −Z. */
const PICK_PLACE_HOME = [0, -77 * Math.PI / 180, 0, -31 * Math.PI / 180, -77 * Math.PI / 180, 0];
/** Table top under bricks = 0.48 m; ColBox HalfZ=0.02 → center Z = 0.46. */
const PICK_PLACE_TABLE = [0.70, -0.20, 0.46];
/** Tower footprint center (bottom brick Z = 0.48 + HalfZ). */
const PICK_PLACE_TOWER = [0.70, 0.0, 0.48];
/** First place-column XY (same Z base as tower). */
const PICK_PLACE_COLUMNS = [0.70, -0.25, 0.48];
/** Walking hex right-middle tip leg: hip, femur, tibia (rad) at default stance. */
const HEX_TIP_START = [-0.1309, 0.5236, -0.5236];
const HEX_TIP_GOAL = [-0.1309, 0.6109, -0.5236];
/** Collision-free home-ish start for obstacle demos (away from table/box). */
const COLLISION_START = [0.0, -1.4, 1.4, -1.7, -1.5708, 0.0];
const COLLISION_GOAL = [1.0, -0.9, 1.0, -1.4, -1.5708, 0.3];
/** ur10e_with_turntable.xacro AllDrivers: […UR×6, turntable_yaw].
 * Robotiq TCP (not tool0) tracks orange spoke corner; flange Motus Z = −world Z (gripper hangs vertical).
 * Waypoints every π/8 so joint-linear stays on the cube (~5 mm). */
const TT_WAYPOINTS = [
  [-0.1539, -1.0030, 1.4702, -2.0380, -1.5708, -1.7247, 0.0],
  [-0.0487, -1.0357, 1.5236, -2.0587, -1.5708, -1.6195, 0.3927],
  [0.0431, -1.1099, 1.6412, -2.1022, -1.5708, -1.5277, 0.7854],
  [0.1125, -1.2196, 1.8057, -2.1569, -1.5708, -1.4583, 1.1781],
  [0.1456, -1.3596, 1.9966, -2.2078, -1.5708, -1.4252, 1.5708],
];
const TT_START = TT_WAYPOINTS[0];
const TT_GOAL = TT_WAYPOINTS[TT_WAYPOINTS.length - 1];

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
  robot: { guid: 'aa3e8488-943e-426f-b205-e8db5f684998', name: 'Motus Robot', nick: 'Robot', w: 74, h: 164,
    inputs: [
      { name: 'Path', nick: 'P', desc: 'Path to .urdf or .xacro file', optional: false, text: '' },
      { name: 'BaseLink', nick: 'B', desc: 'Base link name', optional: true, text: 'base_link' },
      { name: 'TipLink', nick: 'Tip', desc: 'Tip link name', optional: true, text: 'tool0' },
      { name: 'Base', nick: 'Bf', desc: 'Optional base frame override (TCP goals are in this frame)', optional: true, plane: true },
      { name: 'Tool', nick: 'Tl', desc: 'Optional Motus Tool definition', optional: true },
      { name: 'AllDrivers', nick: 'All', desc: 'Plan tip path + side-branch drivers (e.g. DKP)', optional: true, bool: false },
      { name: 'Attach', nick: 'At', desc: 'Optional fixture geometry grafted at AttachLink', optional: true },
      { name: 'AttachLink', nick: 'Al', desc: 'Parent link for Attach', optional: true, text: '' },
      // No default point — persisting [0,0,0] as three numbers list-explodes Robot ×3.
      { name: 'AttachOrigin', nick: 'Ao', desc: 'Attach origin in AttachLink frame (m)', optional: true },
    ],
    outputs: [{ name: 'Robot', nick: 'Rb', desc: 'Robot model with URDF kinematics chain' }] },
  ur10e: { guid: '84b06a7d-8a3d-46ec-968f-25e74c249ad1', name: 'Motus UR10e Robotiq', nick: 'UR10e', w: 74, h: 44,
    inputs: [],
    outputs: [{ name: 'Robot', nick: 'Rb', desc: 'Robot model with URDF kinematics chain' }] },
  // w≥96: DropDownAttributes Cap strip; h gets +DROPDOWN_EXTRA_H in motusComponent.
  tool: { guid: 'b7c4e2a1-9f3d-4b6e-8c1d-2a5f9e0b3d71', name: 'Motus Tool', nick: 'Tool', w: 96, h: 148,
    desc: 'TCP + Cap schema (None|Robotiq2F85|Custom) + optional G/L or Rd+Bd. Cap ≠ ToolMode.',
    inputs: [
      { name: 'Name', nick: 'N', desc: 'Tool name', optional: false, text: 'gripper' },
      { name: 'TCP', nick: 'P', desc: 'TCP in flange frame (Z = tool axis); unwired + Description → TipTcp', optional: true, plane: true },
      { name: 'Geometry', nick: 'G', desc: 'Optional static gripper mesh (legacy Cap+STL); ignored when Description wired', optional: true },
      { name: 'GeomPlane', nick: 'L', desc: 'Geometry pose in TCP-local frame', optional: true, plane: true },
      { name: 'Description', nick: 'Rd', desc: 'Optional actuated mechanism (Motus Urdf Assemble) grafted on Motus Robot Tl', optional: true },
      { name: 'Binding', nick: 'Bd', desc: 'Driver joint for Cap width when Rd wired (required for Cap=Custom)', optional: true, text: '' },
      { name: 'WidthMin', nick: 'Wmin', desc: 'Cap=Custom jaw width min (m)', optional: true, number: 0 },
      { name: 'WidthMax', nick: 'Wmax', desc: 'Cap=Custom jaw width max / open (m)', optional: true, number: 0.085 },
      { name: 'ClosedDriver', nick: 'Cd', desc: 'Cap=Custom closed driver value at Wmin', optional: true, number: 0.8 },
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
  plan: { guid: '8bb0bae3-527f-4e80-a8a4-c8a88b7276de', name: 'Motus Plan', nick: 'Quick', w: 96, h: 104,
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
  // w≥96 for Play button min width; live h grows by BUTTON_EXTRA_H.
  preview: { guid: 'd4a8f1c2-3e5b-4a7d-9c1e-8f2b6d4e0a91', name: 'Motus Preview', nick: 'Preview', w: 96, h: 84,
    inputs: [
      { name: 'Trajectory', nick: 'Tr', desc: 'Motus trajectory from Motus Plan (list concatenates sequential goals)', optional: false, access: 1, typeId: PTYPE.trajectory },
      { name: 'ShowStart', nick: 'SS', desc: 'Also preview the trajectory start pose as a ghost', optional: false, bool: true, typeId: PTYPE.boolean },
      { name: 'Position', nick: 'P', desc: 'Optional normalized playback position 0–1 (Motus Scrub)', optional: true, typeId: PTYPE.number },
      { name: 'Scene', nick: 'Sc', desc: 'Optional collision scene for attach-aware obstacle preview', optional: true, typeId: PTYPE.colScene },
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
  colBoxes: { guid: 'a4b5c6d7-e8f9-4012-b345-6789abcdef01', name: 'Motus Collision Boxes', nick: 'Boxes', w: 74, h: 104,
    inputs: [
      { name: 'Planes', nick: 'P', desc: 'Box centers; plane XYZ = box XYZ', optional: false, access: 1 },
      { name: 'HalfX', nick: 'X', desc: 'Half extent X (m)', optional: false, number: 0.04 },
      { name: 'HalfY', nick: 'Y', desc: 'Half extent Y (m)', optional: false, number: 0.02 },
      { name: 'HalfZ', nick: 'Z', desc: 'Half extent Z (m)', optional: false, number: 0.01 },
      { name: 'Prefix', nick: 'N', desc: 'Name prefix → N00, N01, …', optional: false, text: 'b' },
    ],
    outputs: [{ name: 'Objects', nick: 'O', desc: 'Collision objects', access: 1 }] },
  pickPlace: { guid: 'b5c6d7e8-f9a0-4123-c456-789abcdef012', name: 'Motus Pick Place', nick: 'PickPlace', w: 74, h: 164,
    inputs: [
      { name: 'Grasp', nick: 'G', desc: 'Grasp TCP planes (visit order)', optional: false, access: 1 },
      { name: 'Place', nick: 'Pl', desc: 'Place TCP planes', optional: false, access: 1 },
      { name: 'Objects', nick: 'O', desc: 'Collision objects to attach', optional: false, access: 1 },
      { name: 'Approach', nick: 'Az', desc: 'Hover height world +Z (m)', optional: false, number: 0.08 },
      { name: 'Open', nick: 'Wopen', desc: 'Open jaw width (m)', optional: false, number: 0.085 },
      { name: 'Close', nick: 'Wclose', desc: 'Close jaw width (m)', optional: false, number: 0.04 },
      { name: 'Step', nick: 'St', desc: 'LIN step (m)', optional: true, number: 0.005 },
      { name: 'Sampling Transfers', nick: 'RRT', desc: 'RRT between hover poses', optional: true, bool: false },
      { name: 'Touch Bodies', nick: 'Touch', desc: 'Gripper collision body names (required)', optional: false, access: 1, text: 'robotiq_2f85' },
    ],
    outputs: [{ name: 'Segments', nick: 'Seg', desc: 'Motion segments for Motus Program', access: 1 }] },
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
      { name: 'GraspTcp', nick: 'G', desc: 'Optional TCP plane at grasp — auto TcpLocal from Object pose', optional: true, plane: true },
      { name: 'TcpLocal', nick: 'P', desc: 'Manual TCP-local pose when GraspTcp unwired', optional: true, plane: true },
      { name: 'SourceName', nick: 'Src', desc: 'Optional scene object name to hide while attached', optional: true, text: '' },
    ],
    outputs: [{ name: 'Attach', nick: 'A', desc: 'Attached body' }] },
  toolState: { guid: 'a1b2c3d4-e5f6-7890-abcd-ef1234567890', name: 'Motus Tool State', nick: 'ToolState', w: 96, h: 84,
    desc: 'End-effector state from Cap schema; Preset on-component; Width used when Custom',
    inputs: [
      { name: 'Tool', nick: 'Tl', desc: 'Motus Tool or Robot (uses Robot.Tool / bundled Cap schema)', optional: true },
      { name: 'Width', nick: 'W', desc: 'Jaw width (m) when Preset=Custom', optional: true, number: 0.085 },
      { name: 'Speed', nick: 'Sp', desc: 'Grip speed ratio 0–1 (export hint)', optional: true, number: 0.5 },
      { name: 'Force', nick: 'F', desc: 'Grip force ratio 0–1 (export hint)', optional: true, number: 0.5 },
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
  progPlan: { guid: '8d5f0b3e-2c4e-4f9b-0a7d-3e9c6b8f0d42', name: 'Motus Program', nick: 'Program', w: 96, h: 144,
    desc: 'Plan Motus Move sequence (Auto Plan or click Plan); LIN failures do not fall back to joint paths',
    inputs: [
      { name: 'Robot', nick: 'Rb', desc: 'Robot model', optional: false },
      { name: 'Segments', nick: 'Seg', desc: 'List of Motus Move segments (wire order = program order)', optional: false, access: 1 },
      { name: 'Start', nick: 'St0', desc: 'Start joint state (defaults to home)', optional: true },
      { name: 'Collision', nick: 'C', desc: 'Collision scene', optional: true },
      { name: 'Group', nick: 'Gr', desc: 'Optional planning group (locks non-group joints)', optional: true },
      { name: 'Attach', nick: 'A', desc: 'Optional attached bodies list', optional: true, access: 1 },
      { name: 'Prior', nick: 'Tr0', desc: 'Optional prior trajectory — last point supplies start joints and initial tool state when St0 is unwired', optional: true, typeId: PTYPE.trajectory },
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
    guid: 'd9e3b2c1-5f4a-4b8d-9e2f-3c7a1d0b6f82', name: 'Motus Joint Table', nick: 'JointTbl', w: 74, h: 180,
    desc: 'Joint table → tree. Plan tip path by default; AllDrivers promotes side branches.',
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
      { name: 'Home', nick: 'Q', desc: 'Optional home q along Plan DOF order', optional: true, list: true, access: 1 },
      { name: 'BaseSE2', nick: 'SE2', desc: 'Optional holonomic base goal X, Y, Yaw(rad) — also used as preview base frame', optional: true, list: true, access: 1 },
      { name: 'AllDrivers', nick: 'All', desc: 'Plan tip path + side-branch drivers', optional: true, bool: false },
    ],
    outputs: [{ name: 'Robot', nick: 'Rb', desc: 'Robot model (same as Motus Robot)' }],
  },
  stewart: {
    guid: 'a9e1c3f0-7b2d-4e8a-9c1f-6d4b2a0e8f73', name: 'Motus Stewart', nick: 'Stewart', w: 80, h: 160,
    desc: 'Stewart/Gough hexapod (Family=stewart; Q = leg lengths in meters)',
    inputs: [
      { name: 'Path', nick: 'P', desc: 'Optional Stewart JSON (schemaVersion=1)', optional: true, text: '' },
      { name: 'Base', nick: 'Base', desc: 'Optional 6 base anchor points (m)', optional: true, access: 1 },
      { name: 'Plat', nick: 'Plat', desc: 'Optional 6 platform anchor points (m)', optional: true, access: 1 },
      { name: 'BaseRadius', nick: 'Br', desc: 'Classic base radius (m) when Base/Plat unwired', optional: true, number: 0.5 },
      { name: 'PlatformRadius', nick: 'Pr', desc: 'Classic platform radius (m) when Base/Plat unwired', optional: true, number: 0.3 },
      { name: 'MinStroke', nick: 'Lmin', desc: 'Min leg length (m)', optional: true, number: 0.35 },
      { name: 'MaxStroke', nick: 'Lmax', desc: 'Max leg length (m)', optional: true, number: 0.90 },
      { name: 'Name', nick: 'N', desc: 'Model name', optional: true, text: 'stewart_classic' },
      { name: 'PairSep', nick: 'Sep', desc: 'Classic pair angular separation (rad)', optional: true, number: 0.15 },
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
  unitY: { guid: 'd3d195ea-2d59-4ffa-90b1-8b7ff3369f69', name: 'Unit Y', nick: 'Y', w: 44, h: 22, outputs: ['Vector'] },
  deconstructPlane: { guid: '3cd2949b-4ea8-4ffb-a70c-5c380f9f46ea', name: 'Deconstruct Plane', nick: 'DePlane', w: 65, h: 84 },
  constructPlane: { guid: 'bc3e379e-7206-4e7b-b63a-ff61f4b38a3e', name: 'Construct Plane', nick: 'Pl', w: 65, h: 64 },
  vectorAmplitude: { guid: '6ec39468-dae7-4ffa-a766-f2ab22a2c62e', name: 'Amplitude', nick: 'Amp', w: 65, h: 44 },
  moveTranslate: { guid: 'b40f28a2-ba30-4ac2-afe5-a6ece7f985fc', name: 'Move', nick: 'Move', w: 44, h: 44 },
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
GROUP_COLOUR.env = GROUP_COLOUR.collision;  // Obstacles / env / fixture
GROUP_COLOUR.play = GROUP_COLOUR.preview;   // Scrub + Preview + handoff

/**
 * Shared L→R pipeline for every example:
 *   Robot → Env + Traj → Plan → Play
 * Coordinates are authored Bounds (live chrome can grow — Cassis verifies after load).
 * Without opening Grasshopper: assertAuthoredOverlaps + writeLayoutSvg.
 */
const PIPE = {
  x0: 40,
  y0: 80,              // stage header Y (content below title / note)
  /** Title/note near canvas X origin (Cassis-measured on 09). */
  titleX: -1.66,
  titleY: -68.58,
  noteX: 0.82,
  noteY: -30.84,
  titleSize: 28,
  noteSize: 14,
  groupHeaderSize: 25,
  headerGap: 40,       // scribble above first component in a stage
  colGap: 120,         // horizontal gap between stage columns
  rowGap: 28,
  /** Play stage scribble X offset from Plan origin (clears Plan header width). */
  playHeaderDx: 200,
};
function id() {
  return crypto.randomUUID();
}

function esc(s) {
  return String(s).replace(/&/g, '&amp;').replace(/</g, '&lt;');
}

function item(name, type, code, value) {
  return `            <item name="${name}" type_name="${type}" type_code="${code}">${value}</item>`;
}

/** Viewport-hide native GH geometry (IGH_PreviewObject.Hidden) so construction planes/points don't clutter Rhino. */
function hidePreview(part) {
  if (!part?.xml || part.xml.includes('name="Hidden"')) return part;
  const xml = part.xml
    .replace(
      /(<chunk name="Container">\s*<items count=")(\d+)(">)/,
      (_, a, n, c) => `${a}${Number(n) + 1}${c}`,
    )
    .replace(
      /(<item name="Description"[^>]*>[^<]*<\/item>\n)/,
      `$1                    ${item('Hidden', 'gh_bool', '1', 'true')}\n`,
    );
  return { ...part, xml };
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

/** One GH_Point (not three numbers — that list-explodes Param_Point). */
function persistentPoint(xyz) {
  const [x, y, z] = xyz;
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
                                    <item name="point" type_name="gh_point" type_code="14">
                                      <X>${x}</X>
                                      <Y>${y}</Y>
                                      <Z>${z}</Z>
                                    </item>
                                  </items>
                                </chunk>
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
    else if (inp.point && !sources.length) persistent = persistentPoint(inp.point);
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
  // Adjust height by pin count (dropdown/Play chrome is live-only — budget via belowY / PLAN_*).
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
  const toolCap = (options.toolCapabilities || options.text?.Capabilities || 'None').toString().trim();
  const toolFlags = key === 'tool' ? [
    item('ToolCapabilities', 'gh_string', '10', esc(toolCap === '' ? 'None' : toolCap)),
    item('CanvasPivotX', 'gh_double', '6', String(x + spec.w / 2)),
    item('CanvasPivotY', 'gh_double', '6', String(y + spec.h / 2)),
  ] : [];
  const toolStatePreset = (options.toolStatePreset || options.text?.Preset || 'Open').toString().trim();
  const toolStateFlags = key === 'toolState' ? [
    item('ToolStatePreset', 'gh_string', '10', esc(toolStatePreset)),
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
    ...toolFlags,
    ...toolStateFlags,
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
      else if (inp.point && !sources.length) persistent = persistentPoint(inp.point);
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
 * Native GH Number Slider. Wire outRef(node, 'Number').
 * Interval: 0=Float, 1=Integer, 2=Odd, 3=Even (GH_NumberSlider.Write).
 * Default integer (Body N). Pass digits>0 / interval:0 for floats (Stewart Br/Pr/…).
 */
function nativeNumberSlider(x, y, {
  value = 6, min = 4, max = 12, nick = 'N', w = NATIVE.numberSlider.w,
  digits = 0, interval = 1,
} = {}) {
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
                        ${item('Digits', 'gh_int32', '3', String(digits))}
                        ${item('GripDisplay', 'gh_int32', '3', '1')}
                        ${item('Interval', 'gh_int32', '3', String(interval))}
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
 * Plan → Scrub → Preview spacing (Cassis-measured live Bounds, 0.13.2):
 * Button/dropdown chrome widens Plan/Preview (~±11) and adds ~28–36px height.
 * Scrub must clear Preview's live left edge: scrubRight + gap < previewLeft.
 *   Plan (px,py) → Scrub (+120,+88) → Preview (+420,+9)
 */
const BUTTON_EXTRA_H = 28;
const DROPDOWN_EXTRA_H = 36;
const PLAN_SCRUB_DX = 120;
const PLAN_SCRUB_DY = 88;
const PLAN_SCRUB_W = 200;
const PLAN_PREVIEW_DX = 420;
const PLAN_PREVIEW_DY = 9;
const STACK_GAP = 24;
/** Authored Preview pin height + Play button — use for stacking Waypoints/Export. */
const PREVIEW_LAYOUT_H = 84 + BUTTON_EXTRA_H;

/** Y just below an authored component after live Play/Replan button chrome. */
function belowY(y, authoredH, gap = STACK_GAP) {
  return y + authoredH + BUTTON_EXTRA_H + gap;
}

/** Y below Preview in a Plan→Scrub→Preview cluster. */
function belowPreview(planY, gap = STACK_GAP) {
  return planY + PLAN_PREVIEW_DY + PREVIEW_LAYOUT_H + gap;
}

function previewWithScrub(planX, planY, trajectoryRef, options = {}) {
  const scrubW = options.scrubWidth ?? PLAN_SCRUB_W;
  const scrub = motusScrub(planX + PLAN_SCRUB_DX, planY + PLAN_SCRUB_DY, options.scrubValue ?? 0, scrubW);
  const trajRefs = Array.isArray(trajectoryRef) ? trajectoryRef : [trajectoryRef];
  let merge = null;
  let trajWire;
  if (trajRefs.length > 1) {
    merge = nativeMerge(planX + PLAN_SCRUB_DX, planY + PLAN_SCRUB_DY + 36, trajRefs);
    trajWire = outRef(merge.node, 'Result');
  } else {
    trajWire = trajRefs[0];
  }
  const previewInputs = {
    Trajectory: [trajWire],
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
  return { scrub, preview, merge };
}

function nativePanel(x, y, text, nick = '', w = NATIVE.panel.w, h = NATIVE.panel.h, colourArgb = '255;255;250;90') {
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
                        ${item('Colour', 'gh_drawing_color', '36', `\n                          <ARGB>${colourArgb}</ARGB>\n                        `)}
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

/**
 * Title + note scribbles — left edge at canvas X≈0 (Cassis-measured).
 * Note is a smaller scribble (not a Panel).
 */
function exampleHeader(titleText, noteText) {
  const title = nativeScribble(PIPE.titleX, PIPE.titleY, titleText, PIPE.titleSize);
  const note = nativeScribble(PIPE.noteX, PIPE.noteY, noteText, PIPE.noteSize);
  return { title, note };
}

/**
 * Coloured GH Group with a size-25 Scribble header, left-aligned at the top of the stage.
 * NickName left blank (no ZWSP/space — GH still draws empty nick chrome which looks silly).
 * Place `hx, hy` at the top-left of the stage; components should sit below hy + headerGap.
 */
function stageGroup(label, members, colourArgb, hx, hy) {
  const header = nativeScribble(hx, hy, label, PIPE.groupHeaderSize);
  const group = nativeGroup('', [header, ...members], colourArgb);
  return { header, group };
}

/** Y of first component under a stage scribble at hy. */
function stageContentY(hy) {
  return hy + PIPE.headerGap;
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

function nativeUnitY(x, y) {
  const instance = id();
  const outGuid = id();
  const node = { key: 'unitY', instance, outputs: [{ name: 'Vector', _guid: outGuid }] };
  return { xml: `<chunk name="Object" index="PLACEHOLDER">
              <items count="2">
                ${item('GUID', 'gh_guid', '9', NATIVE.unitY.guid)}
                ${item('Name', 'gh_string', '10', 'Unit Y')}
              </items>
              <chunks count="1">
                <chunk name="Container">
                  <items count="5">
                    ${item('Description', 'gh_string', '10', 'Unit vector along the Y-axis')}
                    ${item('InstanceGuid', 'gh_guid', '9', instance)}
                    ${item('Name', 'gh_string', '10', 'Unit Y')}
                    ${item('NickName', 'gh_string', '10', 'Y')}
                    ${item('SourceCount', 'gh_int32', '3', '0')}
                  </items>
                  <chunks count="2">
                    ${bounds(x, y, 44, 22)}
                    <chunk name="param_output" index="0">
                      <items count="6">
                        ${item('Description', 'gh_string', '10', 'Unit Y vector')}
                        ${item('InstanceGuid', 'gh_guid', '9', outGuid)}
                        ${item('Name', 'gh_string', '10', 'Vector')}
                        ${item('NickName', 'gh_string', '10', 'V')}
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

function nativeVectorAmplitude(x, y, vectorRef, amplitude) {
  const instance = id();
  const outGuid = id();
  const inVec = id();
  const inAmp = id();
  const node = { key: 'vectorAmplitude', instance, outputs: [{ name: 'Vector', _guid: outGuid }] };
  return { xml: `<chunk name="Object" index="PLACEHOLDER">
              <items count="2">
                ${item('GUID', 'gh_guid', '9', NATIVE.vectorAmplitude.guid)}
                ${item('Name', 'gh_string', '10', 'Amplitude')}
              </items>
              <chunks count="1">
                <chunk name="Container">
                  <items count="4">
                    ${item('Description', 'gh_string', '10', 'Set the amplitude (length) of a vector.')}
                    ${item('InstanceGuid', 'gh_guid', '9', instance)}
                    ${item('Name', 'gh_string', '10', 'Amplitude')}
                    ${item('NickName', 'gh_string', '10', 'Amp')}
                  </items>
                  <chunks count="4">
                    ${bounds(x, y, 65, 44)}
                    <chunk name="param_input" index="0">
                      <items count="7">
                        ${item('Description', 'gh_string', '10', 'Base vector')}
                        ${item('InstanceGuid', 'gh_guid', '9', inVec)}
                        ${item('Name', 'gh_string', '10', 'Vector')}
                        ${item('NickName', 'gh_string', '10', 'V')}
                        ${item('Optional', 'gh_bool', '1', 'false')}
                        ${sourceItem(0, vectorRef._guid)}
                        ${item('SourceCount', 'gh_int32', '3', '1')}
                      </items>
                      <chunks count="1">${bounds(x + 2, y + 2, 14, 20)}</chunks>
                    </chunk>
                    <chunk name="param_input" index="1">
                      <items count="6">
                        ${item('Description', 'gh_string', '10', 'Amplitude (length) value')}
                        ${item('InstanceGuid', 'gh_guid', '9', inAmp)}
                        ${item('Name', 'gh_string', '10', 'Amplitude')}
                        ${item('NickName', 'gh_string', '10', 'A')}
                        ${item('Optional', 'gh_bool', '1', 'false')}
                        ${item('SourceCount', 'gh_int32', '3', '0')}
                      </items>
                      <chunks count="2">
                        ${bounds(x + 2, y + 22, 14, 20)}
                        ${persistentNumbers([amplitude])}
                      </chunks>
                    </chunk>
                    <chunk name="param_output" index="0">
                      <items count="6">
                        ${item('Description', 'gh_string', '10', 'Resulting vector')}
                        ${item('InstanceGuid', 'gh_guid', '9', outGuid)}
                        ${item('Name', 'gh_string', '10', 'Vector')}
                        ${item('NickName', 'gh_string', '10', 'V')}
                        ${item('Optional', 'gh_bool', '1', 'false')}
                        ${item('SourceCount', 'gh_int32', '3', '0')}
                      </items>
                      <chunks count="1">${bounds(x + 48, y + 14, 14, 14)}</chunks>
                    </chunk>
                  </chunks>
                </chunk>
              </chunks>
            </chunk>`, node };
}

function nativeDeconstructPlane(x, y, planeRef) {
  const instance = id();
  const outs = ['Origin', 'X-Axis', 'Y-Axis', 'Z-Axis'].map((name) => ({ name, _guid: id() }));
  const inPlane = id();
  const node = { key: 'deconstructPlane', instance, outputs: outs };
  const outChunks = outs.map((out, i) => `<chunk name="param_output" index="${i}">
                      <items count="6">
                        ${item('Description', 'gh_string', '10', out.name)}
                        ${item('InstanceGuid', 'gh_guid', '9', out._guid)}
                        ${item('Name', 'gh_string', '10', out.name === 'Origin' ? 'Origin' : out.name)}
                        ${item('NickName', 'gh_string', '10', out.name === 'Origin' ? 'O' : out.name.charAt(0))}
                        ${item('Optional', 'gh_bool', '1', 'false')}
                        ${item('SourceCount', 'gh_int32', '3', '0')}
                      </items>
                      <chunks count="1">${bounds(x + 48, y + 2 + i * 18, 14, 14)}</chunks>
                    </chunk>`).join('\n                    ');
  return { xml: `<chunk name="Object" index="PLACEHOLDER">
              <items count="2">
                ${item('GUID', 'gh_guid', '9', NATIVE.deconstructPlane.guid)}
                ${item('Name', 'gh_string', '10', 'Deconstruct Plane')}
              </items>
              <chunks count="1">
                <chunk name="Container">
                  <items count="4">
                    ${item('Description', 'gh_string', '10', 'Deconstruct a plane into its component parts.')}
                    ${item('InstanceGuid', 'gh_guid', '9', instance)}
                    ${item('Name', 'gh_string', '10', 'Deconstruct Plane')}
                    ${item('NickName', 'gh_string', '10', 'DePlane')}
                  </items>
                  <chunks count="6">
                    ${bounds(x, y, 65, 84)}
                    <chunk name="param_input" index="0">
                      <items count="7">
                        ${item('Description', 'gh_string', '10', 'Plane to deconstruct')}
                        ${item('InstanceGuid', 'gh_guid', '9', inPlane)}
                        ${item('Name', 'gh_string', '10', 'Plane')}
                        ${item('NickName', 'gh_string', '10', 'P')}
                        ${item('Optional', 'gh_bool', '1', 'false')}
                        ${sourceItem(0, planeRef._guid)}
                        ${item('SourceCount', 'gh_int32', '3', '1')}
                      </items>
                      <chunks count="1">${bounds(x + 2, y + 2, 14, 20)}</chunks>
                    </chunk>
                    ${outChunks}
                  </chunks>
                </chunk>
              </chunks>
            </chunk>`, node };
}

function nativeConstructPlaneAxes(x, y, originRef, xAxisRef, yAxisRef) {
  const instance = id();
  const outGuid = id();
  const inO = id();
  const inX = id();
  const inY = id();
  const node = { key: 'constructPlaneAxes', instance, outputs: [{ name: 'Plane', _guid: outGuid }] };
  return { xml: `<chunk name="Object" index="PLACEHOLDER">
              <items count="2">
                ${item('GUID', 'gh_guid', '9', NATIVE.constructPlane.guid)}
                ${item('Name', 'gh_string', '10', 'Construct Plane')}
              </items>
              <chunks count="1">
                <chunk name="Container">
                  <items count="4">
                    ${item('Description', 'gh_string', '10', 'Construct a plane from an origin point and {x}, {y} axes.')}
                    ${item('InstanceGuid', 'gh_guid', '9', instance)}
                    ${item('Name', 'gh_string', '10', 'Construct Plane')}
                    ${item('NickName', 'gh_string', '10', 'Pl')}
                  </items>
                  <chunks count="5">
                    ${bounds(x, y, 65, 64)}
                    <chunk name="param_input" index="0">
                      <items count="7">
                        ${item('Description', 'gh_string', '10', 'Origin of plane')}
                        ${item('InstanceGuid', 'gh_guid', '9', inO)}
                        ${item('Name', 'gh_string', '10', 'Origin')}
                        ${item('NickName', 'gh_string', '10', 'O')}
                        ${item('Optional', 'gh_bool', '1', 'false')}
                        ${sourceItem(0, originRef._guid)}
                        ${item('SourceCount', 'gh_int32', '3', '1')}
                      </items>
                      <chunks count="1">${bounds(x + 2, y + 2, 14, 14)}</chunks>
                    </chunk>
                    <chunk name="param_input" index="1">
                      <items count="7">
                        ${item('Description', 'gh_string', '10', 'X-Axis direction of plane')}
                        ${item('InstanceGuid', 'gh_guid', '9', inX)}
                        ${item('Name', 'gh_string', '10', 'X-Axis')}
                        ${item('NickName', 'gh_string', '10', 'X')}
                        ${item('Optional', 'gh_bool', '1', 'false')}
                        ${sourceItem(0, xAxisRef._guid)}
                        ${item('SourceCount', 'gh_int32', '3', '1')}
                      </items>
                      <chunks count="1">${bounds(x + 2, y + 18, 14, 14)}</chunks>
                    </chunk>
                    <chunk name="param_input" index="2">
                      <items count="7">
                        ${item('Description', 'gh_string', '10', 'Y-Axis direction of plane')}
                        ${item('InstanceGuid', 'gh_guid', '9', inY)}
                        ${item('Name', 'gh_string', '10', 'Y-Axis')}
                        ${item('NickName', 'gh_string', '10', 'Y')}
                        ${item('Optional', 'gh_bool', '1', 'false')}
                        ${sourceItem(0, yAxisRef._guid)}
                        ${item('SourceCount', 'gh_int32', '3', '1')}
                      </items>
                      <chunks count="1">${bounds(x + 2, y + 34, 14, 14)}</chunks>
                    </chunk>
                    <chunk name="param_output" index="0">
                      <items count="6">
                        ${item('Description', 'gh_string', '10', 'Constructed plane')}
                        ${item('InstanceGuid', 'gh_guid', '9', outGuid)}
                        ${item('Name', 'gh_string', '10', 'Plane')}
                        ${item('NickName', 'gh_string', '10', 'Pl')}
                        ${item('Optional', 'gh_bool', '1', 'false')}
                        ${item('SourceCount', 'gh_int32', '3', '0')}
                      </items>
                      <chunks count="1">${bounds(x + 48, y + 24, 14, 14)}</chunks>
                    </chunk>
                  </chunks>
                </chunk>
              </chunks>
            </chunk>`, node };
}

function nativeMoveTranslate(x, y, geometryRef, translationRef) {
  const instance = id();
  const outGuid = id();
  const inG = id();
  const inT = id();
  const node = { key: 'moveTranslate', instance, outputs: [{ name: 'Geometry', _guid: outGuid }] };
  return { xml: `<chunk name="Object" index="PLACEHOLDER">
              <items count="2">
                ${item('GUID', 'gh_guid', '9', NATIVE.moveTranslate.guid)}
                ${item('Name', 'gh_string', '10', 'Move')}
              </items>
              <chunks count="1">
                <chunk name="Container">
                  <items count="4">
                    ${item('Description', 'gh_string', '10', 'Translate (move) an object along a vector.')}
                    ${item('InstanceGuid', 'gh_guid', '9', instance)}
                    ${item('Name', 'gh_string', '10', 'Move')}
                    ${item('NickName', 'gh_string', '10', 'Move')}
                  </items>
                  <chunks count="4">
                    ${bounds(x, y, 44, 44)}
                    <chunk name="param_input" index="0">
                      <items count="7">
                        ${item('Description', 'gh_string', '10', 'Base geometry')}
                        ${item('InstanceGuid', 'gh_guid', '9', inG)}
                        ${item('Name', 'gh_string', '10', 'Geometry')}
                        ${item('NickName', 'gh_string', '10', 'G')}
                        ${item('Optional', 'gh_bool', '1', 'false')}
                        ${sourceItem(0, geometryRef._guid)}
                        ${item('SourceCount', 'gh_int32', '3', '1')}
                      </items>
                      <chunks count="1">${bounds(x + 2, y + 2, 14, 14)}</chunks>
                    </chunk>
                    <chunk name="param_input" index="1">
                      <items count="7">
                        ${item('Description', 'gh_string', '10', 'Translation vector')}
                        ${item('InstanceGuid', 'gh_guid', '9', inT)}
                        ${item('Name', 'gh_string', '10', 'Translation')}
                        ${item('NickName', 'gh_string', '10', 'T')}
                        ${item('Optional', 'gh_bool', '1', 'false')}
                        ${sourceItem(0, translationRef._guid)}
                        ${item('SourceCount', 'gh_int32', '3', '1')}
                      </items>
                      <chunks count="1">${bounds(x + 2, y + 18, 14, 14)}</chunks>
                    </chunk>
                    <chunk name="param_output" index="0">
                      <items count="6">
                        ${item('Description', 'gh_string', '10', 'Translated geometry')}
                        ${item('InstanceGuid', 'gh_guid', '9', outGuid)}
                        ${item('Name', 'gh_string', '10', 'Geometry')}
                        ${item('NickName', 'gh_string', '10', 'G')}
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
  // Optional per-example canvas framing (default = compact left band).
  const view = objects._meta.view ?? { x: 400, y: 200, zoom: 0.75 };
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
                ${item('Target', 'gh_drawing_point', '30', `\n                  <X>${view.x}</X>\n                  <Y>${view.y}</Y>\n                `)}
                ${item('Zoom', 'gh_single', '5', String(view.zoom))}
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
  const hint = obj?.node?.key || obj?.key || (obj?.xml ? 'xml-only' : typeof obj);
  throw new Error('object missing InstanceGuid: ' + hint);
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
  // Live GH scribble height ≈ size×0.93 (Cassis-measured Ca→Cc); keep Bounds pad ±5.
  const th = size * 0.93;
  const bx = x - 5;
  const by = y - 5;
  const bw = tw + 10;
  const bh = th + 10;
  const node = { key: 'scribble', instance, outputs: [] };
  // Pivot must equal Ca (not Bounds center) — otherwise GH shifts the scribble on load.
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
                    ${item('Font', 'gh_string', '10', 'Arial')}
                    ${item('InstanceGuid', 'gh_guid', '9', instance)}
                    ${item('Italic', 'gh_bool', '1', 'false')}
                    ${item('Name', 'gh_string', '10', 'Scribble')}
                    ${item('NickName', 'gh_string', '10', 'Scribble')}
                    ${item('Size', 'gh_single', '5', String(size))}
                    ${item('Text', 'gh_string', '10', esc(text))}
                  </items>
                  <chunks count="1">
                    <chunk name="Attributes">
                      <items count="2">
                        ${item('Bounds', 'gh_drawing_rectanglef', '35', `\n                          <X>${bx}</X>\n                          <Y>${by}</Y>\n                          <W>${bw}</W>\n                          <H>${bh}</H>\n                        `)}
                        ${item('Pivot', 'gh_drawing_pointf', '31', `\n                          <X>${x}</X>\n                          <Y>${y}</Y>\n                        `)}
                      </items>
                    </chunk>
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
  // Pipeline: Robot → Env+Traj → Plan → Play (flat wires: Merge Y ≈ Plan Goal).
  const { title, note } = exampleHeader(
    '01 · Quick plan',
    'Auto Plan on. Scrub Preview when Status OK.',
  );
  const hy = PIPE.y0;
  const cy = stageContentY(hy);

  // Robot
  const rx = PIPE.x0;
  const robot = ur10eRobot(rx, cy);
  const start = motusComponent('joints', rx, cy + 100, {}, { jointValues: MOTION_START });

  // Env + Traj (goals)
  const ex = rx + 200;
  const goalJoint = motusComponent('joints', ex, cy, {}, { jointValues: GOAL_JOINTS });
  const tcp = motusComponent('tcpPose', ex + 160, cy, {
    Robot: [outRef(robot.node, 'Robot')],
    State: [outRef(goalJoint.node, 'State')],
  });
  const uz = nativeUnitZ(ex, cy + 120);
  const ptLin = nativeConstructPoint(ex, cy + 180, [0.48, 0.18, 0.48]);
  const plLin = nativePlane(ex + 140, cy + 180, ptLin.node.outputs[0], uz.node.outputs[0]);
  const goalsMerge = nativeMerge(ex + 320, cy + 40, [
    outRef(goalJoint.node, 'State'),
    outRef(tcp.node, 'Plane'),
    outRef(plLin.node, 'Plane'),
  ]);

  // Plan
  const planX = ex + 460;
  const planY = cy;
  const plan = motusComponent('plan', planX, planY, {
    Robot: [outRef(robot.node, 'Robot')],
    Goal: [outRef(goalsMerge.node, 'Result')],
    Start: [outRef(start.node, 'State')],
  });

  // Play
  const { scrub, preview } = previewWithScrub(planX, planY, outRef(plan.node, 'Trajectory'));
  const stackX = planX + PLAN_PREVIEW_DX;
  const waypoints = motusComponent('waypoints', stackX, belowPreview(planY), {
    Trajectory: [outRef(plan.node, 'Trajectory')],
  });
  const exp = motusComponent('export', stackX, belowPreview(planY) + 100, {
    Trajectory: [outRef(plan.node, 'Trajectory')],
  });

  const gRobot = stageGroup('Robot', [robot, start], GROUP_COLOUR.robot, rx, hy);
  const gEnv = stageGroup('Env + Traj', [
    goalJoint, tcp, { xml: uz.xml, node: uz.node }, ptLin, plLin, goalsMerge,
  ], GROUP_COLOUR.goals, ex, hy);
  const gPlan = stageGroup('Plan', [plan], GROUP_COLOUR.plan, planX, hy);
  const gPlay = stageGroup('Play', [scrub, preview, waypoints, exp], GROUP_COLOUR.play, planX + PIPE.playHeaderDx, hy);

  const objs = [
    title, note, robot, start, goalJoint, tcp,
    { xml: uz.xml }, { xml: ptLin.xml }, { xml: plLin.xml }, goalsMerge,
    plan, scrub, preview, waypoints, exp,
    gRobot.header, gEnv.header, gPlan.header, gPlay.header,
    gRobot.group, gEnv.group, gPlan.group, gPlay.group,
  ];
  objs._meta = {
    fileName: '01_quick_plan.ghx',
    description: 'Quick plan: sequential Joint State + TCP Pose LIN + Plane goal (via Merge) -> Preview / Export / Waypoints. Auto Plan on; drag Motus Scrub or Play.',
    view: { x: 520, y: 220, zoom: 0.7 },
  };
  return buildGraph(objs);
}

/** 02 — collision RRT + shapes + SRDF/group/attach (was 03+04+05). */
function graph02() {
  // Pipeline: Robot → Env+Traj (obstacles+attach+RRT) → Plan → Play
  const { title, note } = exampleHeader(
    '02 · Collision + SRDF',
    'RRT detours the sphere. Group pin unwired until OMPL fix. Auto Plan on.',
  );
  const hy = PIPE.y0;
  const cy = stageContentY(hy);

  // Robot
  const rx = PIPE.x0;
  const robot = ur10eRobot(rx, cy);
  const start = motusComponent('joints', rx, cy + 90, {}, { jointValues: COLLISION_START });
  const goal = motusComponent('joints', rx, cy + 200, {}, { jointValues: COLLISION_GOAL });

  // Env + Traj — obstacles row, then attach/RRT below (no overlap)
  const ex = rx + 200;
  const sphereCenter = nativeConstructPoint(ex, cy, [0.76, 0.50, 0.73]);
  const sphere = motusComponent('colSphere', ex + 160, cy, {
    Center: [outRef(sphereCenter.node, 'Point')],
  }, { text: { Name: 'block' }, numbers: { Radius: 0.18 } });
  const uz = nativeUnitZ(ex, cy + 120);
  const boxOrigin = nativeConstructPoint(ex, cy + 180, [0.70, 0.20, 0.04]);
  const boxPlane = nativePlane(ex + 140, cy + 180, boxOrigin.node.outputs[0], uz.node.outputs[0]);
  const box = motusComponent('colBox', ex + 300, cy + 160, { Plane: [outRef(boxPlane.node, 'Plane')] }, {
    text: { Name: 'table' },
    numbers: { HalfX: 0.25, HalfY: 0.18, HalfZ: 0.02 },
  });
  const obstaclesMerge = nativeMerge(ex + 460, cy + 40, [
    outRef(sphere.node, 'Object'),
    outRef(box.node, 'Object'),
  ]);
  const srdfPanel = pathPanel(ex, cy + 300, repoRel('assets', 'srdf', 'table_base.srdf'), 'Srdf', 280, 40);
  const scene = motusComponent('colScene', ex + 460, cy + 160, {
    Objects: [outRef(obstaclesMerge.node, 'Result')],
    Srdf: [outRef(srdfPanel.node, 'Text')],
  });
  const group = motusComponent('group', ex + 620, cy + 160, { Group: [outRef(scene.node, 'Groups')] });
  // Attach + RRT — below obstacles
  const attachY = cy + 400;
  const graspCenter = nativeConstructPoint(ex, attachY, [0, 0, 0.02]);
  const grasp = motusComponent('colSphere', ex + 160, attachY, {
    Center: [outRef(graspCenter.node, 'Point')],
  }, { text: { Name: 'grasp' }, numbers: { Radius: 0.03 } });
  const attach = motusComponent('attach', ex + 340, attachY, { Object: [outRef(grasp.node, 'Object')] }, { text: { Name: 'grasp' } });
  const rrt = motusComponent('rrtSettings', ex + 500, attachY, {});

  // Plan (Motus Planning Group stays on canvas; Plan Group pin unwired — ShowGroup for ParameterData order)
  const planX = ex + 800;
  const planY = cy;
  const plan = motusComponent('plan', planX, planY, {
    Robot: [outRef(robot.node, 'Robot')],
    Goal: [outRef(goal.node, 'State')],
    Start: [outRef(start.node, 'State')],
    Collision: [outRef(scene.node, 'Scene')],
    Attach: [outRef(attach.node, 'Attach')],
    RrtSettings: [outRef(rrt.node, 'Settings')],
  }, { advanced: ['Collision', 'Group', 'Attach', 'RrtSettings'] });
  const { scrub, preview } = previewWithScrub(planX, planY, outRef(plan.node, 'Trajectory'));

  const gRobot = stageGroup('Robot', [robot, start, goal], GROUP_COLOUR.robot, rx, hy);
  const gEnv = stageGroup('Env + Traj', [
    sphereCenter, sphere, boxOrigin, { xml: uz.xml, node: uz.node }, boxPlane, box,
    obstaclesMerge, srdfPanel, scene, group,
    graspCenter, grasp, attach, rrt,
  ], GROUP_COLOUR.env, ex, hy);
  const gPlan = stageGroup('Plan', [plan], GROUP_COLOUR.plan, planX, hy);
  const gPlay = stageGroup('Play', [scrub, preview], GROUP_COLOUR.play, planX + PIPE.playHeaderDx, hy);

  const objs = [
    title, note, robot, start, goal,
    { xml: sphereCenter.xml }, sphere,
    { xml: boxOrigin.xml }, { xml: uz.xml }, { xml: boxPlane.xml }, box, obstaclesMerge,
    srdfPanel, scene, group,
    { xml: graspCenter.xml }, grasp, attach, rrt, plan, scrub, preview,
    gRobot.header, gEnv.header, gPlan.header, gPlay.header,
    gRobot.group, gEnv.group, gPlan.group, gPlay.group,
  ];
  objs._meta = {
    fileName: '02_collision_srdf.ghx',
    description: 'Collision RRT: sphere blocks the joint-linear mid-path so Plan detours. ColSphere+ColBox via Merge → ColScene (SRDF) + Attach + RRT. Auto Plan on; scrub Preview.',
    view: { x: 700, y: 280, zoom: 0.55 },
  };
  return buildGraph(objs);
}

/** 03 — URDF load + base/tool frames + Robotiq mesh (was 06+07+09+10). */
function graph03() {
  // Pipeline: Robot (URDF+tool) → Env+Traj (start/goal) → Plan → Play
  const { title, note } = exampleHeader(
    '03 · URDF + tool frames',
    'Custom URDF + Tool TCP. Preview ShowStart on. Auto Plan on.',
  );
  const hy = PIPE.y0;
  const cy = stageContentY(hy);

  // Robot column — URDF / base / tool → Motus Robot
  const rx = PIPE.x0;
  const urdfFile = pathPanel(rx, cy, repoRel('resources', 'robots', 'ur10e_robotiq', 'ur10e_robotiq.urdf'), 'Urdf');
  const basePl = nativeXYPlane(rx, cy + 80);
  const tcpPt = nativeConstructPoint(rx, cy + 160, [0, 0, 0.1633]);
  const ux = nativeUnitX(rx, cy + 240);
  const tcpPl = nativePlane(rx + 160, cy + 160, tcpPt.node.outputs[0], ux.node.outputs[0]);
  const meshPath = pathPanel(rx, cy + 320, repoRel('resources', 'tools', 'robotiq_2f85_tcp_local.stl'), 'Mesh');
  const loadMesh = motusComponent('loadMesh', rx + 220, cy + 300, {
    Path: [outRef(meshPath.node, 'Text')],
  });
  const tool = motusComponent('tool', rx + 380, cy + 140, {
    TCP: [outRef(tcpPl.node, 'Plane')],
    Geometry: [outRef(loadMesh.node, 'Mesh')],
  }, { text: { Name: 'robotiq_2f85' }, toolCapabilities: 'Robotiq2F85' });
  const robot = motusComponent('robot', rx + 560, cy, {
    Path: [outRef(urdfFile.node, 'Text')],
    Base: [outRef(basePl.node, 'Plane')],
    Tool: [outRef(tool.node, 'Tool')],
  }, { text: { BaseLink: 'base_link', TipLink: 'tool0' }, hidden: true });

  // Env + Traj — joint start/goal
  const ex = rx + 720;
  const start = motusComponent('joints', ex, cy, {}, { jointValues: START_JOINTS });
  const goal = motusComponent('joints', ex, cy + 110, {}, { jointValues: GOAL_JOINTS });

  // Plan + Play
  const planX = ex + 200;
  const planY = cy;
  const plan = motusComponent('plan', planX, planY, {
    Robot: [outRef(robot.node, 'Robot')],
    Goal: [outRef(goal.node, 'State')],
    Start: [outRef(start.node, 'State')],
  });
  const { scrub, preview } = previewWithScrub(planX, planY, outRef(plan.node, 'Trajectory'));
  const exp = motusComponent('export', planX + PLAN_PREVIEW_DX, belowPreview(planY), {
    Trajectory: [outRef(plan.node, 'Trajectory')],
  });

  const gRobot = stageGroup('Robot', [
    urdfFile, basePl, tcpPt, { xml: ux.xml, node: ux.node }, tcpPl, meshPath, loadMesh, tool, robot,
  ], GROUP_COLOUR.robot, rx, hy);
  const gEnv = stageGroup('Env + Traj', [start, goal], GROUP_COLOUR.goals, ex, hy);
  const gPlan = stageGroup('Plan', [plan], GROUP_COLOUR.plan, planX, hy);
  const gPlay = stageGroup('Play', [scrub, preview, exp], GROUP_COLOUR.play, planX + PIPE.playHeaderDx, hy);

  const objs = [
    title, note, urdfFile, basePl,
    { xml: tcpPt.xml }, { xml: ux.xml }, { xml: tcpPl.xml },
    meshPath, loadMesh, tool, robot, start, goal, plan, scrub, preview, exp,
    gRobot.header, gEnv.header, gPlan.header, gPlay.header,
    gRobot.group, gEnv.group, gPlan.group, gPlay.group,
  ];
  objs._meta = {
    fileName: '03_urdf_tool_frames.ghx',
    description: 'Motus Robot URDF + Base override + Robotiq Tool (Load Mesh, Cap=Robotiq2F85) + Start + Preview ShowStart. Auto Plan on.',
    view: { x: 700, y: 260, zoom: 0.55 },
  };
  return buildGraph(objs);
}

/** 04 — motion program: PTP + LIN + CIRC + SET gripper (was 08+11). */
function graph04() {
  // Pipeline: Robot → Env+Traj (one row per move) → Plan (Program) → Play
  const { title, note } = exampleHeader(
    '04 · Motion program',
    'One row per move → Merge → Program. Auto Plan on; scrub when Status OK.',
  );
  const hy = PIPE.y0;
  const cy = stageContentY(hy);

  // Robot
  const rx = PIPE.x0;
  const robot = ur10eRobot(rx, cy);
  const start = motusComponent('joints', rx, cy + 100, {}, { jointValues: MOTION_START });

  // Env + Traj — one horizontal row per move (top→bottom = program order)
  const ex = rx + 200;
  const rowH = 200;

  // Row 1 — PTP
  const ptpGoal = motusComponent('joints', ex, cy, {}, { jointValues: GOAL_JOINTS });
  const stateOpen = motusComponent('toolState', ex + 160, cy, {
    Tool: [outRef(robot.node, 'Robot')],
  }, { toolStatePreset: 'Open' });
  const segPtp = motusComponent('segment', ex + 360, cy, {
    Goal: [outRef(ptpGoal.node, 'State')],
    ToolState: [outRef(stateOpen.node, 'State')],
  }, { text: { Type: 'PTP' } });

  // Row 2 — LIN
  const yLin = cy + rowH;
  const uz = nativeUnitZ(ex, yLin);
  const ptLin = nativeConstructPoint(ex, yLin + 60, [0.45, 0.15, 0.45]);
  const plLin = nativePlane(ex + 140, yLin + 60, ptLin.node.outputs[0], uz.node.outputs[0]);
  const segLin = motusComponent('segment', ex + 360, yLin + 40, {
    Goal: [outRef(plLin.node, 'Plane')],
  }, { text: { Type: 'LIN' } });

  // Row 3 — CIRC
  const yCirc = cy + rowH * 2;
  const ptVia = nativeConstructPoint(ex, yCirc, [0.453, 0.152, 0.45]);
  const plVia = nativePlane(ex + 140, yCirc, ptVia.node.outputs[0], uz.node.outputs[0]);
  const ptGoal = nativeConstructPoint(ex, yCirc + 80, [0.45, 0.154, 0.45]);
  const plGoal = nativePlane(ex + 140, yCirc + 80, ptGoal.node.outputs[0], uz.node.outputs[0]);
  const segCirc = motusComponent('segment', ex + 360, yCirc + 20, {
    Goal: [outRef(plGoal.node, 'Plane')],
    Via: [outRef(plVia.node, 'Plane')],
  }, { text: { Type: 'CIRC' } });

  // Row 4 — SET
  const ySet = cy + rowH * 3;
  const stateClosed = motusComponent('toolState', ex, ySet, {
    Tool: [outRef(robot.node, 'Robot')],
  }, { toolStatePreset: 'Closed' });
  const segSet = motusComponent('segment', ex + 360, ySet, {
    ToolState: [outRef(stateClosed.node, 'State')],
  }, { text: { Type: 'SET' }, numbers: { Duration: 0.2 } });

  const segsMerge = nativeMerge(ex + 540, cy + rowH, [
    outRef(segPtp.node, 'Segment'),
    outRef(segLin.node, 'Segment'),
    outRef(segCirc.node, 'Segment'),
    outRef(segSet.node, 'Segment'),
  ]);

  // Plan (Program)
  const progX = ex + 700;
  const progY = cy + rowH;
  const progPlan = motusComponent('progPlan', progX, progY, {
    Robot: [outRef(robot.node, 'Robot')],
    Segments: [outRef(segsMerge.node, 'Result')],
    Start: [outRef(start.node, 'State')],
  });
  const { scrub, preview } = previewWithScrub(progX, progY, outRef(progPlan.node, 'Trajectory'));
  const exp = motusComponent('export', progX + PLAN_PREVIEW_DX, belowPreview(progY), {
    Trajectory: [outRef(progPlan.node, 'Trajectory')],
  });

  const gRobot = stageGroup('Robot', [robot, start], GROUP_COLOUR.robot, rx, hy);
  const gEnv = stageGroup('Env + Traj', [
    ptpGoal, stateOpen, segPtp,
    { xml: uz.xml, node: uz.node }, ptLin, plLin, segLin,
    ptVia, plVia, ptGoal, plGoal, segCirc,
    stateClosed, segSet, segsMerge,
  ], GROUP_COLOUR.goals, ex, hy);
  const gPlan = stageGroup('Plan', [progPlan], GROUP_COLOUR.plan, progX, hy);
  const gPlay = stageGroup('Play', [scrub, preview, exp], GROUP_COLOUR.play, progX + PIPE.playHeaderDx, hy);

  const flat = [
    title, note, robot, start,
    ptpGoal, stateOpen, segPtp,
    { xml: uz.xml }, { xml: ptLin.xml }, { xml: plLin.xml }, segLin,
    { xml: ptVia.xml }, { xml: plVia.xml }, { xml: ptGoal.xml }, { xml: plGoal.xml }, segCirc,
    stateClosed, segSet, segsMerge, progPlan, scrub, preview, exp,
    gRobot.header, gEnv.header, gPlan.header, gPlay.header,
    gRobot.group, gEnv.group, gPlan.group, gPlay.group,
  ];
  flat._meta = {
    fileName: '04_motion_program.ghx',
    description: 'Motion program: PTP + LIN + CIRC + SET gripper (via Merge) -> Motus Program -> Preview / Export. Auto Plan on; drag Motus Scrub or Play.',
    view: { x: 620, y: 360, zoom: 0.55 },
  };
  return buildGraph(flat);
}

/** 05 — Serial Chain + Reach Samples (on-component preview; no Plan). */
function graph05() {
  // Pipeline: Robot (Serial) → Env+Traj (N) → Play (Reach — no Plan stage)
  const { title, note } = exampleHeader(
    '05 · Serial + Reach',
    'Rail on: L0 = +Z stroke, L1… = planar arm. Drag N; edit L / Q on Serial. No Plan — Rhino preview only.',
  );
  const hy = PIPE.y0;
  const cy = stageContentY(hy);

  const rx = PIPE.x0;
  const chain = motusComponent('serialChain', rx, cy, {}, {
    jointValues: [0.50, 0.35, 0.28, 0.22, 0.15, 0.10],
    numberList: { Home: [0.25, 0.55, -0.85, 0.60, 0.25, 0] },
    bools: { Rail: true },
  });

  const ex = rx + 220;
  const nSlider = nativeNumberSlider(ex, cy + 40, {
    value: 128, min: 32, max: 512, nick: 'N', w: 200, digits: 0, interval: 1,
  });

  const playX = ex + 360;
  const reach = motusComponent('reachSamples', playX, cy + 20, {
    Robot: [outRef(chain.node, 'Robot')],
    Count: [outRef(nSlider.node, 'Number')],
  });

  const gRobot = stageGroup('Robot', [chain], GROUP_COLOUR.robot, rx, hy);
  const gEnv = stageGroup('Env + Traj', [nSlider], GROUP_COLOUR.goals, ex, hy);
  const gPlay = stageGroup('Play', [reach], GROUP_COLOUR.play, playX, hy);

  const objs = [
    title, note, chain, reach, nSlider,
    gRobot.header, gEnv.header, gPlay.header,
    gRobot.group, gEnv.group, gPlay.group,
  ];
  objs._meta = {
    fileName: '05_serial_reach.ghx',
    description: 'Rail Serial Chain (L0 stroke + arm) → Reach Samples (N slider). On-component preview; no Plan.',
    view: { x: 400, y: 200, zoom: 0.85 },
  };
  return buildGraph(objs);
}

/**
 * 06 — UR prefab + 1-DOF turntable: Robotiq TCP tracks spoke fixture; GH box → Robot Attach.
 * AllDrivers = tip×6 + turntable_yaw. Plan moves arm+table together.
 * Fixture = Center Box → Robot At + Point → Ao on turntable_link (TreeFK).
 */
function graph06() {
  // Pipeline: Robot → Env+Traj (fixture + waypoints) → Plan → Play
  const { title, note } = exampleHeader(
    '06 · UR + Turntable',
    'Fixture: Box → Robot At; Point → Ao on turntable_link (TreeFK). AllDrivers Plan moves arm + turntable — scrub Preview (Show TCP).',
  );
  const hy = PIPE.y0;
  const cy = stageContentY(hy);

  // Robot — xacro + Motus Robot (attach wired from Env)
  const rx = PIPE.x0;
  const urdfFile = pathPanel(
    rx, cy,
    repoRel('resources', 'robots', 'ur10e_robotiq', 'ur10e_with_turntable.xacro'),
    'Xacro', 220, 36,
  );

  // Env — fixture geometry + joint waypoints
  const ex = rx + 280;
  const xy = nativeXYPlane(ex, cy);
  const fixtureBox = nativeCenterBox(ex, cy + 60, outRef(xy.node, 'Plane'), [0.02, 0.02, 0.02]);
  const attachOrigin = nativeConstructPoint(ex, cy + 160, [0.275, 0.025, 0.035]);

  const robot = motusComponent('robot', rx, cy + 80, {
    Path: [outRef(urdfFile.node, 'Text')],
    Attach: [outRef(fixtureBox.node, 'Box')],
    AttachOrigin: [outRef(attachOrigin.node, 'Point')],
  }, {
    text: { BaseLink: 'world', TipLink: 'tool0', AttachLink: 'turntable_link' },
    bools: { AllDrivers: true },
    hidden: true,
  });

  const start = motusComponent('joints', ex + 200, cy, {}, { jointValues: TT_START });
  const midGoals = TT_WAYPOINTS.slice(1).map((q, i) =>
    motusComponent('joints', ex + 200, cy + 90 + i * 90, {}, { jointValues: q }));
  const goalsMerge = nativeMerge(ex + 380, cy + 180, midGoals.map((g) => outRef(g.node, 'State')));

  // Plan + Play
  const planX = ex + 540;
  const planY = cy + 100;
  const plan = motusComponent('plan', planX, planY, {
    Robot: [outRef(robot.node, 'Robot')],
    Goal: [outRef(goalsMerge.node, 'Result')],
    Start: [outRef(start.node, 'State')],
  });
  const preview = previewWithScrub(planX, planY, outRef(plan.node, 'Trajectory'));

  const gRobot = stageGroup('Robot', [urdfFile, robot], GROUP_COLOUR.robot, rx, hy);
  const gEnv = stageGroup('Env + Traj', [
    { xml: xy.xml, node: xy.node }, fixtureBox, attachOrigin,
    start, ...midGoals, goalsMerge,
  ], GROUP_COLOUR.env, ex, hy);
  const gPlan = stageGroup('Plan', [plan], GROUP_COLOUR.plan, planX, hy);
  const gPlay = stageGroup('Play', [preview.scrub, preview.preview], GROUP_COLOUR.play, planX + PIPE.playHeaderDx, hy);

  const objs = [
    title, note,
    urdfFile, robot, start, ...midGoals, goalsMerge,
    { xml: xy.xml }, { xml: fixtureBox.xml }, { xml: attachOrigin.xml },
    plan, preview.scrub, preview.preview,
    gRobot.header, gEnv.header, gPlan.header, gPlay.header,
    gRobot.group, gEnv.group, gPlan.group, gPlay.group,
  ];
  objs._meta = {
    fileName: '06_turntable_group.ghx',
    description:
      'UR10e+turntable: GH fixture box → Robot Attach on turntable_link (TreeFK); AllDrivers multi-waypoint Robotiq TCP tracks spoke corner.',
    view: { x: 620, y: 320, zoom: 0.55 },
  };
  return buildGraph(objs);
}

/**
 * 07 — Compact L→R: boxes→gripper→Tool Rd→Robot→PTP Ramp→Preview (pinch).
 * Revolute about +Z at Y=±0.035: pad centers sit at local (−X,+Z) so +q (and mimic −1)
 * swings both pads toward the midplane. Avoid joint-centered long-Z boxes (look like a
 * cross through the palm and spin in place instead of pinching).
 * Cap=Custom + Bd=j_left; Closed → driver 0.8 rad. Opens framed on the full story.
 */
function graph07() {
  // Pipeline: Env+Traj (author) → Robot (Tool+arm) → Plan → Play
  const { title, note } = exampleHeader(
    '07 · Gripper Tool pinch',
    'Author → Tool (Cap=Custom, Bd=j_left) → Robot → PTP Ramp Closed. Scrub = pinch.',
  );
  const hy = PIPE.y0;
  const cy = stageContentY(hy);

  // Env + Traj — author gripper geometry
  const ex = PIPE.x0;
  const yPalm = cy;
  const yL = cy + 100;
  const yR = cy + 220;

  const xy = hidePreview(nativeXYPlane(ex, yPalm));
  const uz = hidePreview(nativeUnitZ(ex, yPalm + 70));
  const palmBox = hidePreview(nativeCenterBox(ex, yL, outRef(xy.node, 'Plane'), [0.10, 0.08, 0.02]));
  const fingerCenter = hidePreview(nativeConstructPoint(ex, yR, [-0.045, 0, 0.055]));
  const fingerPl = hidePreview(nativePlane(ex + 140, yR, fingerCenter.node.outputs[0], uz.node.outputs[0]));
  const leftBox = hidePreview(nativeCenterBox(ex + 280, yR, outRef(fingerPl.node, 'Plane'), [0.07, 0.012, 0.08]));
  const rightBox = hidePreview(nativeCenterBox(ex + 280, yR + 100, outRef(fingerPl.node, 'Plane'), [0.07, 0.012, 0.08]));

  const palm = motusComponent('urdfLink', ex + 440, yL, {
    Visual: [outRef(palmBox.node, 'Box')],
  }, { text: { Name: 'palm' }, hidden: true });
  const left = motusComponent('urdfLink', ex + 440, yR, {
    Visual: [outRef(leftBox.node, 'Box')],
  }, { text: { Name: 'L' }, hidden: true });
  const right = motusComponent('urdfLink', ex + 440, yR + 100, {
    Visual: [outRef(rightBox.node, 'Box')],
  }, { text: { Name: 'R' }, hidden: true });

  const leftOrigin = hidePreview(nativeConstructPoint(ex + 580, yL, [0, 0.035, 0]));
  const rightOrigin = hidePreview(nativeConstructPoint(ex + 580, yR, [0, -0.035, 0]));
  const leftAxis = hidePreview(nativeLineSdl(ex + 700, yL, leftOrigin.node.outputs[0], uz.node.outputs[0], 0.05));
  const rightAxis = hidePreview(nativeLineSdl(ex + 700, yR, rightOrigin.node.outputs[0], uz.node.outputs[0], 0.05));

  const jLeft = motusComponent('urdfJoint', ex + 840, yPalm, {
    Axis: [outRef(leftAxis.node, 'Line')],
  }, {
    text: { Name: 'j_left', Type: 'Revolute', Parent: 'palm', Child: 'L' },
    numbers: { Lower: 0, Upper: 0.8 },
  });
  const jRight = motusComponent('urdfJoint', ex + 840, yPalm + 270, {
    Axis: [outRef(rightAxis.node, 'Line')],
  }, {
    text: {
      Name: 'j_right', Type: 'Revolute', Parent: 'palm', Child: 'R', MimicJoint: 'j_left',
    },
    numbers: { Lower: 0, Upper: 0.8, MimicMult: -1, MimicOffset: 0 },
  });

  const linksMerge = nativeMerge(ex + 1020, yL, [
    outRef(palm.node, 'Link'),
    outRef(left.node, 'Link'),
    outRef(right.node, 'Link'),
  ]);
  const jointsMerge = nativeMerge(ex + 1020, yR, [
    outRef(jLeft.node, 'Joint'),
    outRef(jRight.node, 'Joint'),
  ]);
  const assemble = motusComponent('urdfAssemble', ex + 1180, yL + 20, {
    Links: [outRef(linksMerge.node, 'Result')],
    Joints: [outRef(jointsMerge.node, 'Result')],
  }, { text: { Name: 'demo_gripper', Tip: 'palm' } });

  // Robot — Tool + UR arm
  const rx = ex + 1360;
  const tool = motusComponent('tool', rx, yL, {
    Description: [outRef(assemble.node, 'Description')],
  }, { text: { Name: 'demo_gripper', Binding: 'j_left' }, toolCapabilities: 'Custom' });
  const stateClosed = motusComponent('toolState', rx, yR + 160, {
    Tool: [outRef(tool.node, 'Tool')],
  }, { toolStatePreset: 'Closed' });
  const urdfFile = pathPanel(rx + 200, yL, repoRel('assets', 'ur10e', 'ur10e_minimal.urdf'), 'Urdf', 140, 36);
  const robot = motusComponent('robot', rx + 360, yL, {
    Path: [outRef(urdfFile.node, 'Text')],
    Tool: [outRef(tool.node, 'Tool')],
  }, { text: { BaseLink: 'base_link', TipLink: 'tool0' }, hidden: true });
  const start = motusComponent('joints', rx + 520, yL, {}, { jointValues: START_JOINTS });
  const goal = motusComponent('joints', rx + 520, yL + 110, {}, { jointValues: GOAL_JOINTS });

  // Plan
  const planX = rx + 700;
  const planY = yL + 20;
  const segPtp = motusComponent('segment', planX, planY, {
    Goal: [outRef(goal.node, 'State')],
    ToolState: [outRef(stateClosed.node, 'State')],
  }, { text: { Type: 'PTP' }, toolMode: 'Ramp' });
  const prog = motusComponent('progPlan', planX + 180, planY, {
    Robot: [outRef(robot.node, 'Robot')],
    Segments: [outRef(segPtp.node, 'Segment')],
    Start: [outRef(start.node, 'State')],
  });
  const { scrub, preview } = previewWithScrub(planX + 180, planY, outRef(prog.node, 'Trajectory'));

  const gEnv = stageGroup('Env + Traj', [
    { xml: xy.xml, node: xy.node },
    { xml: uz.xml, node: uz.node },
    palmBox,
    { xml: fingerCenter.xml, node: fingerCenter.node },
    { xml: fingerPl.xml, node: fingerPl.node },
    leftBox, rightBox, palm, left, right,
    { xml: leftOrigin.xml, node: leftOrigin.node },
    { xml: rightOrigin.xml, node: rightOrigin.node },
    leftAxis, rightAxis, jLeft, jRight, linksMerge, jointsMerge, assemble,
  ], GROUP_COLOUR.env, ex, hy);
  const gRobot = stageGroup('Robot', [
    tool, stateClosed, urdfFile, robot, start, goal,
  ], GROUP_COLOUR.robot, rx, hy);
  const gPlan = stageGroup('Plan', [segPtp, prog], GROUP_COLOUR.plan, planX, hy);
  const gPlay = stageGroup('Play', [scrub, preview], GROUP_COLOUR.play, planX + 180 + PIPE.playHeaderDx, hy);

  const objs = [
    title, note,
    { xml: xy.xml }, { xml: uz.xml }, palmBox,
    { xml: fingerCenter.xml }, { xml: fingerPl.xml }, leftBox, rightBox,
    palm, left, right,
    { xml: leftOrigin.xml }, { xml: rightOrigin.xml }, leftAxis, rightAxis,
    jLeft, jRight, linksMerge, jointsMerge, assemble,
    tool, stateClosed,
    urdfFile, robot, start, goal, segPtp, prog, scrub, preview,
    gEnv.header, gRobot.header, gPlan.header, gPlay.header,
    gEnv.group, gRobot.group, gPlan.group, gPlay.group,
  ];
  objs._meta = {
    fileName: '07_urdf_gripper_tool.ghx',
    description:
      'Boxes→ULink→UJoint→Assemble→Tool Rd (Cap=Custom, Bd=j_left)→ur10e_minimal→PTP Ramp Closed→Preview pinch.',
    view: { x: 1100, y: 340, zoom: 0.38 },
  };
  return buildGraph(objs);
}

function graph08() {
  // Pipeline: Robot (Stewart) → Env+Traj (TCP loop) → Plan → Play
  const { title, note } = exampleHeader(
    '08 · Stewart TCP path',
    'Sliders → Stewart → Plan TCP loop → Preview. Q = leg lengths (m). Drag Br/Pr; keep Lmin/Lmax if Status hits StrokeLimit. Auto Plan on.',
  );
  const hy = PIPE.y0;
  const cy = stageContentY(hy);

  // Robot
  const rx = PIPE.x0;
  const floatOpts = { digits: 3, interval: 0, w: 180 };
  const br = nativeNumberSlider(rx, cy, { ...floatOpts, value: 0.5, min: 0.2, max: 0.8, nick: 'Br' });
  const pr = nativeNumberSlider(rx, cy + 40, { ...floatOpts, value: 0.3, min: 0.1, max: 0.6, nick: 'Pr' });
  const lmin = nativeNumberSlider(rx, cy + 80, { ...floatOpts, value: 0.35, min: 0.2, max: 0.6, nick: 'Lmin' });
  const lmax = nativeNumberSlider(rx, cy + 120, { ...floatOpts, value: 0.90, min: 0.5, max: 1.2, nick: 'Lmax' });
  const stewart = motusComponent('stewart', rx + 220, cy + 20, {
    BaseRadius: [outRef(br.node, 'Number')],
    PlatformRadius: [outRef(pr.node, 'Number')],
    MinStroke: [outRef(lmin.node, 'Number')],
    MaxStroke: [outRef(lmax.node, 'Number')],
  });

  // Env + Traj — TCP loop (pitch 72 avoids AABB kisses on live Construct Point)
  const ex = rx + 420;
  const uz = nativeUnitZ(ex, cy);
  const pathPts = [
    [0, 0, 0.62],
    [0.08, 0.02, 0.70],
    [0.02, 0.10, 0.52],
    [-0.08, 0.04, 0.68],
    [-0.04, -0.08, 0.55],
    [0.06, -0.06, 0.62],
    [0, 0, 0.62],
  ];
  const pathParts = [];
  const planeRefs = [];
  for (let i = 0; i < pathPts.length; i++) {
    const [x, y, z] = pathPts[i];
    const py = cy + 40 + i * 72;
    const pt = nativeConstructPoint(ex, py, [x, y, z]);
    const pl = nativePlane(ex + 120, py, outRef(pt.node, 'Point'), outRef(uz.node, 'Vector'));
    pathParts.push({ xml: pt.xml, node: pt.node }, { xml: pl.xml, node: pl.node });
    planeRefs.push(outRef(pl.node, 'Plane'));
  }
  const goalsMerge = nativeMerge(ex + 240, cy + 160, planeRefs.slice(1));

  // Plan + Play
  const planX = ex + 400;
  const planY = cy + 80;
  const plan = motusComponent('plan', planX, planY, {
    Robot: [outRef(stewart.node, 'Robot')],
    Start: [planeRefs[0]],
    Goal: [outRef(goalsMerge.node, 'Result')],
  });
  const { scrub, preview } = previewWithScrub(planX, planY, outRef(plan.node, 'Trajectory'));
  const waypoints = motusComponent('waypoints', planX + PLAN_PREVIEW_DX, belowPreview(planY), {
    Trajectory: [outRef(plan.node, 'Trajectory')],
  });

  const gRobot = stageGroup('Robot', [br, pr, lmin, lmax, stewart], GROUP_COLOUR.robot, rx, hy);
  const gEnv = stageGroup('Env + Traj', [
    { xml: uz.xml, node: uz.node }, ...pathParts, goalsMerge,
  ], GROUP_COLOUR.goals, ex, hy);
  const gPlan = stageGroup('Plan', [plan], GROUP_COLOUR.plan, planX, hy);
  const gPlay = stageGroup('Play', [scrub, preview, waypoints], GROUP_COLOUR.play, planX + PIPE.playHeaderDx, hy);

  const objs = [
    title, note,
    br, pr, lmin, lmax, stewart,
    { xml: uz.xml },
    ...pathParts, goalsMerge, plan, scrub, preview, waypoints,
    gRobot.header, gEnv.header, gPlan.header, gPlay.header,
    gRobot.group, gEnv.group, gPlan.group, gPlay.group,
  ];
  objs._meta = {
    fileName: '08_stewart_tcp_path.ghx',
    description:
      'Sliders Br/Pr/Lmin/Lmax → Motus Stewart → dramatic multi-waypoint TCP loop (heave/sway) → Preview + Waypoints (leg lengths in meters). Wire Base+Plat for custom anchors.',
    view: { x: 700, y: 360, zoom: 0.5 },
  };
  return buildGraph(objs);
}

/**
 * Shared Walk graph (Body+Leg+Mechanism + Ground + arc → Walk → Preview).
 * Logic asserted by Motus.NET Example09 + qa-smoke for N=6. N is the only structural knob.
 */
function graphWalking({ n, label, fileName, description }) {
  // Pipeline: Robot (Body+Leg+Mech) → Env+Traj (terrain+arc) → Plan (Walk) → Play
  const { title, note } = exampleHeader(
    label,
    'N → Body → Leg → Mechanism → Walk; Ground → Tn; arc → Tr → Preview. Drag N (4–12). Green rings = planted feet.',
  );
  const hy = PIPE.y0;
  const cy = stageContentY(hy);

  // Robot
  const rx = PIPE.x0;
  const nSlider = nativeNumberSlider(rx, cy, { value: n, min: 4, max: 12, nick: 'N', w: 180 });
  const body = motusComponent('body', rx + 220, cy, {
    N: [outRef(nSlider.node, 'Number')],
  });
  const leg = motusComponent('leg', rx + 220, cy + 160, {});
  const mech = motusComponent('mechanism', rx + 360, cy + 60, {
    Body: [outRef(body.node, 'Body')],
    Leg: [outRef(leg.node, 'Leg')],
  });

  // Env + Traj — terrain + body arc
  const ex = rx + 520;
  const uz = nativeUnitZ(ex, cy);
  const groundOrigin = nativeConstructPoint(ex, cy + 60, [0.22, 0, 0]);
  const ground = motusComponent('terrainPatch', ex + 160, cy + 40, {
    Origin: [outRef(groundOrigin.node, 'Point')],
  }, { numbers: { Amp: 0.02 } });
  const arcParts = [];
  const planeRefs = [];
  const ARC_N = 9;
  for (let i = 0; i < ARC_N; i++) {
    const a = Math.PI - (i / (ARC_N - 1)) * Math.PI;
    const px = 0.22 + 0.18 * Math.cos(a);
    const py = 0.18 * Math.sin(a);
    const rowY = cy + 160 + i * 72;
    const pt = nativeConstructPoint(ex, rowY, [px, py, 0]);
    const pl = nativePlane(ex + 120, rowY, outRef(pt.node, 'Point'), outRef(uz.node, 'Vector'));
    arcParts.push({ xml: pt.xml, node: pt.node }, { xml: pl.xml, node: pl.node });
    planeRefs.push(outRef(pl.node, 'Plane'));
  }
  const planesMerge = nativeMerge(ex + 280, cy + 280, planeRefs);

  // Plan (Walk) + Play
  const planX = ex + 440;
  const planY = cy + 200;
  const walk = motusComponent('walk', planX, planY, {
    Mechanism: [outRef(mech.node, 'Mechanism')],
    Planes: [outRef(planesMerge.node, 'Result')],
    Terrain: [outRef(ground.node, 'Mesh')],
  }, { numbers: { Lift: 0.06 } });
  const { scrub, preview } = previewWithScrub(planX, planY, outRef(walk.node, 'Trajectory'));

  const gRobot = stageGroup('Robot', [nSlider, body, leg, mech], GROUP_COLOUR.robot, rx, hy);
  const gEnv = stageGroup('Env + Traj', [
    { xml: uz.xml, node: uz.node }, groundOrigin, ground, ...arcParts, planesMerge,
  ], GROUP_COLOUR.env, ex, hy);
  const gPlan = stageGroup('Plan', [walk], GROUP_COLOUR.plan, planX, hy);
  const gPlay = stageGroup('Play', [scrub, preview], GROUP_COLOUR.play, planX + PIPE.playHeaderDx, hy);

  const objs = [
    title, note, { xml: uz.xml },
    nSlider, groundOrigin, ground, body, leg, mech,
    ...arcParts, planesMerge, walk, scrub, preview,
    gRobot.header, gEnv.header, gPlan.header, gPlay.header,
    gRobot.group, gEnv.group, gPlan.group, gPlay.group,
  ];
  objs._meta = {
    fileName,
    description,
    view: { x: 700, y: 420, zoom: 0.45 },
  };
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

/** 10 — tower destack: C# layout script + Boxes + Pick Place → one Program (20 cycles).
 * Authored/saved from Cassis (C# Script component); do not overwrite via --only=10. */
function graph10() {
  throw new Error(
    'examples/10_pick_place.ghx is Cassis-authored (C# tower layout). Save from Rhino; do not regenerate with --only=10.',
  );
}

/**
 * 11 — free-flyer HolonomicSE3 hover (Motus 2.1 foundation).
 * Drone-only: Motus Robot (Family→aerial) + Start/Goal WorldXY planes → Plan → Preview.
 * Pass-off with arm is deferred until this path is solid in Rhino.
 */
function graph11() {
  const { title, note } = exampleHeader(
    '11 · Aerial hover',
    'Free-flyer HolonomicSE3 Start→Goal. Auto Plan. bodyPose ≠ MoveJ.',
  );
  const hy = PIPE.y0;
  const cy = stageContentY(hy);
  const rx = PIPE.x0;
  const ex = rx + 360;
  const planX = ex + 280;

  const flyerPath = pathPanel(rx, cy, repoRel('assets', 'aerial', 'free_flyer_box.urdf'), 'FlyerUrdf', 240, 36);
  const drone = motusComponent('robot', rx, cy + 80, {
    Path: [outRef(flyerPath.node, 'Text')],
  }, { text: { BaseLink: 'body', TipLink: 'body' }, hidden: true });

  // WorldXY body planes (Z up) — FromPlanePlate → Motus identity; not serial Z→X remap.
  // Hidden: construction planes/points must not draw in Rhino (Preview owns the flyer).
  const uz = hidePreview(nativeUnitZ(ex, cy));
  const ptStart = hidePreview(nativeConstructPoint(ex, cy + 60, [-0.5, 0.35, 0.55]));
  const plStart = hidePreview(nativePlane(ex + 140, cy + 60, ptStart.node.outputs[0], uz.node.outputs[0]));
  const ptGoal = hidePreview(nativeConstructPoint(ex, cy + 140, [0.45, -0.25, 1.05]));
  const plGoal = hidePreview(nativePlane(ex + 140, cy + 140, ptGoal.node.outputs[0], uz.node.outputs[0]));

  const plan = motusComponent('plan', planX, cy, {
    Robot: [outRef(drone.node, 'Robot')],
    Goal: [outRef(plGoal.node, 'Plane')],
    Start: [outRef(plStart.node, 'Plane')],
  });
  const { scrub, preview } = previewWithScrub(planX, cy, outRef(plan.node, 'Trajectory'));
  const stackX = planX + PLAN_PREVIEW_DX;
  const waypoints = motusComponent('waypoints', stackX, belowPreview(cy), {
    Trajectory: [outRef(plan.node, 'Trajectory')],
  });
  const exp = motusComponent('export', stackX, belowPreview(cy) + 100, {
    Trajectory: [outRef(plan.node, 'Trajectory')],
  });

  const gRobot = stageGroup('Robot', [flyerPath, drone], GROUP_COLOUR.robot, rx, hy);
  const gEnv = stageGroup('Env + Traj', [
    { xml: uz.xml, node: uz.node }, ptStart, plStart, ptGoal, plGoal,
  ], GROUP_COLOUR.goals, ex, hy);
  const gPlan = stageGroup('Plan', [plan], GROUP_COLOUR.plan, planX, hy);
  const gPlay = stageGroup('Play', [scrub, preview, waypoints, exp], GROUP_COLOUR.play, planX + PIPE.playHeaderDx, hy);

  const objs = [
    title, note, flyerPath, drone,
    { xml: uz.xml }, { xml: ptStart.xml }, { xml: plStart.xml },
    { xml: ptGoal.xml }, { xml: plGoal.xml },
    plan, scrub, preview, waypoints, exp,
    gRobot.header, gEnv.header, gPlan.header, gPlay.header,
    gRobot.group, gEnv.group, gPlan.group, gPlay.group,
  ];
  objs._meta = {
    fileName: '11_aerial_hover.ghx',
    description:
      'Free-flyer HolonomicSE3: Motus Robot (assets/aerial/free_flyer_box.urdf) Start/Goal WorldXY planes → Plan → Preview / Export / Waypoints. Auto Plan. Motus 2.1 ADR 0006.',
    view: { x: 520, y: 220, zoom: 0.7 },
  };
  return buildGraph(objs);
}

const graphs = [graph01, graph02, graph03, graph04, graph05, graph06, graph07, graph08, graph09, graph11];
// graph10 (10_pick_place.ghx) is Cassis-authored — not in default regen list.
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

const onlyArg = process.argv.find((a) => a.startsWith('--only='));
const onlyGraph = onlyArg?.slice('--only='.length);
const onlyBuilders = {
  1: graph01, 2: graph02, 3: graph03, 4: graph04, 5: graph05,
  6: graph06, 7: graph07, 8: graph08, 9: graph09, 10: graph10, 11: graph11,
};

const buildList = onlyGraph
  ? (() => {
      const fn = onlyBuilders[onlyGraph];
      if (!fn) throw new Error(`unknown --only=${onlyGraph} (supported: ${Object.keys(onlyBuilders).join(', ')})`);
      return [fn];
    })()
  : graphs;

for (const buildFn of buildList) {
  const xml = buildFn();
  const meta = lastGraphMeta;
  if (!meta?.fileName) throw new Error(`missing meta for ${buildFn.name}`);
  const outPath = path.join(outDir, meta.fileName);
  fs.writeFileSync(outPath, xml, 'utf8');
  console.log('wrote', meta.fileName);
}

/** Authored Bounds AABB check (live chrome can still grow — Cassis verifies after load). */
function assertAuthoredOverlaps({ onlyFiles = null, warnOnly = false } = {}) {
  const files = fs.readdirSync(outDir).filter((f) => f.endsWith('.ghx')).sort();
  const scope = onlyFiles ? files.filter((f) => onlyFiles.includes(f)) : files;
  let failed = 0;
  for (const f of scope) {
    const xml = fs.readFileSync(path.join(outDir, f), 'utf8');
    const objs = [];
    for (const part of xml.split(/<chunk name="Object"/).slice(1)) {
      const nameM = part.match(/<item name="Name"[^>]*>([^<]*)<\/item>/);
      const nickM = part.match(/<item name="NickName"[^>]*>([^<]*)<\/item>/);
      const bM = part.match(/<item name="Bounds"[^>]*>\s*<X>([^<]*)<\/X>\s*<Y>([^<]*)<\/Y>\s*<W>([^<]*)<\/W>\s*<H>([^<]*)<\/H>/);
      if (!nameM || !bM || nameM[1] === 'Group') continue;
      objs.push({ name: nameM[1], nick: nickM?.[1] ?? '', x: +bM[1], y: +bM[2], w: +bM[3], h: +bM[4] });
    }
    const hits = [];
    for (let i = 0; i < objs.length; i++) {
      for (let j = i + 1; j < objs.length; j++) {
        const a = objs[i];
        const b = objs[j];
        const ix = Math.max(0, Math.min(a.x + a.w, b.x + b.w) - Math.max(a.x, b.x));
        const iy = Math.max(0, Math.min(a.y + a.h, b.y + b.h) - Math.max(a.y, b.y));
        if (ix > 2 && iy > 2) hits.push(`${a.nick || a.name}×${b.nick || b.name}(${ix | 0}x${iy | 0})`);
      }
    }
    if (hits.length) {
      failed += hits.length;
      console.error(`${warnOnly ? 'WARN' : 'OVERLAP'} ${f}:`, hits.join(' | '));
    }
  }
  if (failed && !warnOnly) throw new Error(`authored Bounds overlaps: ${failed}`);
  if (!failed || warnOnly) console.log(warnOnly ? 'authored Bounds: overlaps logged (warn-only)' : 'authored Bounds: no overlaps');
}

/**
 * SVG map of authored Bounds — inspect layout without opening Grasshopper.
 * Writes `.cassis-audit/layout/<name>.svg` (gitignored audit folder is fine).
 */
function writeLayoutSvg(fileName) {
  const auditDir = path.join(repoRoot, '.cassis-audit', 'layout');
  fs.mkdirSync(auditDir, { recursive: true });
  const xml = fs.readFileSync(path.join(outDir, fileName), 'utf8');
  const objs = [];
  for (const part of xml.split(/<chunk name="Object"/).slice(1)) {
    const nameM = part.match(/<item name="Name"[^>]*>([^<]*)<\/item>/);
    const nickM = part.match(/<item name="NickName"[^>]*>([^<]*)<\/item>/);
    const textM = part.match(/<item name="Text"[^>]*>([^<]*)<\/item>/);
    const bM = part.match(/<item name="Bounds"[^>]*>\s*<X>([^<]*)<\/X>\s*<Y>([^<]*)<\/Y>\s*<W>([^<]*)<\/W>\s*<H>([^<]*)<\/H>/);
    if (!nameM || !bM || nameM[1] === 'Group') continue;
    objs.push({
      name: nameM[1],
      nick: nickM?.[1] ?? '',
      text: textM?.[1] ?? '',
      x: +bM[1], y: +bM[2], w: +bM[3], h: +bM[4],
    });
  }
  if (!objs.length) return;
  let minX = Infinity; let minY = Infinity; let maxX = -Infinity; let maxY = -Infinity;
  for (const o of objs) {
    minX = Math.min(minX, o.x);
    minY = Math.min(minY, o.y);
    maxX = Math.max(maxX, o.x + o.w);
    maxY = Math.max(maxY, o.y + o.h);
  }
  const pad = 40;
  const W = Math.ceil(maxX - minX + pad * 2);
  const H = Math.ceil(maxY - minY + pad * 2);
  const colour = (n) => {
    if (n === 'Scribble') return '#1a1a1a';
    if (n.includes('Panel')) return '#fff59d';
    if (n.includes('Motus')) return '#b2dfdb';
    return '#e0e0e0';
  };
  const rects = objs.map((o) => {
    const x = o.x - minX + pad;
    const y = o.y - minY + pad;
    const label = o.name === 'Scribble' ? (o.text || 'Scribble') : (o.nick || o.name);
    const fs = o.name === 'Scribble' ? Math.min(18, Math.max(10, o.h * 0.55)) : 10;
    return `<g>
  <rect x="${x.toFixed(1)}" y="${y.toFixed(1)}" width="${o.w}" height="${o.h}" fill="${colour(o.name)}" stroke="#555" stroke-width="1" opacity="0.9"/>
  <text x="${(x + 3).toFixed(1)}" y="${(y + Math.min(o.h - 2, fs + 2)).toFixed(1)}" font-family="Arial,sans-serif" font-size="${fs}" fill="#111">${esc(label).slice(0, 40)}</text>
</g>`;
  }).join('\n');
  const svg = `<?xml version="1.0" encoding="UTF-8"?>
<svg xmlns="http://www.w3.org/2000/svg" width="${W}" height="${H}" viewBox="0 0 ${W} ${H}">
  <rect width="100%" height="100%" fill="#f5f5f5"/>
  <text x="${pad}" y="24" font-family="Arial,sans-serif" font-size="14" fill="#333">${esc(fileName)} — authored Bounds (no Rhino)</text>
  ${rects}
</svg>`;
  const out = path.join(auditDir, fileName.replace(/\.ghx$/, '.svg'));
  fs.writeFileSync(out, svg, 'utf8');
  console.log('layout svg', path.relative(repoRoot, out));
}

assertAuthoredOverlaps({ onlyFiles: onlyGraph ? [lastGraphMeta?.fileName].filter(Boolean) : null, warnOnly: Boolean(onlyGraph) });
const layoutFiles = onlyGraph
  ? [lastGraphMeta?.fileName].filter(Boolean)
  : fs.readdirSync(outDir).filter((f) => /^\d{2}_.*\.ghx$/.test(f) && f !== '10_pick_place.ghx');
for (const f of layoutFiles) writeLayoutSvg(f);
console.log('Done.');
