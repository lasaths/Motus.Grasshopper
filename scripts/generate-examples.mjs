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
/** Collision-free home-ish start for obstacle demos (away from table/box). */
const COLLISION_START = [0.0, -1.4, 1.4, -1.7, -1.5708, 0.0];
const COLLISION_GOAL = [1.0, -0.9, 1.0, -1.4, -1.5708, 0.3];

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
  tool: { guid: 'b7c4e2a1-9f3d-4b6e-8c1d-2a5f9e0b3d71', name: 'Motus Tool', nick: 'Tool', w: 74, h: 104,
    inputs: [
      { name: 'Name', nick: 'N', desc: 'Tool name', optional: false, text: 'gripper' },
      { name: 'TCP', nick: 'P', desc: 'TCP in flange frame (Z = tool axis)', optional: false, plane: true },
      { name: 'Geometry', nick: 'G', desc: 'Optional gripper mesh or brep (TCP-local)', optional: true },
      { name: 'GeomPlane', nick: 'L', desc: 'Geometry pose in TCP-local frame', optional: true, plane: true },
      { name: 'Capabilities', nick: 'Cap', desc: 'None or Robotiq2F85 (jaw presets for Motus Tool State)', optional: true, text: 'None' },
    ],
    outputs: [{ name: 'Tool', nick: 'Tl', desc: 'Tool definition' }] },
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
      { name: 'Robot', nick: 'Rb', desc: 'Robot model', optional: false, typeId: PTYPE.robot },
      { name: 'Goal', nick: 'G', desc: 'Targets as Planes (TCP LIN) or Joint States', optional: false, access: 1, typeId: PTYPE.generic },
      { name: 'Start', nick: 'St0', desc: 'Start as Plane (IK) or Joint State (defaults to home/zeros)', optional: true, typeId: PTYPE.generic },
      { name: 'Step', nick: 'St', desc: 'Plane goals only: TCP LIN step size (m)', optional: true, number: 0.005, typeId: PTYPE.number },
    ],
    advancedInputs: [
      { name: 'Collision', nick: 'C', desc: 'Collision scene; joint goals use RRT; plane goals validate LIN against scene', optional: true, typeId: PTYPE.colScene },
      { name: 'Group', nick: 'Gr', desc: 'Optional planning group (locks non-group joints)', optional: true, typeId: PTYPE.generic },
      { name: 'Attach', nick: 'A', desc: 'Attached bodies for collision checks', optional: true, access: 1, typeId: PTYPE.generic },
      { name: 'RrtSettings', nick: 'Rrt', desc: 'Optional RRT tuning from Motus RRT Settings (joint goals + collision only)', optional: true, typeId: PTYPE.generic },
    ],
    outputs: [
      { name: 'Trajectory', nick: 'Tr', desc: 'Planned trajectories', access: 1, typeId: PTYPE.trajectory },
      { name: 'Status', nick: 'Msg', desc: 'Planning status', typeId: PTYPE.string },
      { name: 'Warnings', nick: 'W', desc: 'Capability / validation warnings', access: 1, typeId: PTYPE.string },
    ] },
  preview: { guid: 'd4a8f1c2-3e5b-4a7d-9c1e-8f2b6d4e0a91', name: 'Motus Preview', nick: 'Preview', w: 74, h: 84,
    inputs: [
      { name: 'Trajectory', nick: 'Tr', desc: 'Motus trajectory from Motus Plan (list concatenates sequential goals)', optional: false, access: 1, typeId: PTYPE.trajectory },
      { name: 'ShowStart', nick: 'SS', desc: 'Also preview the trajectory start pose as a ghost', optional: false, bool: false, typeId: PTYPE.boolean },
      { name: 'Position', nick: 'P', desc: 'Optional normalized playback position 0–1 (Motus Scrub)', optional: true, typeId: PTYPE.number },
    ],
    outputs: [
      { name: 'Meshes', nick: 'M', desc: 'Link meshes at the current frame', access: 1, typeId: PTYPE.mesh },
      { name: 'Links', nick: 'L', desc: 'Link lines at the current frame', access: 1, typeId: PTYPE.line },
      { name: 'TCP Path', nick: 'Path', desc: 'Full TCP polyline via FK', typeId: PTYPE.curve },
      { name: 'State', nick: 'Js', desc: 'Joint state at the current frame', typeId: PTYPE.jointState },
      { name: 'Time', nick: 'Tm', desc: 'Elapsed trajectory time at current frame (seconds)', typeId: PTYPE.number },
    ] },
  export: { guid: '0a443b6f-605b-48e3-843c-cd0a709f8379', name: 'Motus Export', nick: 'Export', w: 74, h: 84,
    inputs: [
      { name: 'Trajectory', nick: 'Tr', desc: 'Motus trajectory (list concatenates sequential goals)', optional: false, access: 1 },
      { name: 'Retime', nick: 'R', desc: 'Apply bottleneck path retiming before export', optional: true, bool: true },
      { name: 'Validate', nick: 'V', desc: 'Validate limits/velocity after retiming', optional: true, bool: false },
    ],
    outputs: [
      { name: 'Json', nick: 'J', desc: 'Trajectory JSON' },
      { name: 'Csv', nick: 'C', desc: 'Trajectory CSV' },
      { name: 'Validation', nick: 'Val', desc: 'Validation summary when Validate=true', optional: true },
    ] },
  trajData: { guid: 'a72b5cfa-5cf5-4e54-a5cd-943e2aae82da', name: 'Motus Trajectory Data', nick: 'Data', w: 74, h: 84,
    inputs: [{ name: 'Trajectory', nick: 'Tr', desc: 'Motus trajectory (list concatenates sequential goals)', optional: false, access: 1 }],
    outputs: [
      { name: 'Planes', nick: 'P', desc: 'TCP plane per waypoint' },
      { name: 'Times', nick: 'Tm', desc: 'Waypoint times (seconds)' },
      { name: 'Joints', nick: 'J', desc: 'Per-axis joint values', access: 1 },
      { name: 'ToolStates', nick: 'Ts', desc: 'Tool state JSON per waypoint', optional: true },
    ] },
  rrtSettings: { guid: '11d59b15-ffe2-488e-83b8-52eddf772025', name: 'Motus RRT Settings', nick: 'RrtSet', w: 74, h: 104,
    inputs: [
      { name: 'MaxIter', nick: 'Mi', desc: 'Max sampling iterations', optional: false, number: 4000 },
      { name: 'TimeLimit', nick: 'Lim', desc: 'Wall-clock cap in seconds (0 = off)', optional: false, number: 30 },
      { name: 'Planner', nick: 'P', desc: 'Sampling planner from registry', optional: false, text: 'RrtConnect' },
      { name: 'GoalBias', nick: 'Gb', desc: 'Goal bias 0–1', optional: false, number: 0.08 },
      { name: 'Step', nick: 'St', desc: 'Tree step size (rad)', optional: false, number: 0.12 },
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
  filePath: { guid: '06953bda-1d37-4d58-9b38-4b3c74e54c8f', name: 'File Path', nick: 'Path', w: 50, h: 24 },
  move: { guid: '4f7cd4e3-9b20-41d8-9c00-2940fe7f3aa0', name: 'Move', nick: 'Move', w: 44, h: 44,
    inputs: ['Geometry', 'Motion'], outputs: ['Geometry'] },
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
                                  <items count="2">
                                    ${item('null_string', 'gh_bool', '1', 'false')}
                                    ${item('string', 'gh_string', '10', esc(text))}
                                  </items>
                                </chunk>
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
  return `<chunk name="param_input" index="${index}">
                      <items count="${6 + count + (access ? 1 : 0)}">
                        ${access}
                        ${item('Description', 'gh_string', '10', esc(def.desc ?? def.name))}
                        ${item('InstanceGuid', 'gh_guid', '9', def._guid)}
                        ${item('Name', 'gh_string', '10', def.name)}
                        ${item('NickName', 'gh_string', '10', def.nick ?? def.name)}
                        ${optional}
                        ${srcItems}
                        ${item('SourceCount', 'gh_int32', '3', String(count))}
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
    if (inp.list && options.jointValues) persistent = persistentNumbers(options.jointValues);
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
  ] : [];
  const containerItems = [
    item('Description', 'gh_string', '10', esc(spec.desc ?? spec.name)),
    item('InstanceGuid', 'gh_guid', '9', instance),
    ...moveFlags,
    item('Name', 'gh_string', '10', spec.name),
    item('NickName', 'gh_string', '10', spec.nick),
    ...planFlags,
    ...progFlags,
  ];

  let containerChunks;
  if (USE_PARAMETER_DATA.has(key)) {
    containerChunks = `${bounds(x, y, spec.w, spec.h)}
                    ${parameterDataChunk(inputs, outputs, x, y, spec.w, wireMapSafe, options)}`;
  } else {
    const inChunks = inputs.map((inp, i) => {
      const sources = (wireMapSafe[inp.name] ?? []).map((ref) => ref._guid);
      let persistent = null;
      if (inp.list && options.jointValues) persistent = persistentNumbers(options.jointValues);
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

function motusScrub(x, y, value = 0, w = MOTUS.scrub.w) {
  const spec = MOTUS.scrub;
  const instance = id();
  const h = spec.h;
  const node = { key: 'scrub', instance, outputs: [{ name: 'Number', _guid: instance }] };
  return { xml: `<chunk name="Object" index="PLACEHOLDER">
              <items count="3">
                ${item('GUID', 'gh_guid', '9', spec.guid)}
                ${item('Lib', 'gh_guid', '9', MOTUS_LIB)}
                ${item('Name', 'gh_string', '10', spec.name)}
              </items>
              <chunks count="1">
                <chunk name="Container">
                  <items count="6">
                    ${item('Description', 'gh_string', '10', 'Normalized playback position (0–1) for Motus Preview; resize wider for finer control')}
                    ${item('InstanceGuid', 'gh_guid', '9', instance)}
                    ${item('Name', 'gh_string', '10', spec.name)}
                    ${item('NickName', 'gh_string', '10', spec.nick)}
                    ${item('Optional', 'gh_bool', '1', 'false')}
                    ${item('SourceCount', 'gh_int32', '3', '0')}
                  </items>
                  <chunks count="2">
                    ${bounds(x, y, w, h)}
                    <chunk name="Slider">
                      <items count="7">
                        ${item('Digits', 'gh_int32', '3', '3')}
                        ${item('GripDisplay', 'gh_int32', '3', '1')}
                        ${item('Interval', 'gh_int32', '3', '1')}
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

function previewWithScrub(x, y, trajectoryRef, options = {}) {
  // Scrub sits above Preview (same column) so it never overlaps Motus Plan to the left.
  const scrubW = options.scrubWidth ?? 220;
  const scrub = motusScrub(x, y - 52, options.scrubValue ?? 0, scrubW);
  const previewInputs = {
    Trajectory: [trajectoryRef],
    Position: [outRef(scrub.node, 'Number')],
    ...(options.inputs ?? {}),
  };
  const preview = motusComponent('preview', x, y, previewInputs, options.preview ?? {});
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

function ur10eRobot(x, y) {
  return motusComponent('ur10e', x, y, {});
}

/** 01 — quick plan: sequential joint + TCP Pose LIN + Export / TrajData / Preview (was 01+02+12). */
function graph01() {
  const robot = ur10eRobot(40, 40);
  const start = motusComponent('joints', 40, 180, {}, { jointValues: MOTION_START });
  const goalJoint = motusComponent('joints', 40, 320, {}, { jointValues: GOAL_JOINTS });
  const tcp = motusComponent('tcpPose', 240, 220, {
    Robot: [outRef(robot.node, 'Robot')],
    State: [outRef(goalJoint.node, 'State')],
  });
  const uz = nativeUnitZ(40, 460);
  const ptLin = nativeConstructPoint(40, 520, [0.48, 0.18, 0.48]);
  const plLin = nativePlane(200, 520, ptLin.node.outputs[0], uz.node.outputs[0]);
  const plan = motusComponent('plan', 440, 200, {
    Robot: [outRef(robot.node, 'Robot')],
    Goal: [
      outRef(goalJoint.node, 'State'),
      outRef(tcp.node, 'Plane'),
      outRef(plLin.node, 'Plane'),
    ],
    Start: [outRef(start.node, 'State')],
  });
  const { scrub, preview } = previewWithScrub(720, 200, outRef(plan.node, 'Trajectory'));
  const trajData = motusComponent('trajData', 720, 360, { Trajectory: [outRef(plan.node, 'Trajectory')] });
  const exp = motusComponent('export', 720, 520, { Trajectory: [outRef(plan.node, 'Trajectory')] });
  const objs = [
    robot, start, goalJoint, tcp,
    { xml: uz.xml }, { xml: ptLin.xml }, { xml: plLin.xml },
    plan, scrub, preview, trajData, exp,
  ];
  objs._meta = {
    fileName: '01_quick_plan.ghx',
    description: 'Quick plan: sequential Joint State + TCP Pose LIN + Plane goal -> Preview / Export / Trajectory Data. Auto Plan on; drag Motus Scrub or Play.',
  };
  return buildGraph(objs);
}

/** 02 — collision RRT + shapes + SRDF/group/attach (was 03+04+05). */
function graph02() {
  const robot = ur10eRobot(40, 40);
  const start = motusComponent('joints', 40, 180, {}, { jointValues: COLLISION_START });
  const goal = motusComponent('joints', 40, 320, {}, { jointValues: COLLISION_GOAL });
  // Obstacles clear of start/goal, but sit on the straight-line mid path.
  const sphereCenter = nativeConstructPoint(40, 460, [0.45, 0.35, 0.55]);
  const sphere = motusComponent('colSphere', 220, 460, {
    Center: [outRef(sphereCenter.node, 'Point')],
  }, { text: { Name: 'block' }, numbers: { Radius: 0.12 } });
  const boxOrigin = nativeConstructPoint(40, 620, [0.70, 0.20, 0.20]);
  const uz = nativeUnitZ(40, 560);
  const boxPlane = nativePlane(200, 620, boxOrigin.node.outputs[0], uz.node.outputs[0]);
  const box = motusComponent('colBox', 380, 600, { Plane: [outRef(boxPlane.node, 'Plane')] }, {
    text: { Name: 'table' },
    numbers: { HalfX: 0.18, HalfY: 0.12, HalfZ: 0.04 },
  });
  const srdfPanel = nativePanel(-40, 780, absPath('examples/srdf/table_base.srdf'), 'Srdf', 280, 44);
  const scene = motusComponent('colScene', 580, 460, {
    Objects: [outRef(sphere.node, 'Object'), outRef(box.node, 'Object')],
    Srdf: [outRef(srdfPanel.node, 'Text')],
  });
  const group = motusComponent('group', 760, 460, { Group: [outRef(scene.node, 'Groups')] });
  // Small TCP-local grasp body (center near origin; Attach poses it at TCP).
  const graspCenter = nativeConstructPoint(40, 860, [0, 0, 0.02]);
  const grasp = motusComponent('colSphere', 220, 860, {
    Center: [outRef(graspCenter.node, 'Point')],
  }, { text: { Name: 'grasp' }, numbers: { Radius: 0.03 } });
  const attach = motusComponent('attach', 400, 860, { Object: [outRef(grasp.node, 'Object')] }, { text: { Name: 'grasp' } });
  const rrt = motusComponent('rrtSettings', 580, 660, {});
  const plan = motusComponent('plan', 960, 260, {
    Robot: [outRef(robot.node, 'Robot')],
    Goal: [outRef(goal.node, 'State')],
    Start: [outRef(start.node, 'State')],
    Collision: [outRef(scene.node, 'Scene')],
    Group: [outRef(group.node, 'Group')],
    Attach: [outRef(attach.node, 'Attach')],
    RrtSettings: [outRef(rrt.node, 'Settings')],
  });
  const { scrub, preview } = previewWithScrub(1180, 260, outRef(plan.node, 'Trajectory'));
  const objs = [
    robot, start, goal,
    { xml: sphereCenter.xml }, sphere,
    { xml: boxOrigin.xml }, { xml: uz.xml }, { xml: boxPlane.xml }, box,
    srdfPanel, scene, group,
    { xml: graspCenter.xml }, grasp, attach, rrt, plan, scrub, preview,
  ];
  objs._meta = {
    fileName: '02_collision_srdf.ghx',
    description: 'Collision RRT: ColSphere + ColBox -> ColScene (SRDF) + Group + Attach + RRT Settings -> Plan. Wire ColMesh the same way as sphere/box. Auto Plan on.',
  };
  return buildGraph(objs);
}

/** 03 — URDF load + base/tool frames + Robotiq mesh (was 06+07+09+10). */
function graph03() {
  // Bundled URDF with collision meshes (examples/ur10e/ur10e.urdf expects meshes/ beside it — use package URDF).
  const urdfFile = nativeFilePath(-220, 40, absPath('resources/robots/ur10e_robotiq/ur10e_robotiq.urdf'));
  const basePl = nativeXYPlane(-40, 180);
  const tcpPt = nativeConstructPoint(-40, 280, [0, 0, 0.1633]);
  const ux = nativeUnitX(-40, 360);
  const tcpPl = nativePlane(140, 280, tcpPt.node.outputs[0], ux.node.outputs[0]);
  const meshPath = nativeFilePath(-40, 460, absPath('resources/tools/robotiq_2f85_tcp_local.stl'), '*.stl|*.stl|All files|*.*');
  const loadMesh = motusComponent('loadMesh', 180, 440, {
    Path: [outRef(meshPath.node, 'Path')],
  });
  const tool = motusComponent('tool', 360, 300, {
    TCP: [outRef(tcpPl.node, 'Plane')],
    Geometry: [outRef(loadMesh.node, 'Mesh')],
  }, { text: { Name: 'robotiq_2f85', Capabilities: 'Robotiq2F85' } });
  const robot = motusComponent('robot', 560, 60, {
    Path: [outRef(urdfFile.node, 'Path')],
    Base: [outRef(basePl.node, 'Plane')],
    Tool: [outRef(tool.node, 'Tool')],
  }, { text: { BaseLink: 'base_link', TipLink: 'tool0' } });
  const start = motusComponent('joints', 560, 260, {}, { jointValues: START_JOINTS });
  const goal = motusComponent('joints', 560, 400, {}, { jointValues: GOAL_JOINTS });
  const plan = motusComponent('plan', 780, 220, {
    Robot: [outRef(robot.node, 'Robot')],
    Goal: [outRef(goal.node, 'State')],
    Start: [outRef(start.node, 'State')],
  });
  const { scrub, preview } = previewWithScrub(1000, 220, outRef(plan.node, 'Trajectory'), {
    preview: { bools: { ShowStart: true } },
  });
  const exp = motusComponent('export', 1000, 400, { Trajectory: [outRef(plan.node, 'Trajectory')] });
  const objs = [
    urdfFile, basePl,
    { xml: tcpPt.xml }, { xml: ux.xml }, { xml: tcpPl.xml },
    meshPath, loadMesh, tool, robot, start, goal, plan, scrub, preview, exp,
  ];
  objs._meta = {
    fileName: '03_urdf_tool_frames.ghx',
    description: 'Motus Robot URDF + Base override + Robotiq Tool (Load Mesh, Cap=Robotiq2F85) + Start + Preview ShowStart. Auto Plan on.',
  };
  return buildGraph(objs);
}

/** 04 — motion program: PTP + LIN + CIRC + SET gripper (was 08+11). */
function graph04() {
  const robot = ur10eRobot(40, 40);
  const start = motusComponent('joints', 40, 180, {}, { jointValues: MOTION_START });
  const ptpGoal = motusComponent('joints', 40, 320, {}, { jointValues: GOAL_JOINTS });
  const stateOpen = motusComponent('toolState', 40, 900, {
    Tool: [outRef(robot.node, 'Robot')],
  }, { text: { Preset: 'Open' } });
  const stateClosed = motusComponent('toolState', 200, 900, {
    Tool: [outRef(robot.node, 'Robot')],
  }, { text: { Preset: 'Closed' } });
  const segPtp = motusComponent('segment', 420, 40, {
    Goal: [outRef(ptpGoal.node, 'State')],
    ToolState: [outRef(stateOpen.node, 'State')],
  }, { text: { Type: 'PTP' } });
  const ptLin = nativeConstructPoint(40, 520, [0.45, 0.15, 0.45]);
  const uz = nativeUnitZ(40, 460);
  const plLin = nativePlane(200, 520, ptLin.node.outputs[0], uz.node.outputs[0]);
  const segLin = motusComponent('segment', 420, 220, {
    Goal: [outRef(plLin.node, 'Plane')],
  }, { text: { Type: 'LIN' } });
  const ptVia = nativeConstructPoint(40, 680, [0.453, 0.152, 0.45]);
  const ptGoal = nativeConstructPoint(40, 780, [0.45, 0.154, 0.45]);
  const plVia = nativePlane(200, 680, ptVia.node.outputs[0], uz.node.outputs[0]);
  const plGoal = nativePlane(200, 780, ptGoal.node.outputs[0], uz.node.outputs[0]);
  const segCirc = motusComponent('segment', 420, 420, {
    Goal: [outRef(plGoal.node, 'Plane')],
    Via: [outRef(plVia.node, 'Plane')],
  }, { text: { Type: 'CIRC' } });
  const segSet = motusComponent('segment', 420, 640, {
    ToolState: [outRef(stateClosed.node, 'State')],
  }, { text: { Type: 'SET' }, numbers: { Duration: 0.2 } });
  const progPlan = motusComponent('progPlan', 640, 280, {
    Robot: [outRef(robot.node, 'Robot')],
    Segments: [
      outRef(segPtp.node, 'Segment'),
      outRef(segLin.node, 'Segment'),
      outRef(segCirc.node, 'Segment'),
      outRef(segSet.node, 'Segment'),
    ],
    Start: [outRef(start.node, 'State')],
  });
  const { scrub, preview } = previewWithScrub(880, 280, outRef(progPlan.node, 'Trajectory'));
  const exp = motusComponent('export', 880, 460, { Trajectory: [outRef(progPlan.node, 'Trajectory')] });
  const flat = [
    robot, start, ptpGoal, stateOpen, stateClosed, segPtp,
    { xml: ptLin.xml }, { xml: uz.xml }, { xml: plLin.xml },
    segLin,
    { xml: ptVia.xml }, { xml: ptGoal.xml }, { xml: plVia.xml }, { xml: plGoal.xml },
    segCirc, segSet, progPlan, scrub, preview, exp,
  ];
  flat._meta = {
    fileName: '04_motion_program.ghx',
    description: 'Motion program: PTP + LIN + CIRC + SET gripper -> Motus Program -> Preview / Export. Auto Plan on; drag Motus Scrub or Play.',
  };
  return buildGraph(flat);
}

const graphs = [graph01, graph02, graph03, graph04];
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
