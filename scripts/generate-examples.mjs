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
const outDir = path.resolve(__dirname, '../examples');
const MOTUS_LIB = 'dc547e55-81a8-c313-e25d-e1468ddecddb';
const csproj = fs.readFileSync(path.resolve(__dirname, '../src/Motus.GH/Motus.GH.csproj'), 'utf8');
const props = fs.readFileSync(path.resolve(__dirname, '../build/MotusNetPackages.props'), 'utf8');
const MOTUS_NET_VERSION = props.match(/<MotusNetVersion[^>]*>([^<]+)<\/MotusNetVersion>/)?.[1]?.trim() ?? '0.6.6';
const PLUGIN_VERSION = csproj.match(/<Version>([^<]+)<\/Version>/)?.[1] ?? MOTUS_NET_VERSION;
const PLUGIN_ASSEMBLY_VERSION = `${PLUGIN_VERSION}.0`;

const GOAL_JOINTS = [1.2, -1, 1.2, -1.6, -1.5708, 0];
const START_JOINTS = [0, -1.2, 1.2, -1.6, -1.5708, 0];
const MOTION_START = [0, -0.5, 1.0, -1.0, 0.0, 0.0];
const UR10E_START_DEG = '0\n-90\n0\n-90\n0\n0';
const UR10E_GOAL_DEG = '0\n-80\n0\n-90\n0\n0';
const BUNDLED_URDF = 'resources/robots/ur10e_robotiq/ur10e_robotiq.urdf';

const MOTUS = {
  robot: { guid: 'aa3e8488-943e-426f-b205-e8db5f684998', name: 'Motus Robot', nick: 'Robot', w: 74, h: 104,
    inputs: [
      { name: 'Path', nick: 'P', desc: 'Path to .urdf or .xacro file', optional: false, text: '' },
      { name: 'BaseLink', nick: 'B', desc: 'Base link name', optional: true, text: 'base_link' },
      { name: 'TipLink', nick: 'T', desc: 'Tip link name', optional: true, text: 'tool0' },
      { name: 'Base', nick: 'Bf', desc: 'Optional base frame override (TCP goals are in this frame)', optional: true, plane: true },
      { name: 'Tool', nick: 'Tl', desc: 'Optional Motus Tool definition', optional: true },
    ],
    outputs: [{ name: 'Robot', nick: 'Rb', desc: 'Robot model with URDF kinematics chain' }] },
  ur10e: { guid: '84b06a7d-8a3d-46ec-968f-25e74c249ad1', name: 'Motus UR10e Robotiq', nick: 'UR10e', w: 74, h: 44,
    inputs: [],
    outputs: [{ name: 'Robot', nick: 'Rb', desc: 'Robot model with URDF kinematics chain' }] },
  tool: { guid: 'b7c4e2a1-9f3d-4b6e-8c1d-2a5f9e0b3d71', name: 'Motus Tool', nick: 'Tool', w: 74, h: 84,
    inputs: [
      { name: 'Name', nick: 'N', desc: 'Tool name', optional: false, text: 'gripper' },
      { name: 'TCP', nick: 'P', desc: 'TCP in flange frame (Z = tool axis)', optional: false, plane: true },
      { name: 'Geometry', nick: 'G', desc: 'Optional gripper mesh or brep (TCP-local)', optional: true },
      { name: 'GeomPlane', nick: 'L', desc: 'Geometry pose in TCP-local frame', optional: true, plane: true },
    ],
    outputs: [{ name: 'Tool', nick: 'T', desc: 'Tool definition' }] },
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
    outputs: [{ name: 'State', nick: 'S', desc: 'Joint state' }] },
  tcpPose: { guid: 'f1a2b3c4-d5e6-4789-a123-4567890abcde', name: 'Motus TCP Pose', nick: 'TCP', w: 65, h: 44,
    inputs: [
      { name: 'Robot', nick: 'Rb', desc: 'Robot model', optional: false },
      { name: 'State', nick: 'S', desc: 'Joint state', optional: false },
    ],
    outputs: [{ name: 'Plane', nick: 'P', desc: 'TCP pose in robot base frame (position + orientation)' }] },
  plan: { guid: '8bb0bae3-527f-4e80-a8a4-c8a88b7276de', name: 'Motus Plan', nick: 'Plan', w: 74, h: 172,
    desc: 'Plan to plane/joint goal; optional Collision, Group, Attach, and RrtSettings (click Plan or enable Auto Plan)',
    inputs: [
      { name: 'Robot', nick: 'Rb', desc: 'Robot model', optional: false },
      { name: 'Goal', nick: 'G', desc: 'Targets as Planes (TCP LIN) or Joint States', optional: false, access: 1 },
      { name: 'Start', nick: 'S', desc: 'Optional start joint state (defaults to home)', optional: true },
      { name: 'Step', nick: 'St', desc: 'Plane goals only: TCP LIN step size (m)', optional: true, number: 0.005 },
      { name: 'Collision', nick: 'C', desc: 'Collision scene; joint goals use RRT; plane goals validate LIN against scene', optional: true },
      { name: 'Group', nick: 'Gr', desc: 'Optional planning group (locks non-group joints)', optional: true },
      { name: 'Attach', nick: 'A', desc: 'Attached bodies for collision checks', optional: true, access: 1 },
      { name: 'RrtSettings', nick: 'Rrt', desc: 'Optional RRT tuning from Motus RRT Settings (joint goals + collision only)', optional: true },
    ],
    outputs: [
      { name: 'Trajectory', nick: 'T', desc: 'Planned trajectories' },
      { name: 'Status', nick: 'St', desc: 'Planning status' },
      { name: 'Warnings', nick: 'W', desc: 'Capability / validation warnings' },
    ] },
  preview: { guid: 'd4a8f1c2-3e5b-4a7d-9c1e-8f2b6d4e0a91', name: 'Motus Preview', nick: 'Preview', w: 74, h: 104,
    inputs: [
      { name: 'Trajectory', nick: 'T', desc: 'Motus trajectory from Motus Plan', optional: false },
      { name: 'ShowStart', nick: 'S', desc: 'Also preview the trajectory start pose as a ghost', optional: false, bool: false },
      { name: 'Position', nick: 'P', desc: 'Optional normalized playback position 0–1 (Motus Scrub)', optional: true },
    ],
    outputs: [
      { name: 'Meshes', nick: 'M', desc: 'Link meshes at the current frame' },
      { name: 'Links', nick: 'L', desc: 'Link lines at the current frame' },
      { name: 'TCP Path', nick: 'P', desc: 'Full TCP polyline via FK' },
      { name: 'State', nick: 'S', desc: 'Joint state at the current frame' },
      { name: 'Time', nick: 'Tm', desc: 'Elapsed trajectory time at current frame (seconds)' },
      { name: 'Index', nick: 'I', desc: 'Current waypoint index (0-based)' },
      { name: 'Invalid', nick: 'X', desc: 'Invalid TCP segments (joint/velocity/acceleration limits)', access: 1 },
      { name: 'ToolState', nick: 'Ts', desc: 'Tool state at the current frame', optional: true },
      { name: 'Width', nick: 'W', desc: 'Gripper width (m) at playhead when present', optional: true },
    ] },
  export: { guid: '0a443b6f-605b-48e3-843c-cd0a709f8379', name: 'Motus Export', nick: 'Export', w: 74, h: 84,
    inputs: [
      { name: 'Trajectory', nick: 'T', desc: 'Motus trajectory', optional: false },
      { name: 'Retime', nick: 'R', desc: 'Apply bottleneck path retiming before export', optional: true, bool: true },
      { name: 'Validate', nick: 'V', desc: 'Validate limits/velocity after retiming', optional: true, bool: false },
    ],
    outputs: [
      { name: 'Json', nick: 'J', desc: 'Trajectory JSON' },
      { name: 'Csv', nick: 'C', desc: 'Trajectory CSV' },
      { name: 'Validation', nick: 'Val', desc: 'Validation summary when Validate=true', optional: true },
    ] },
  trajData: { guid: 'a72b5cfa-5cf5-4e54-a5cd-943e2aae82da', name: 'Motus Trajectory Data', nick: 'Data', w: 74, h: 84,
    inputs: [{ name: 'Trajectory', nick: 'T', desc: 'Motus trajectory', optional: false }],
    outputs: [
      { name: 'Planes', nick: 'P', desc: 'TCP plane per waypoint' },
      { name: 'Times', nick: 'Tm', desc: 'Waypoint times (seconds)' },
      { name: 'Joints', nick: 'J', desc: 'Per-axis joint values', access: 1 },
      { name: 'ToolStates', nick: 'Ts', desc: 'Tool state JSON per waypoint', optional: true },
    ] },
  rrtSettings: { guid: '11d59b15-ffe2-488e-83b8-52eddf772025', name: 'Motus RRT Settings', nick: 'RrtSet', w: 74, h: 104,
    inputs: [
      { name: 'MaxIter', nick: 'Mi', desc: 'Max sampling iterations', optional: false, number: 4000 },
      { name: 'TimeLimit', nick: 'T', desc: 'Wall-clock cap in seconds (0 = off)', optional: false, number: 30 },
      { name: 'Planner', nick: 'P', desc: 'Sampling planner from registry', optional: false, text: 'RrtConnect' },
      { name: 'GoalBias', nick: 'Gb', desc: 'Goal bias 0–1', optional: false, number: 0.08 },
      { name: 'Step', nick: 'St', desc: 'Tree step size (rad)', optional: false, number: 0.12 },
    ],
    outputs: [{ name: 'Settings', nick: 'S', desc: 'Sampling planner settings for Motus Plan' }] },
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
      { name: 'Scene', nick: 'S', desc: 'Collision scene' },
      { name: 'Groups', nick: 'G', desc: 'Planning groups from SRDF (optional)', access: 1 },
      { name: 'EndEffectors', nick: 'EE', desc: 'End-effector map from SRDF as name=parent_link entries', access: 1 },
    ] },
  group: { guid: '91e2a9db-cfb4-4a6c-99a3-305ba27fdf1e', name: 'Motus Planning Group', nick: 'Group', w: 74, h: 84,
    inputs: [
      { name: 'Group', nick: 'G', desc: 'Optional existing planning group (e.g. from ColScene SRDF output)', optional: true },
      { name: 'Name', nick: 'N', desc: 'Group name', optional: true, text: 'manipulator' },
      { name: 'BaseLink', nick: 'B', desc: 'Base link name', optional: true, text: 'base_link' },
      { name: 'TipLink', nick: 'T', desc: 'Tip link name', optional: true, text: 'tool0' },
      { name: 'Joints', nick: 'J', desc: 'Joint names (leave empty to use base..tip shorthand)', optional: true, access: 1 },
    ],
    outputs: [{ name: 'Group', nick: 'G', desc: 'Planning group' }] },
  attach: { guid: '0c464ac8-0e1d-4c7a-9c8c-0a21f1046314', name: 'Motus Attach Body', nick: 'Attach', w: 74, h: 74,
    inputs: [
      { name: 'Object', nick: 'O', desc: 'Collision object geometry to attach', optional: false },
      { name: 'Name', nick: 'N', desc: 'Attached body name', optional: true, text: 'grasp' },
      { name: 'TcpLocal', nick: 'P', desc: 'TCP-local pose of attached geometry', optional: true, plane: true },
      { name: 'SourceName', nick: 'S', desc: 'Optional scene object name to hide while attached', optional: true, text: '' },
    ],
    outputs: [{ name: 'Attach', nick: 'A', desc: 'Attached body' }] },
  toolState: { guid: 'a1b2c3d4-e5f6-7890-abcd-ef1234567890', name: 'Motus Tool State', nick: 'ToolState', w: 74, h: 84,
    inputs: [
      { name: 'Tool', nick: 'T', desc: 'Optional tool for validation and presets', optional: true },
      { name: 'Preset', nick: 'P', desc: 'Open, Closed, or Custom', optional: false, text: 'Open' },
      { name: 'Width', nick: 'W', desc: 'Jaw width (m) when Preset=Custom', optional: true, number: 0.085 },
      { name: 'Speed', nick: 'Sp', desc: 'Grip speed ratio 0–1', optional: true, number: 0.5 },
      { name: 'Force', nick: 'F', desc: 'Grip force ratio 0–1', optional: true, number: 0.5 },
    ],
    outputs: [{ name: 'State', nick: 'S', desc: 'End-effector state' }] },
  segment: { guid: '7c4e9a2f-1b3d-4e8a-9f6c-2d8b5a7e9c31', name: 'Motus Motion Segment', nick: 'Segment', w: 74, h: 144,
    desc: 'Build PTP/LIN/CIRC/SET/WAIT motion segment (Type dropdown)',
    inputs: [
      { name: 'Type', nick: 'T', desc: 'PTP, LIN, CIRC, SET, or WAIT', optional: false, text: 'PTP' },
      { name: 'Goal', nick: 'G', desc: 'PTP: Joint State; LIN/CIRC: Plane (TCP pose)', optional: true },
      { name: 'Via', nick: 'V', desc: 'CIRC only: arc via point (TCP plane)', optional: true },
      { name: 'Step', nick: 'St', desc: 'LIN only: TCP step size (m)', optional: true, number: 0.005 },
      { name: 'Samples', nick: 'N', desc: 'CIRC only: arc samples (>= 4)', optional: true, number: 16 },
      { name: 'Blend', nick: 'B', desc: 'Blend radius (m, default 0)', optional: true, number: 0 },
      { name: 'ToolState', nick: 'Ts', desc: 'Optional tool state goal', optional: true },
      { name: 'ToolMode', nick: 'Tm', desc: 'Hold, Ramp, or Instant (arm segments)', optional: true, text: 'Hold' },
      { name: 'Duration', nick: 'D', desc: 'SET/WAIT duration (s)', optional: true, number: 0 },
    ],
    outputs: [{ name: 'Segment', nick: 'S', desc: 'Motion segment' }] },
  progPlan: { guid: '8d5f0b3e-2c4e-4f9b-0a7d-3e9c6b8f0d42', name: 'Motus Program Plan', nick: 'ProgPlan', w: 74, h: 144,
    desc: 'Plan mixed PTP/LIN/CIRC program (click Plan); LIN failures do not fall back to joint paths',
    inputs: [
      { name: 'Robot', nick: 'Rb', desc: 'Robot model', optional: false },
      { name: 'Segments', nick: 'Seg', desc: 'List of motion segments', optional: false, access: 1 },
      { name: 'Start', nick: 'S', desc: 'Start joint state (defaults to home)', optional: true },
      { name: 'Collision', nick: 'C', desc: 'Collision scene', optional: true },
      { name: 'Group', nick: 'Gr', desc: 'Optional planning group (locks non-group joints)', optional: true },
      { name: 'Attach', nick: 'A', desc: 'Optional attached bodies list', optional: true, access: 1 },
    ],
    outputs: [
      { name: 'Trajectory', nick: 'T', desc: 'Planned trajectory' },
      { name: 'Status', nick: 'St', desc: 'Planning status' },
      { name: 'Warnings', nick: 'W', desc: 'Capability / validation warnings' },
    ] },
  scrub: { guid: 'e1f2a3b4-c5d6-4789-a012-3456789abc01', name: 'Motus Scrub', nick: 'Scrub', w: 220, h: 44 },
};

const NATIVE = {
  valueList: { guid: '00027467-0d24-4fa7-b178-8dc0ac5f42ec', name: 'Value List', nick: 'Model', w: 163, h: 22, outputs: ['Value'] },
  panel: { guid: '59e0b89a-e487-49f8-bab8-b5bab16be14c', name: 'Panel', w: 160, h: 60 },
  constructPoint: { guid: '2e78b80c-5c6e-4dcc-9b49-4a7cde34af52', name: 'Construct Point', nick: 'Pt', w: 44, h: 44,
    inputs: ['X', 'Y', 'Z'], outputs: ['Point'] },
  unitZ: { guid: '1bbd9fdd-9aea-4f6e-92af-2311b107d8c9', name: 'Unit Z', nick: 'Z', w: 44, h: 22, outputs: ['Vector'] },
  unitX: { guid: '7e6bff32-67ee-4da3-b9ab-7c1921b5f1d4', name: 'Unit X', nick: 'X', w: 44, h: 22, outputs: ['Vector'] },
  plane: { guid: '35abd9fb-7453-458e-9c86-8bf9f94becc8', name: 'Plane', nick: 'Pln', w: 44, h: 44,
    inputs: ['Origin', 'Normal'], outputs: ['Plane'] },
  xyPlane: { guid: '1bbf3dec-0ddd-49b8-aba3-ddde6bb8de4e', name: 'XY Plane', nick: 'XY', w: 44, h: 22, outputs: ['Plane'] },
  filePath: { guid: '06953bda-1d37-4d58-9b38-4b3c74e54c8f', name: 'File Path', nick: 'Path', w: 50, h: 24 },
  move: { guid: '4f7cd4e3-9b20-41d8-9c00-2940fe7f3aa0', name: 'Move', nick: 'Move', w: 44, h: 44,
    inputs: ['Geometry', 'Motion'], outputs: ['Geometry'] },
  vectorZ: { guid: '2a1b5b5079e4413c8c48c6e2fb4c1a42', name: 'Unit Z', nick: 'Z', w: 44, h: 22, outputs: ['Vector'] },
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

function motusComponent(key, x, y, wireMap, options = {}) {
  const spec = structuredClone(MOTUS[key]);
  const instance = id();
  const inputs = spec.inputs.map((inp) => {
    const copy = { ...inp, _guid: id() };
    if (options.numbers?.[inp.name] !== undefined) copy.number = options.numbers[inp.name];
    if (options.points?.[inp.name] !== undefined) copy.point = options.points[inp.name];
    if (options.text?.[inp.name] !== undefined) copy.text = options.text[inp.name];
    if (options.bools?.[inp.name] !== undefined) copy.bool = options.bools[inp.name];
    if (options.angle?.[inp.name] !== undefined) copy.angle = true;
    if (options.useDegrees?.[inp.name] !== undefined) copy.useDegrees = options.useDegrees[inp.name];
    return copy;
  });
  const outputs = spec.outputs.map((out) => ({ ...out, _guid: id() }));
  const inChunks = inputs.map((inp, i) => {
    const sources = (wireMap[inp.name] ?? []).map((ref) => ref._guid);
    let persistent = null;
    if (inp.list && options.jointValues) persistent = persistentNumbers(options.jointValues);
    else if (inp.bool !== undefined && !sources.length) persistent = persistentBool(inp.bool);
    else if (inp.number !== undefined && !sources.length) persistent = persistentNumbers([inp.number]);
    else if (inp.text !== undefined && !sources.length) persistent = persistentText(inp.text);
    else if (inp.point && !sources.length) persistent = persistentNumbers(inp.point);
    return paramInput(inp, i, x, y, spec.w, sources, persistent);
  });
  const outChunks = outputs.map((out, i) => paramOutput(out, i, x, y, spec.w));
  const node = { key, instance, inputs, outputs, spec };
  return { xml: `<chunk name="Object" index="PLACEHOLDER">
              <items count="3">
                ${item('GUID', 'gh_guid', '9', spec.guid)}
                ${item('Lib', 'gh_guid', '9', MOTUS_LIB)}
                ${item('Name', 'gh_string', '10', spec.name)}
              </items>
              <chunks count="1">
                <chunk name="Container">
                  <items count="4">
                    ${item('Description', 'gh_string', '10', esc(spec.desc ?? spec.name))}
                    ${item('InstanceGuid', 'gh_guid', '9', instance)}
                    ${item('Name', 'gh_string', '10', spec.name)}
                    ${item('NickName', 'gh_string', '10', spec.nick)}
                  </items>
                  <chunks count="${1 + inputs.length + outputs.length}">
                    ${bounds(x, y, spec.w, spec.h)}
                    ${inChunks.join('\n                    ')}
                    ${outChunks.join('\n                    ')}
                  </chunks>
                </chunk>
              </chunks>
            </chunk>`, node };
}

function valueList(x, y, selected = 'UR10e') {
  const instance = id();
  // Value List is a native GH_Param: InstanceGuid is the wire target (no param_output chunk).
  const items = MODELS.map((name, i) => `<chunk name="ListItem" index="${i}">
                      <items count="3">
                        ${item('Expression', 'gh_string', '10', `"${name}"`)}
                        ${item('Name', 'gh_string', '10', name)}
                        ${item('Selected', 'gh_bool', '1', name === selected ? 'true' : 'false')}
                      </items>
                    </chunk>`).join('\n                    ');
  const node = { key: 'valueList', instance, outputs: [{ name: 'Value', _guid: instance }] };
  return { xml: `<chunk name="Object" index="PLACEHOLDER">
              <items count="2">
                ${item('GUID', 'gh_guid', '9', NATIVE.valueList.guid)}
                ${item('Name', 'gh_string', '10', 'Value List')}
              </items>
              <chunks count="1">
                <chunk name="Container">
                  <items count="8">
                    ${item('Description', 'gh_string', '10', 'Provides a list of preset values to choose from')}
                    ${item('InstanceGuid', 'gh_guid', '9', instance)}
                    ${item('ListCount', 'gh_int32', '3', String(MODELS.length))}
                    ${item('ListMode', 'gh_int32', '3', '1')}
                    ${item('Name', 'gh_string', '10', 'Value List')}
                    ${item('NickName', 'gh_string', '10', 'Model')}
                    ${item('Optional', 'gh_bool', '1', 'false')}
                    ${item('SourceCount', 'gh_int32', '3', '0')}
                  </items>
                  <chunks count="${MODELS.length + 1}">
                    ${items}
                    ${bounds(x, y, NATIVE.valueList.w, NATIVE.valueList.h)}
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
  const scrub = motusScrub(x - 260, y + 10, options.scrubValue ?? 0, options.scrubWidth ?? 280);
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
                ${item('Name', 'gh_string', '10', 'Plane')}
              </items>
              <chunks count="1">
                <chunk name="Container">
                  <items count="5">
                    ${item('Description', 'gh_string', '10', 'Create a plane from origin and normal')}
                    ${item('InstanceGuid', 'gh_guid', '9', instance)}
                    ${item('Name', 'gh_string', '10', 'Plane')}
                    ${item('NickName', 'gh_string', '10', 'Goal')}
                    ${item('SourceCount', 'gh_int32', '3', '0')}
                  </items>
                  <chunks count="4">
                    ${bounds(x, y, 44, 44)}
                    <chunk name="param_input" index="0">
                      <items count="7">
                        ${item('Description', 'gh_string', '10', 'Plane origin')}
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
                        ${item('Description', 'gh_string', '10', 'Plane normal')}
                        ${item('InstanceGuid', 'gh_guid', '9', inNormal)}
                        ${item('Name', 'gh_string', '10', 'Normal')}
                        ${item('NickName', 'gh_string', '10', 'N')}
                        ${item('Optional', 'gh_bool', '1', 'false')}
                        ${sourceItem(0, normalRef._guid)}
                        ${item('SourceCount', 'gh_int32', '3', '1')}
                      </items>
                      <chunks count="1">${bounds(x + 2, y + 18, 15, 14)}</chunks>
                    </chunk>
                    <chunk name="param_output" index="0">
                      <items count="6">
                        ${item('Description', 'gh_string', '10', 'Plane')}
                        ${item('InstanceGuid', 'gh_guid', '9', outGuid)}
                        ${item('Name', 'gh_string', '10', 'Plane')}
                        ${item('NickName', 'gh_string', '10', 'Pl')}
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

function graph01() {
  const robot = ur10eRobot(140, 60);
  const joints = motusComponent('joints', 140, 220, {}, { jointValues: GOAL_JOINTS });
  const plan = motusComponent('plan', 420, 140, {
    Robot: [outRef(robot.node, 'Robot')],
    Goal: [outRef(joints.node, 'State')],
  });
  const { scrub, preview } = previewWithScrub(620, 120, outRef(plan.node, 'Trajectory'));
  const trajData = motusComponent('trajData', 620, 260, { Trajectory: [outRef(plan.node, 'Trajectory')] });
  const exp = motusComponent('export', 620, 380, { Trajectory: [outRef(plan.node, 'Trajectory')] });
  const objs = [robot, joints, plan, scrub, preview, trajData, exp];
  objs._meta = {
    fileName: '01_joint_planning.ghx',
    description: 'Joint-linear planning: UR10e Robotiq + Joint State -> Plan -> Preview / Export / Trajectory Data. Click Plan, then drag Motus Scrub or Play.',
  };
  return buildGraph(objs);
}

function graph02() {
  const robot = ur10eRobot(140, 60);
  const joints = motusComponent('joints', 140, 220, {}, { jointValues: GOAL_JOINTS });
  const tcp = motusComponent('tcpPose', 300, 140, {
    Robot: [outRef(robot.node, 'Robot')],
    State: [outRef(joints.node, 'State')],
  });
  const plan = motusComponent('plan', 480, 140, {
    Robot: [outRef(robot.node, 'Robot')],
    Goal: [outRef(tcp.node, 'Plane')],
  });
  const { scrub, preview } = previewWithScrub(680, 140, outRef(plan.node, 'Trajectory'));
  const objs = [robot, joints, tcp, plan, scrub, preview];
  objs._meta = {
    fileName: '02_cartesian_planning.ghx',
    description: 'Cartesian TCP LIN: Joint State -> TCP Pose (FK plane) -> Plan. Click Plan, then drag Motus Scrub or Play on Preview.',
  };
  return buildGraph(objs);
}

function graph03() {
  const robot = ur10eRobot(140, 60);
  const joints = motusComponent('joints', 140, 220, {}, { jointValues: GOAL_JOINTS });
  const sphere = motusComponent('colSphere', 140, 380, {});
  const scene = motusComponent('colScene', 300, 380, { Objects: [outRef(sphere.node, 'Object')] });
  const rrt = motusComponent('rrtSettings', 300, 500, {});
  const plan = motusComponent('plan', 480, 180, {
    Robot: [outRef(robot.node, 'Robot')],
    Goal: [outRef(joints.node, 'State')],
    Collision: [outRef(scene.node, 'Scene')],
    RrtSettings: [outRef(rrt.node, 'Settings')],
  });
  const { scrub, preview } = previewWithScrub(680, 160, outRef(plan.node, 'Trajectory'));
  const objs = [robot, joints, sphere, scene, rrt, plan, scrub, preview];
  objs._meta = {
    fileName: '03_collision_rrt.ghx',
    description: 'Collision-aware RRT: ColSphere -> ColScene -> Plan.Collision with Motus RRT Settings on joint goal. Click Plan, then drag Motus Scrub or Play.',
  };
  return buildGraph(objs);
}

function graph04() {
  const robot = ur10eRobot(140, 60);
  const joints = motusComponent('joints', 140, 200, {}, { jointValues: GOAL_JOINTS });
  const sphere = motusComponent('colSphere', 140, 360, {}, { text: { Name: 'sphere' } });
  const xy = nativeXYPlane(140, 480);
  const box = motusComponent('colBox', 300, 460, { Plane: [outRef(xy.node, 'Plane')] });
  const scene = motusComponent('colScene', 460, 400, {
    Objects: [outRef(sphere.node, 'Object'), outRef(box.node, 'Object')],
  });
  const plan = motusComponent('plan', 620, 180, {
    Robot: [outRef(robot.node, 'Robot')],
    Goal: [outRef(joints.node, 'State')],
    Collision: [outRef(scene.node, 'Scene')],
  });
  const { scrub, preview } = previewWithScrub(820, 160, outRef(plan.node, 'Trajectory'));
  const objs = [robot, joints, sphere, xy, box, scene, plan, scrub, preview];
  objs._meta = {
    fileName: '04_collision_shapes.ghx',
    description: 'Multiple obstacle shapes: ColSphere + ColBox -> ColScene -> Plan (RRT). Wire your own mesh into ColMesh the same way.',
  };
  return buildGraph(objs);
}

function graph05() {
  const robot = ur10eRobot(140, 60);
  const joints = motusComponent('joints', 140, 200, {}, { jointValues: GOAL_JOINTS });
  const tableCenter = nativeConstructPoint(140, 340, [0.35, 0.15, 0.35]);
  const sphere = motusComponent('colSphere', 140, 360, {
    Center: [outRef(tableCenter.node, 'Point')],
  }, { text: { Name: 'table' } });
  const srdfPanel = nativePanel(-55, 410, 'examples/srdf/table_base.srdf', 'Srdf', 220, 44);
  const scene = motusComponent('colScene', 300, 360, {
    Objects: [outRef(sphere.node, 'Object')],
    Srdf: [outRef(srdfPanel.node, 'Text')],
  });
  const group = motusComponent('group', 460, 360, { Group: [outRef(scene.node, 'Groups')] });
  const graspCenter = nativeConstructPoint(140, 500, [0, 0, 0.08]);
  const grasp = motusComponent('colSphere', 140, 520, {
    Center: [outRef(graspCenter.node, 'Point')],
  }, {
    text: { Name: 'grasp' },
    numbers: { Radius: 0.05 },
  });
  const attach = motusComponent('attach', 300, 520, { Object: [outRef(grasp.node, 'Object')] }, { text: { Name: 'grasp' } });
  const plan = motusComponent('plan', 620, 200, {
    Robot: [outRef(robot.node, 'Robot')],
    Goal: [outRef(joints.node, 'State')],
    Collision: [outRef(scene.node, 'Scene')],
    Group: [outRef(group.node, 'Group')],
    Attach: [outRef(attach.node, 'Attach')],
  });
  const { scrub, preview } = previewWithScrub(820, 180, outRef(plan.node, 'Trajectory'));
  const objs = [robot, joints, { xml: tableCenter.xml }, sphere, srdfPanel, scene, group, { xml: graspCenter.xml }, grasp, attach, plan, scrub, preview];
  objs._meta = {
    fileName: '05_srdf_group_attach.ghx',
    description: 'SRDF allowed pairs + Planning Group + Attach Body on Plan. Set Srdf panel to your absolute path if needed.',
  };
  return buildGraph(objs);
}

function graph06() {
  const urdfFile = nativeFilePath(40, 80, BUNDLED_URDF);
  const robot = motusComponent('robot', 147, 80, {
    Path: [outRef(urdfFile.node, 'Path')],
  }, { text: { BaseLink: 'base_link', TipLink: 'tool0' } });
  const goalPanel = nativePanel(35, 246, UR10E_GOAL_DEG, 'Goal°', 160, 100);
  const startPanel = nativePanel(148, 383, UR10E_START_DEG, 'Start°', 160, 100);
  const goalJoints = motusComponent('joints', 214, 202, {
    Joints: [outRef(goalPanel.node, 'Text')],
  }, { useDegrees: { Joints: true } });
  const startJoints = motusComponent('joints', 292, 304, {
    Joints: [outRef(startPanel.node, 'Text')],
  }, { useDegrees: { Joints: true } });
  const plan = motusComponent('plan', 416, 174, {
    Robot: [outRef(robot.node, 'Robot')],
    Goal: [outRef(goalJoints.node, 'State')],
    Start: [outRef(startJoints.node, 'State')],
  });
  const { scrub, preview } = previewWithScrub(618, 110, outRef(plan.node, 'Trajectory'));
  const objs = [robot, plan, scrub, preview, urdfFile, startJoints, goalPanel, goalJoints, startPanel];
  objs._meta = {
    fileName: '06_urdf_load.ghx',
    description: 'Motus Robot URDF load: bundled ur10e_robotiq.urdf path + explicit Start on Plan.',
  };
  return buildGraph(objs);
}

function graph07() {
  const urdfFile = nativeFilePath(-180, 60, BUNDLED_URDF);
  const basePl = nativeXYPlane(40, 200);
  const tcpPl = nativeXYPlane(40, 280);
  const tool = motusComponent('tool', 160, 200, { TCP: [outRef(tcpPl.node, 'Plane')] }, { text: { Name: 'offset' } });
  const robot = motusComponent('robot', 340, 80, {
    Path: [outRef(urdfFile.node, 'Path')],
    Base: [outRef(basePl.node, 'Plane')],
    Tool: [outRef(tool.node, 'Tool')],
  }, { text: { BaseLink: 'base_link', TipLink: 'tool0' } });
  const start = motusComponent('joints', 340, 220, {}, { jointValues: START_JOINTS });
  const goal = motusComponent('joints', 340, 340, {}, { jointValues: GOAL_JOINTS });
  const plan = motusComponent('plan', 560, 200, {
    Robot: [outRef(robot.node, 'Robot')],
    Goal: [outRef(goal.node, 'State')],
    Start: [outRef(start.node, 'State')],
  });
  const { scrub, preview } = previewWithScrub(760, 180, outRef(plan.node, 'Trajectory'), {
    preview: { bools: { ShowStart: true } },
  });
  const objs = [urdfFile, basePl, tcpPl, tool, robot, start, goal, plan, scrub, preview];
  objs._meta = {
    fileName: '07_frames_and_start.ghx',
    description: 'Motus Robot: URDF path + Base override + Motus Tool, explicit Start on Plan, ShowStart ghost on Preview.',
  };
  return buildGraph(objs);
}

function graph08() {
  const robot = ur10eRobot(140, 60);
  const start = motusComponent('joints', 140, 200, {}, { jointValues: MOTION_START });
  const ptpGoal = motusComponent('joints', 140, 320, {}, { jointValues: GOAL_JOINTS });
  const segPtp = motusComponent('segment', 340, 60, {
    Goal: [outRef(ptpGoal.node, 'State')],
  }, { text: { Type: 'PTP' } });
  const ptLin = nativeConstructPoint(140, 480, [0.45, 0.15, 0.45]);
  const uz = nativeUnitZ(140, 420);
  const plLin = nativePlane(260, 480, ptLin.node.outputs[0], uz.node.outputs[0]);
  const segLin = motusComponent('segment', 340, 240, {
    Goal: [outRef(plLin.node, 'Plane')],
  }, { text: { Type: 'LIN' } });
  const ptVia = nativeConstructPoint(140, 600, [0.453, 0.152, 0.45]);
  const ptGoal = nativeConstructPoint(140, 680, [0.45, 0.154, 0.45]);
  const plVia = nativePlane(260, 600, ptVia.node.outputs[0], uz.node.outputs[0]);
  const plGoal = nativePlane(260, 680, ptGoal.node.outputs[0], uz.node.outputs[0]);
  const segCirc = motusComponent('segment', 340, 420, {
    Goal: [outRef(plGoal.node, 'Plane')],
    Via: [outRef(plVia.node, 'Plane')],
  }, { text: { Type: 'CIRC' } });
  const progPlan = motusComponent('progPlan', 520, 200, {
    Robot: [outRef(robot.node, 'Robot')],
    Segments: [outRef(segPtp.node, 'Segment'), outRef(segLin.node, 'Segment'), outRef(segCirc.node, 'Segment')],
    Start: [outRef(start.node, 'State')],
  });
  const { scrub, preview } = previewWithScrub(720, 180, outRef(progPlan.node, 'Trajectory'));
  const exp = motusComponent('export', 720, 320, { Trajectory: [outRef(progPlan.node, 'Trajectory')] });
  const flat = [
    robot, start, ptpGoal, segPtp,
    { xml: ptLin.xml }, { xml: uz.xml }, { xml: plLin.xml },
    segLin,
    { xml: ptVia.xml }, { xml: ptGoal.xml }, { xml: plVia.xml }, { xml: plGoal.xml },
    segCirc, progPlan, scrub, preview, exp,
  ];
  flat._meta = {
    fileName: '08_motion_program.ghx',
    description: 'Motion program: PTP + LIN + CIRC segments -> Program Plan -> Preview / Export. Click Plan, then drag Motus Scrub or Play.',
  };
  return buildGraph(flat);
}

function graph09() {
  const urdfFile = nativeFilePath(-180, 60, 'examples/ur10e/ur10e.urdf');
  const tcpPl = nativeXYPlane(40, 200);
  const gripper = motusComponent('colBox', 40, 320, {}, {
    numbers: { HalfX: 0.02, HalfY: 0.02, HalfZ: 0.04 },
    text: { Name: 'gripper_geom' },
  });
  const tool = motusComponent('tool', 200, 260, {
    TCP: [outRef(tcpPl.node, 'Plane')],
    Geometry: [outRef(gripper.node, 'Object')],
  }, { text: { Name: 'gripper' } });
  const robot = motusComponent('robot', 380, 80, {
    Path: [outRef(urdfFile.node, 'Path')],
    Tool: [outRef(tool.node, 'Tool')],
  }, { text: { BaseLink: 'base_link', TipLink: 'tool0' } });
  const goal = motusComponent('joints', 380, 220, {}, { jointValues: GOAL_JOINTS });
  const plan = motusComponent('plan', 580, 160, {
    Robot: [outRef(robot.node, 'Robot')],
    Goal: [outRef(goal.node, 'State')],
  });
  const { scrub, preview } = previewWithScrub(780, 140, outRef(plan.node, 'Trajectory'));
  const exp = motusComponent('export', 780, 280, {
    Trajectory: [outRef(plan.node, 'Trajectory')],
  });
  const objs = [urdfFile, tcpPl, gripper, tool, robot, goal, plan, scrub, preview, exp];
  objs._meta = {
    fileName: '09_tool_tcp.ghx',
    description: 'Motus Robot (ur10e.urdf) + Motus Tool (TCP + gripper box) -> Plan -> Preview/Export.',
  };
  return buildGraph(objs);
}

function graph10() {
  const urdfFile = nativeFilePath(-180, 60, 'examples/ur10e/ur10e.urdf');
  const tcpPt = nativeConstructPoint(40, 200, [0, 0, 0.1633]);
  const ux = nativeUnitX(40, 260);
  const tcpPl = nativePlane(160, 200, tcpPt.node.outputs[0], ux.node.outputs[0]);
  const meshPath = nativeFilePath(40, 340, 'examples/ur10e/meshes/robotiq_2f85/robotiq_2f85_tcp_local.stl', '*.stl|*.stl|All files|*.*');
  const loadMesh = motusComponent('loadMesh', 200, 320, {
    Path: [outRef(meshPath.node, 'Path')],
  });
  const tool = motusComponent('tool', 380, 260, {
    TCP: [outRef(tcpPl.node, 'Plane')],
    Geometry: [outRef(loadMesh.node, 'Mesh')],
  }, { text: { Name: 'robotiq_2f85' } });
  const robot = motusComponent('robot', 540, 80, {
    Path: [outRef(urdfFile.node, 'Path')],
    Tool: [outRef(tool.node, 'Tool')],
  }, { text: { BaseLink: 'base_link', TipLink: 'tool0' } });
  const goal = motusComponent('joints', 540, 220, {}, { jointValues: GOAL_JOINTS });
  const plan = motusComponent('plan', 740, 160, {
    Robot: [outRef(robot.node, 'Robot')],
    Goal: [outRef(goal.node, 'State')],
  });
  const { scrub, preview } = previewWithScrub(940, 140, outRef(plan.node, 'Trajectory'));
  const objs = [urdfFile, tcpPt, ux, tcpPl, meshPath, loadMesh, tool, robot, goal, plan, scrub, preview];
  objs._meta = {
    fileName: '10_robotiq_tool.ghx',
    description: 'Motus Robot (ur10e.urdf) + Robotiq mesh via Motus Tool -> Plan + Preview.',
  };
  return buildGraph(objs);
}

function graph11() {
  const robot = motusComponent('ur10e', 40, 120);
  const start = motusComponent('joints', 40, 260, {}, { jointValues: MOTION_START });
  const ptpGoal = motusComponent('joints', 40, 360, {}, { jointValues: MOTION_START });
  const stateOpen = motusComponent('toolState', 200, 420, {}, { text: { Preset: 'Open' } });
  const stateClosed = motusComponent('toolState', 200, 520, {}, { text: { Preset: 'Closed' } });
  const segPtp = motusComponent('segment', 360, 280, {
    Goal: [outRef(ptpGoal.node, 'State')],
    ToolState: [outRef(stateOpen.node, 'State')],
  }, { text: { Type: 'PTP', ToolMode: 'Hold' } });
  const segSet = motusComponent('segment', 360, 420, {
    ToolState: [outRef(stateClosed.node, 'State')],
  }, { text: { Type: 'SET' }, numbers: { Duration: 0.2 } });
  const progPlan = motusComponent('progPlan', 540, 200, {
    Robot: [outRef(robot.node, 'Robot')],
    Segments: [outRef(segPtp.node, 'Segment'), outRef(segSet.node, 'Segment')],
    Start: [outRef(start.node, 'State')],
  });
  const { scrub, preview } = previewWithScrub(740, 180, outRef(progPlan.node, 'Trajectory'));
  const exp = motusComponent('export', 740, 320, { Trajectory: [outRef(progPlan.node, 'Trajectory')] });
  const objs = [robot, start, ptpGoal, stateOpen, stateClosed, segPtp, segSet, progPlan, scrub, preview, exp];
  objs._meta = {
    fileName: '11_gripper_motion_program.ghx',
    description: 'Motion program with SET gripper close: UR10e Robotiq -> Program Plan -> Preview/Export (toolState on trajectory).',
  };
  return buildGraph(objs);
}

function graph12() {
  const robot = ur10eRobot(140, 60);
  const start = motusComponent('joints', 140, 200, {}, { jointValues: MOTION_START });
  const goalJoint = motusComponent('joints', 140, 320, {}, { jointValues: GOAL_JOINTS });
  const uz = nativeUnitZ(140, 420);
  const ptLin1 = nativeConstructPoint(140, 480, [0.45, 0.15, 0.45]);
  const ptLin2 = nativeConstructPoint(140, 560, [0.48, 0.18, 0.48]);
  const plLin1 = nativePlane(260, 480, ptLin1.node.outputs[0], uz.node.outputs[0]);
  const plLin2 = nativePlane(260, 560, ptLin2.node.outputs[0], uz.node.outputs[0]);
  const plan = motusComponent('plan', 480, 180, {
    Robot: [outRef(robot.node, 'Robot')],
    Goal: [
      outRef(goalJoint.node, 'State'),
      outRef(plLin1.node, 'Plane'),
      outRef(plLin2.node, 'Plane'),
    ],
    Start: [outRef(start.node, 'State')],
  });
  const { scrub, preview } = previewWithScrub(680, 160, outRef(plan.node, 'Trajectory'));
  const exp = motusComponent('export', 680, 300, { Trajectory: [outRef(plan.node, 'Trajectory')] });
  const objs = [
    robot, start, goalJoint,
    { xml: uz.xml }, { xml: ptLin1.xml }, { xml: ptLin2.xml }, { xml: plLin1.xml }, { xml: plLin2.xml },
    plan, scrub, preview, exp,
  ];
  objs._meta = {
    fileName: '12_sequential_goals.ghx',
    description: 'Sequential goals: Joint State + two Plane goals wired to Plan.Goal (list) -> one chained trajectory. Click Plan, then drag Motus Scrub or Play.',
  };
  return buildGraph(objs);
}

const graphs = [graph01, graph02, graph03, graph04, graph05, graph06, graph07, graph08, graph09, graph10, graph11, graph12];
const legacy = ['01_basic_planning.ghx', '02_collision_planning.ghx'];

for (const name of legacy) {
  const p = path.join(outDir, name);
  if (fs.existsSync(p)) fs.unlinkSync(p);
}

for (const buildFn of graphs) {
  const xml = buildFn();
  const meta = lastGraphMeta;
  if (!meta?.fileName) throw new Error(`missing meta for ${buildFn.name}`);
  fs.writeFileSync(path.join(outDir, meta.fileName), xml, 'utf8');
  console.log('wrote', meta.fileName);
}

console.log('Done.');
