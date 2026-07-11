#!/usr/bin/env node
/**
 * PickNik ros2_robotiq_gripper (BSD-3-Clause): https://github.com/PickNikRobotics/ros2_robotiq_gripper
 * Alternatives: ros-industrial/robotiq, a-price/robotiq_arg85_description
 *
 * Downloads visual (DAE) + collision (STL) meshes, builds merged collision STLs for Motus Tool,
 * and patches ur10e_robotiq.urdf with a per-link visual gripper tree on tool0.
 */
import fs from 'node:fs';
import path from 'node:path';
import { fileURLToPath } from 'node:url';

const __dirname = path.dirname(fileURLToPath(import.meta.url));
const repoRoot = path.resolve(__dirname, '..');
const ur10eDir = path.join(repoRoot, 'examples', 'ur10e');
const toolsDir = path.join(repoRoot, 'resources', 'tools');
const bundleUrdf = path.join(repoRoot, 'resources', 'robots', 'ur10e_robotiq', 'ur10e_robotiq.urdf');
const exampleUrdf = path.join(ur10eDir, 'ur10e_robotiq.urdf');
const meshRoot =
  'https://raw.githubusercontent.com/PickNikRobotics/ros2_robotiq_gripper/main/robotiq_description/meshes';

const PART_FILES = [
  'ur_to_robotiq_adapter',
  'robotiq_base',
  'left_knuckle',
  'right_knuckle',
  'left_finger',
  'right_finger',
  'left_inner_knuckle',
  'right_inner_knuckle',
  'left_finger_tip',
  'right_finger_tip',
];

// UR10e flange -> tool0 (matches ur10e_robotiq.urdf flange-tool0).
const T_FLANGE_TOOL0 = tfRpy([0, 0, 0], [Math.PI / 2, 0, Math.PI / 2]);
// Robotiq TCP in flange frame (matches Motus ToolDefinition / UR10e.json).
const T_FLANGE_TCP = tfRpy([0, 0, 0.1633], [0, Math.PI / 2, 0]);
const T_TCP_TOOL0 = mul(inv(T_FLANGE_TCP), T_FLANGE_TOOL0);

const T_ADAPTER_BASE = tfRpy([0, 0, 0.011], [0, 0, 0]);
const T_LEFT_KNUCKLE = tfRpy([0.03060114, 0, 0.05490452], [0, 0, 0]);
const T_RIGHT_KNUCKLE = tfRpy([-0.03060114, 0, 0.05490452], [0, 0, 0]);
const T_LEFT_FINGER = mul(T_LEFT_KNUCKLE, tfRpy([0.03152616, 0, -0.00376347], [0, 0, 0]));
const T_RIGHT_FINGER = mul(T_RIGHT_KNUCKLE, tfRpy([-0.03152616, 0, -0.00376347], [0, 0, 0]));
const T_LEFT_INNER = tfRpy([0.0127, 0, 0.06142], [0, 0, 0]);
const T_RIGHT_INNER = tfRpy([-0.0127, 0, 0.06142], [0, 0, 0]);
const T_LEFT_TIP = mul(T_LEFT_FINGER, tfRpy([0.00563134, 0, 0.04718515], [0, 0, 0]));
const T_RIGHT_TIP = mul(T_RIGHT_FINGER, tfRpy([-0.00563134, 0, 0.04718515], [0, 0, 0]));

const IDENTITY = tfRpy([0, 0, 0], [0, 0, 0]);

// Joint origins from PickNik robotiq_2f_85_macro.urdf.xacro (closed pose, q=0).
const PARTS_IN_TOOL0 = [
  { file: 'ur_to_robotiq_adapter.stl', tf: IDENTITY },
  { file: 'robotiq_base.stl', tf: T_ADAPTER_BASE },
  { file: 'left_knuckle.stl', tf: mul(T_ADAPTER_BASE, T_LEFT_KNUCKLE) },
  { file: 'right_knuckle.stl', tf: mul(T_ADAPTER_BASE, T_RIGHT_KNUCKLE) },
  { file: 'left_finger.stl', tf: mul(T_ADAPTER_BASE, T_LEFT_FINGER) },
  { file: 'right_finger.stl', tf: mul(T_ADAPTER_BASE, T_RIGHT_FINGER) },
  { file: 'left_inner_knuckle.stl', tf: mul(T_ADAPTER_BASE, T_LEFT_INNER) },
  { file: 'right_inner_knuckle.stl', tf: mul(T_ADAPTER_BASE, T_RIGHT_INNER) },
  { file: 'left_finger_tip.stl', tf: mul(T_ADAPTER_BASE, T_LEFT_TIP) },
  { file: 'right_finger_tip.stl', tf: mul(T_ADAPTER_BASE, T_RIGHT_TIP) },
];

const GRIPPER_LINKS = [
  { name: 'robotiq_adapter', mesh: 'ur_to_robotiq_adapter' },
  { name: 'robotiq_base', mesh: 'robotiq_base' },
  { name: 'robotiq_left_knuckle', mesh: 'left_knuckle' },
  { name: 'robotiq_right_knuckle', mesh: 'right_knuckle' },
  { name: 'robotiq_left_finger', mesh: 'left_finger' },
  { name: 'robotiq_right_finger', mesh: 'right_finger' },
  { name: 'robotiq_left_inner_knuckle', mesh: 'left_inner_knuckle' },
  { name: 'robotiq_right_inner_knuckle', mesh: 'right_inner_knuckle' },
  { name: 'robotiq_left_finger_tip', mesh: 'left_finger_tip' },
  { name: 'robotiq_right_finger_tip', mesh: 'right_finger_tip' },
];

const GRIPPER_JOINTS = [
  { name: 'tool0_robotiq_adapter', parent: 'tool0', child: 'robotiq_adapter', xyz: '0 0 0', rpy: '0 0 0' },
  { name: 'robotiq_adapter_base', parent: 'robotiq_adapter', child: 'robotiq_base', xyz: '0 0 0.011', rpy: '0 0 0' },
  { name: 'robotiq_base_left_knuckle', parent: 'robotiq_base', child: 'robotiq_left_knuckle', xyz: '0.03060114 0 0.05490452', rpy: '0 0 0' },
  { name: 'robotiq_base_right_knuckle', parent: 'robotiq_base', child: 'robotiq_right_knuckle', xyz: '-0.03060114 0 0.05490452', rpy: '0 0 0' },
  { name: 'robotiq_left_knuckle_finger', parent: 'robotiq_left_knuckle', child: 'robotiq_left_finger', xyz: '0.03152616 0 -0.00376347', rpy: '0 0 0' },
  { name: 'robotiq_right_knuckle_finger', parent: 'robotiq_right_knuckle', child: 'robotiq_right_finger', xyz: '-0.03152616 0 -0.00376347', rpy: '0 0 0' },
  { name: 'robotiq_base_left_inner', parent: 'robotiq_base', child: 'robotiq_left_inner_knuckle', xyz: '0.0127 0 0.06142', rpy: '0 0 0' },
  { name: 'robotiq_base_right_inner', parent: 'robotiq_base', child: 'robotiq_right_inner_knuckle', xyz: '-0.0127 0 0.06142', rpy: '0 0 0' },
  { name: 'robotiq_left_finger_tip', parent: 'robotiq_left_finger', child: 'robotiq_left_finger_tip', xyz: '0.00563134 0 0.04718515', rpy: '0 0 0' },
  { name: 'robotiq_right_finger_tip', parent: 'robotiq_right_finger', child: 'robotiq_right_finger_tip', xyz: '-0.00563134 0 0.04718515', rpy: '0 0 0' },
];

export async function fetchRobotiqAssets() {
  const collisionDir = path.join(ur10eDir, 'meshes', 'robotiq_2f85', 'collision');
  const visualDir = path.join(ur10eDir, 'meshes', 'robotiq_2f85', 'visual');
  fs.mkdirSync(collisionDir, { recursive: true });
  fs.mkdirSync(visualDir, { recursive: true });
  fs.mkdirSync(toolsDir, { recursive: true });

  for (const base of PART_FILES) {
    for (const [kind, dir, ext] of [
      ['collision', collisionDir, 'stl'],
      ['visual', visualDir, 'dae'],
    ]) {
      const name = `${base}.${ext}`;
      const dest = path.join(dir, name);
      if (fs.existsSync(dest) && fs.statSync(dest).size > 0) {
        console.log(`skip robotiq ${kind}/${name}`);
        continue;
      }
      console.log(`fetch robotiq ${kind}/${name}`);
      const res = await fetch(`${meshRoot}/${kind}/2f_85/${name}`);
      if (!res.ok) throw new Error(`Failed ${kind}/${name}: ${res.status}`);
      fs.writeFileSync(dest, Buffer.from(await res.arrayBuffer()));
    }
  }

  const tcpLocal = [];
  const tool0Local = [];
  for (const part of PARTS_IN_TOOL0) {
    const filePath = path.join(collisionDir, part.file);
    const tris = readBinaryStl(filePath);
    const tfTcp = mul(T_TCP_TOOL0, part.tf);
    for (const [a, b, c] of tris) {
      tcpLocal.push([apply(tfTcp, a), apply(tfTcp, b), apply(tfTcp, c)]);
      tool0Local.push([apply(part.tf, a), apply(part.tf, b), apply(part.tf, c)]);
    }
  }

  const tcpPath = path.join(ur10eDir, 'meshes', 'robotiq_2f85', 'robotiq_2f85_tcp_local.stl');
  const tool0Path = path.join(ur10eDir, 'meshes', 'robotiq_2f85', 'robotiq_2f85_tool0.stl');
  const bundledPath = path.join(toolsDir, 'robotiq_2f85_tcp_local.stl');

  writeBinaryStl(tcpPath, tcpLocal);
  writeBinaryStl(tool0Path, tool0Local);
  fs.copyFileSync(tcpPath, bundledPath);

  console.log(`Wrote ${tcpPath} (${tcpLocal.length} triangles)`);
  console.log(`Wrote ${tool0Path} (${tool0Local.length} triangles)`);
  console.log(`Bundled ${bundledPath}`);

  patchUrdfGripper(exampleUrdf);
  if (fs.existsSync(bundleUrdf)) patchUrdfGripper(bundleUrdf);
}

function patchUrdfGripper(urdfPath) {
  let src = fs.readFileSync(urdfPath, 'utf8');
  const linkXml = GRIPPER_LINKS.map(({ name, mesh }) => `  <link name="${name}">
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry><mesh filename="meshes/robotiq_2f85/visual/${mesh}.dae"/></geometry>
    </visual>
  </link>`).join('\n');

  const jointXml = GRIPPER_JOINTS.map(({ name, parent, child, xyz, rpy }) => `  <joint name="${name}" type="fixed">
    <parent link="${parent}"/><child link="${child}"/>
    <origin xyz="${xyz}" rpy="${rpy}"/>
  </joint>`).join('\n');

  const gripperBlock = `${linkXml}\n\n${jointXml}`;

  // Legacy single-mesh gripper.
  src = src.replace(/  <link name="robotiq_85_assembly">[\s\S]*?<\/link>\r?\n/, '');
  src = src.replace(/  <joint name="tool0_robotiq" type="fixed">[\s\S]*?<\/joint>\r?\n/, '');

  // Idempotent: strip prior per-link gripper before re-inserting.
  src = src.replace(/  <link name="robotiq_[^"]+">[\s\S]*?<\/link>\r?\n/g, '');
  src = src.replace(/  <joint name="(?:tool0_robotiq_adapter|robotiq_[^"]+)" type="fixed">[\s\S]*?<\/joint>\r?\n/g, '');

  const marker = '  <link name="tool0"/>';
  if (!src.includes(marker)) throw new Error(`tool0 link not found in ${urdfPath}`);
  const replaced = src.replace(marker, `${marker}\n${gripperBlock}`);
  fs.writeFileSync(urdfPath, replaced);
  console.log(`Patched gripper visuals in ${urdfPath}`);
}

if (process.argv[1] && path.resolve(process.argv[1]) === fileURLToPath(import.meta.url)) {
  await fetchRobotiqAssets();
}

function tfRpy(xyz, rpy) {
  const [rx, ry, rz] = rpy;
  const cx = Math.cos(rx), sx = Math.sin(rx);
  const cy = Math.cos(ry), sy = Math.sin(ry);
  const cz = Math.cos(rz), sz = Math.sin(rz);
  return {
    r00: cy * cz, r01: cz * sx * sy - cx * sz, r02: cx * cz * sy + sx * sz,
    r10: cy * sz, r11: cx * cz + sx * sy * sz, r12: -cz * sx + cx * sy * sz,
    r20: -sy, r21: cy * sx, r22: cx * cy,
    tx: xyz[0], ty: xyz[1], tz: xyz[2],
  };
}

function mul(a, b) {
  return {
    r00: a.r00 * b.r00 + a.r01 * b.r10 + a.r02 * b.r20,
    r01: a.r00 * b.r01 + a.r01 * b.r11 + a.r02 * b.r21,
    r02: a.r00 * b.r02 + a.r01 * b.r12 + a.r02 * b.r22,
    r10: a.r10 * b.r00 + a.r11 * b.r10 + a.r12 * b.r20,
    r11: a.r10 * b.r01 + a.r11 * b.r11 + a.r12 * b.r21,
    r12: a.r10 * b.r02 + a.r11 * b.r12 + a.r12 * b.r22,
    r20: a.r20 * b.r00 + a.r21 * b.r10 + a.r22 * b.r20,
    r21: a.r20 * b.r01 + a.r21 * b.r11 + a.r22 * b.r21,
    r22: a.r20 * b.r02 + a.r21 * b.r12 + a.r22 * b.r22,
    tx: a.r00 * b.tx + a.r01 * b.ty + a.r02 * b.tz + a.tx,
    ty: a.r10 * b.tx + a.r11 * b.ty + a.r12 * b.tz + a.ty,
    tz: a.r20 * b.tx + a.r21 * b.ty + a.r22 * b.tz + a.tz,
  };
}

function inv(t) {
  const r00 = t.r00, r01 = t.r10, r02 = t.r20;
  const r10 = t.r01, r11 = t.r11, r12 = t.r21;
  const r20 = t.r02, r21 = t.r12, r22 = t.r22;
  return {
    r00, r01, r02, r10, r11, r12, r20, r21, r22,
    tx: -(r00 * t.tx + r01 * t.ty + r02 * t.tz),
    ty: -(r10 * t.tx + r11 * t.ty + r12 * t.tz),
    tz: -(r20 * t.tx + r21 * t.ty + r22 * t.tz),
  };
}

function apply(t, p) {
  return [
    t.r00 * p[0] + t.r01 * p[1] + t.r02 * p[2] + t.tx,
    t.r10 * p[0] + t.r11 * p[1] + t.r12 * p[2] + t.ty,
    t.r20 * p[0] + t.r21 * p[1] + t.r22 * p[2] + t.tz,
  ];
}

function readBinaryStl(filePath) {
  const bytes = fs.readFileSync(filePath);
  if (bytes.length < 84) return [];
  const triCount = bytes.readUInt32LE(80);
  const tris = [];
  let offset = 84;
  for (let i = 0; i < triCount && offset + 50 <= bytes.length; i++) {
    offset += 12;
    const verts = [];
    for (let v = 0; v < 3; v++) {
      verts.push([bytes.readFloatLE(offset), bytes.readFloatLE(offset + 4), bytes.readFloatLE(offset + 8)]);
      offset += 12;
    }
    tris.push(verts);
    offset += 2;
  }
  return tris;
}

function writeBinaryStl(filePath, triangles) {
  const header = Buffer.alloc(80, ' ');
  Buffer.from('Robotiq 2F-85 merged collision').copy(header);
  const buf = Buffer.alloc(84 + triangles.length * 50);
  header.copy(buf, 0);
  buf.writeUInt32LE(triangles.length, 80);
  let offset = 84;
  for (const tri of triangles) {
    if (!Array.isArray(tri) || tri.length !== 3) {
      throw new Error(`writeBinaryStl: expected [v0,v1,v2] triangle, got ${JSON.stringify(tri)}`);
    }
    offset += 12;
    for (const p of tri) {
      if (!Array.isArray(p) || p.length !== 3 || !p.every(Number.isFinite)) {
        throw new Error(`writeBinaryStl: invalid vertex ${JSON.stringify(p)}`);
      }
      buf.writeFloatLE(p[0], offset); offset += 4;
      buf.writeFloatLE(p[1], offset); offset += 4;
      buf.writeFloatLE(p[2], offset); offset += 4;
    }
    buf.writeUInt16LE(0, offset); offset += 2;
  }
  fs.writeFileSync(filePath, buf);
}
