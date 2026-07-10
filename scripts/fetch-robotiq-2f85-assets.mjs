#!/usr/bin/env node
/**
 * Download Robotiq 2F-85 collision meshes and build merged STLs for Motus Tool / URDF.
 * Outputs under examples/ur10e/meshes/robotiq_2f85/ and resources/tools/.
 *
 * Usually run via: node scripts/fetch-ur10e-assets.mjs
 */
import fs from 'node:fs';
import path from 'node:path';
import { fileURLToPath } from 'node:url';

const __dirname = path.dirname(fileURLToPath(import.meta.url));
const repoRoot = path.resolve(__dirname, '..');
const ur10eDir = path.join(repoRoot, 'examples', 'ur10e');
const toolsDir = path.join(repoRoot, 'resources', 'tools');
const meshBase =
  'https://raw.githubusercontent.com/PickNikRobotics/ros2_robotiq_gripper/main/robotiq_description/meshes/collision/2f_85';

const PART_FILES = [
  'ur_to_robotiq_adapter.stl',
  'robotiq_base.stl',
  'left_knuckle.stl',
  'right_knuckle.stl',
  'left_finger.stl',
  'right_finger.stl',
  'left_inner_knuckle.stl',
  'right_inner_knuckle.stl',
  'left_finger_tip.stl',
  'right_finger_tip.stl',
];

const T_FLANGE_TOOL0 = tfRpy([0, 0, 0], [Math.PI / 2, 0, Math.PI / 2]);
const T_TOOL0_BASE = tfRpy([0, 0, 0.011], [0, 0, 0]);
const TCP_FLANGE = tfRpy([0, 0, 0.1633], [0, Math.PI / 2, 0]);

const PARTS_IN_BASE = [
  { file: 'ur_to_robotiq_adapter.stl', tf: inv(T_TOOL0_BASE) },
  { file: 'robotiq_base.stl', tf: tfRpy([0, 0, 0.03232288], [0, 0, 0]) },
  { file: 'left_knuckle.stl', tf: tfRpy([0.03060114, 0, 0.05490452], [0, 0, 0]) },
  { file: 'right_knuckle.stl', tf: tfRpy([-0.03060114, 0, 0.05490452], [0, 0, 0]) },
  { file: 'left_finger.stl', tf: mul(tfRpy([0.03060114, 0, 0.05490452], [0, 0, 0]), tfRpy([0.03152616, 0, -0.00376347], [0, 0, 0])) },
  { file: 'right_finger.stl', tf: mul(tfRpy([-0.03060114, 0, 0.05490452], [0, 0, 0]), tfRpy([-0.03152616, 0, -0.00376347], [0, 0, 0])) },
  { file: 'left_inner_knuckle.stl', tf: tfRpy([0.0127, 0, 0.06142], [0, 0, 0]) },
  { file: 'right_inner_knuckle.stl', tf: tfRpy([-0.0127, 0, 0.06142], [0, 0, 0]) },
  { file: 'left_finger_tip.stl', tf: mul(
    mul(tfRpy([0.03060114, 0, 0.05490452], [0, 0, 0]), tfRpy([0.03152616, 0, -0.00376347], [0, 0, 0])),
    tfRpy([0.00563134, 0, 0.04718515], [0, 0, 0]),
  ) },
  { file: 'right_finger_tip.stl', tf: mul(
    mul(tfRpy([-0.03060114, 0, 0.05490452], [0, 0, 0]), tfRpy([-0.03152616, 0, -0.00376347], [0, 0, 0])),
    tfRpy([-0.00563134, 0, 0.04718515], [0, 0, 0]),
  ) },
];

const T_FLANGE_BASE = mul(T_FLANGE_TOOL0, T_TOOL0_BASE);
const T_TCP_LOCAL = inv(TCP_FLANGE);
const T_TOOL0_LOCAL = inv(T_FLANGE_TOOL0);

export async function fetchRobotiqAssets() {
  const collisionDir = path.join(ur10eDir, 'meshes', 'robotiq_2f85', 'collision');
  fs.mkdirSync(collisionDir, { recursive: true });
  fs.mkdirSync(toolsDir, { recursive: true });

  for (const name of PART_FILES) {
    const dest = path.join(collisionDir, name);
    if (fs.existsSync(dest) && fs.statSync(dest).size > 0) {
      console.log(`skip robotiq ${name}`);
      continue;
    }
    console.log(`fetch robotiq ${name}`);
    const res = await fetch(`${meshBase}/${name}`);
    if (!res.ok) throw new Error(`Failed ${name}: ${res.status}`);
    fs.writeFileSync(dest, Buffer.from(await res.arrayBuffer()));
  }

  const tcpLocal = [];
  const tool0Local = [];
  for (const part of PARTS_IN_BASE) {
    const filePath = path.join(collisionDir, part.file);
    const tris = readBinaryStl(filePath);
    const tfTcp = mul(T_TCP_LOCAL, mul(T_FLANGE_BASE, part.tf));
    const tfTool0 = mul(T_TOOL0_LOCAL, mul(T_FLANGE_BASE, part.tf));
    for (const [a, b, c] of tris) {
      tcpLocal.push(apply(tfTcp, a), apply(tfTcp, b), apply(tfTcp, c));
      tool0Local.push(apply(tfTool0, a), apply(tfTool0, b), apply(tfTool0, c));
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
}

if (import.meta.url === `file://${process.argv[1].replace(/\\/g, '/')}`) {
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
    offset += 12;
    for (const p of tri) {
      buf.writeFloatLE(p[0], offset); offset += 4;
      buf.writeFloatLE(p[1], offset); offset += 4;
      buf.writeFloatLE(p[2], offset); offset += 4;
    }
    buf.writeUInt16LE(0, offset); offset += 2;
  }
  fs.writeFileSync(filePath, buf);
}
