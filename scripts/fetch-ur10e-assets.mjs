#!/usr/bin/env node
/**
 * Download UR10e visual (DAE) and collision (STL) meshes from Universal Robots ROS2 description.
 * Run from repo root: node scripts/fetch-ur10e-assets.mjs
 */
import fs from 'node:fs';
import path from 'node:path';
import { fileURLToPath } from 'node:url';
import { fetchRobotiqAssets } from './fetch-robotiq-2f85-assets.mjs';

const __dirname = path.dirname(fileURLToPath(import.meta.url));
const outDir = path.resolve(__dirname, '../examples/ur10e');
const baseUrl =
  'https://raw.githubusercontent.com/UniversalRobots/Universal_Robots_ROS2_Description/rolling/meshes/ur10e';

const meshes = [
  { subdir: 'visual', files: ['base.dae', 'shoulder.dae', 'upperarm.dae', 'forearm.dae', 'wrist1.dae', 'wrist2.dae', 'wrist3.dae'] },
  { subdir: 'collision', files: ['base.stl', 'shoulder.stl', 'upperarm.stl', 'forearm.stl', 'wrist1.stl', 'wrist2.stl', 'wrist3.stl'] },
];

for (const { subdir, files } of meshes) {
  const meshDir = path.join(outDir, 'meshes', 'ur10e', subdir);
  fs.mkdirSync(meshDir, { recursive: true });
  for (const name of files) {
    const dest = path.join(meshDir, name);
    if (fs.existsSync(dest) && fs.statSync(dest).size > 0) {
      console.log(`skip ${subdir}/${name}`);
      continue;
    }
    console.log(`fetch ${subdir}/${name}`);
    const res = await fetch(`${baseUrl}/${subdir}/${name}`);
    if (!res.ok) throw new Error(`Failed ${subdir}/${name}: ${res.status}`);
    fs.writeFileSync(dest, Buffer.from(await res.arrayBuffer()));
  }
}

console.log(`Meshes in ${path.join(outDir, 'meshes', 'ur10e')}`);
await fetchRobotiqAssets();
