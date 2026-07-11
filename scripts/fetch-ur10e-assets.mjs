#!/usr/bin/env node
/**
 * Download UR10e visual (DAE) and collision (STL) meshes from Universal Robots ROS2 description.
 * Populates examples/ur10e and resources/robots/ur10e_robotiq (plugin bundle).
 * Run from repo root: node scripts/fetch-ur10e-assets.mjs
 */
import fs from 'node:fs';
import path from 'node:path';
import { fileURLToPath } from 'node:url';
import { fetchRobotiqAssets } from './fetch-robotiq-2f85-assets.mjs';

const __dirname = path.dirname(fileURLToPath(import.meta.url));
const outDir = path.resolve(__dirname, '../examples/ur10e');
const bundleDir = path.resolve(__dirname, '../resources/robots/ur10e_robotiq');
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

function copyDir(src, dest, filter = () => true) {
  fs.mkdirSync(dest, { recursive: true });
  for (const entry of fs.readdirSync(src, { withFileTypes: true })) {
    const rel = entry.name;
    if (!filter(rel, entry)) continue;
    const s = path.join(src, rel);
    const d = path.join(dest, rel);
    if (entry.isDirectory()) copyDir(s, d, filter);
    else fs.copyFileSync(s, d);
  }
}

function copyBundleMeshes(srcMeshes, destMeshes) {
  const ur10eSrc = path.join(srcMeshes, 'ur10e');
  if (fs.existsSync(ur10eSrc)) {
    copyDir(ur10eSrc, path.join(destMeshes, 'ur10e'));
  }

  // Robotiq: visual DAE only in plugin bundle (collision + merged STLs stay under examples/).
  const robotiqSrc = path.join(srcMeshes, 'robotiq_2f85');
  const robotiqDest = path.join(destMeshes, 'robotiq_2f85');
  if (fs.existsSync(robotiqSrc)) {
    fs.rmSync(robotiqDest, { recursive: true, force: true });
    copyDir(robotiqSrc, robotiqDest, (name, entry) => {
      if (entry.isDirectory()) return name === 'visual';
      return name.endsWith('.dae');
    });
  }
}

fs.mkdirSync(bundleDir, { recursive: true });
fs.copyFileSync(path.join(outDir, 'ur10e_robotiq.urdf'), path.join(bundleDir, 'ur10e_robotiq.urdf'));
copyBundleMeshes(path.join(outDir, 'meshes'), path.join(bundleDir, 'meshes'));
console.log(`Plugin bundle: ${bundleDir}`);
