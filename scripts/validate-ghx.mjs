#!/usr/bin/env node
/**
 * Validate Motus example .ghx files (XML + wire GUID integrity).
 * Run from repo root: node scripts/validate-ghx.mjs
 *
 * When Rhino 8 is installed, also checks GH_IO.dll can deserialize each archive.
 */
import { spawnSync } from 'node:child_process';
import fs from 'node:fs';
import path from 'node:path';
import { fileURLToPath } from 'node:url';

const __dirname = path.dirname(fileURLToPath(import.meta.url));
const examplesDir = path.resolve(__dirname, '../examples');
const ghIoDll = 'C:\\Program Files\\Rhino 8\\Plug-ins\\Grasshopper\\GH_IO.dll';

const MOTUS_COMPONENTS = new Map([
  ['aa3e8488-943e-426f-b205-e8db5f684998', 'Motus Robot'],
  ['380f17c2-5d5f-4f77-a251-8309f25ef61e', 'Motus Joint State'],
  ['f1a2b3c4-d5e6-4789-a123-4567890abcde', 'Motus TCP Pose'],
  ['8bb0bae3-527f-4e80-a8a4-c8a88b7276de', 'Motus Plan'],
  ['d4a8f1c2-3e5b-4a7d-9c1e-8f2b6d4e0a91', 'Motus Preview'],
  ['0a443b6f-605b-48e3-843c-cd0a709f8379', 'Motus Export'],
  ['a72b5cfa-5cf5-4e54-a5cd-943e2aae82da', 'Motus Trajectory Data'],
  ['c1a2b3c4-d5e6-4789-a012-3456789abcde', 'Motus Collision Sphere'],
  ['d2b3c4d5-e6f7-4890-b123-456789abcdef', 'Motus Collision Box'],
  ['f4d5e6f7-a8b9-4012-d345-6789abcdef01', 'Motus Collision Mesh'],
  ['e3c4d5e6-f7a8-4901-c234-56789abcdef0', 'Motus Collision Scene'],
  ['91e2a9db-cfb4-4a6c-99a3-305ba27fdf1e', 'Motus Planning Group'],
  ['0c464ac8-0e1d-4c7a-9c8c-0a21f1046314', 'Motus Attach Body'],
  ['c8e4a1b2-3f5d-4e6a-9b7c-1d2e3f4a5b6c', 'Motus Load URDF'],
  ['7c4e9a2f-1b3d-4e8a-9f6c-2d8b5a7e9c31', 'Motus Motion Segment'],
  ['8d5f0b3e-2c4e-4f9b-0a7d-3e9c6b8f0d42', 'Motus Program Plan'],
  ['b7c4e2a1-9f3d-4b6e-8c1d-2a5f9e0b3d71', 'Motus Tool'],
  ['c3d4e5f6-a7b8-4901-c234-56789abcdef2', 'Motus Load Mesh'],
]);

const MOTUS_LIB = 'dc547e55-81a8-c313-e25d-e1468ddecddb';

function parseXml(name, xml) {
  if (!xml.startsWith('<?xml')) throw new Error('missing XML declaration');
  if (!xml.includes('<Archive name="Root">')) throw new Error('missing Root archive');

  const itemBlocks = [...xml.matchAll(/<items count="(\d+)">([\s\S]*?)<\/items>/g)];
  for (const [, declared, body] of itemBlocks) {
    const actual = (body.match(/<item /g) ?? []).length;
    if (Number(declared) !== actual) {
      throw new Error(`items count=${declared} but found ${actual} <item> elements`);
    }
  }

  const objectCount = Number(xml.match(/name="ObjectCount"[^>]*>(\d+)</)?.[1]);
  const objects = (xml.match(/<chunk name="Object"/g) ?? []).length;
  if (!Number.isFinite(objectCount)) throw new Error('missing ObjectCount');
  if (objectCount !== objects) throw new Error(`ObjectCount ${objectCount} != objects ${objects}`);

  const instanceGuids = new Set(
    [...xml.matchAll(/name="InstanceGuid"[^>]*>([0-9a-f-]{36})</gi)].map((m) => m[1].toLowerCase()),
  );
  const sources = [...xml.matchAll(/name="Source"[^>]*>([0-9a-f-]{36})</gi)].map((m) => m[1].toLowerCase());
  const dangling = [...new Set(sources.filter((s) => !instanceGuids.has(s)))];
  if (dangling.length) throw new Error(`dangling wire sources: ${dangling.join(', ')}`);

  const libs = [...xml.matchAll(/<chunk name="Library"[\s\S]*?<item name="Id"[^>]*>([0-9a-f-]{36})<\/item>/gi)]
    .map((m) => m[1].toLowerCase());
  if (!libs.includes(MOTUS_LIB)) throw new Error('Motus GHA library entry missing');

  const objectChunks = xml.split(/<chunk name="Object" index="/).slice(1);
  for (const chunk of objectChunks) {
    const guid = chunk.match(/<item name="GUID"[^>]*>([0-9a-f-]{36})<\/item>/)?.[1];
    const lib = chunk.match(/<item name="Lib"[^>]*>([0-9a-f-]{36})<\/item>/)?.[1];
    if (!lib || lib.toLowerCase() !== MOTUS_LIB) continue;
    if (!guid || !MOTUS_COMPONENTS.has(guid.toLowerCase())) {
      throw new Error(`unknown Motus component GUID ${guid ?? '(missing)'}`);
    }
  }

  return { objectCount, wireCount: sources.length };
}

function ghIoRead(file) {
  const ps = `
Add-Type -Path '${ghIoDll.replace(/'/g, "''")}'
$arch = New-Object GH_IO.Serialization.GH_Archive
$ok = $arch.ReadFromFile('${file.replace(/'/g, "''")}')
if (-not $ok) { throw 'GH_IO ReadFromFile returned false' }
`;
  const r = spawnSync('powershell', ['-NoProfile', '-Command', ps], { encoding: 'utf8' });
  if (r.status !== 0) throw new Error((r.stderr || r.stdout || 'GH_IO read failed').trim());
}

const files = fs.readdirSync(examplesDir).filter((f) => f.endsWith('.ghx')).sort();
const ghIoAvailable = fs.existsSync(ghIoDll);
let failed = 0;

for (const name of files) {
  const file = path.join(examplesDir, name);
  const xml = fs.readFileSync(file, 'utf8').replace(/^\uFEFF/, '');
  try {
    const stats = parseXml(name, xml);
    if (ghIoAvailable) ghIoRead(file);
    const io = ghIoAvailable ? ' GH_IO' : '';
    console.log(`OK  ${name}  objects=${stats.objectCount} wires=${stats.wireCount}${io}`);
  } catch (err) {
    failed++;
    console.error(`FAIL ${name}: ${err.message}`);
  }
}

if (failed) process.exit(1);
const ioNote = ghIoAvailable ? ' (GH_IO.dll archive read)' : ' (GH_IO.dll not found — wire/XML checks only)';
console.log(`\n${files.length}/${files.length} example GHX files passed validation${ioNote}.`);
