#!/usr/bin/env node
/** Patch Motus Move MotionType + Ty persistent per object chunk (safe). */
import fs from 'fs';
import path from 'path';
import { fileURLToPath } from 'url';

const ghxPath = path.resolve(path.dirname(fileURLToPath(import.meta.url)), '../examples/10_pick_place.ghx');
let xml = fs.readFileSync(ghxPath, 'utf8');

const moves = new Map([
  ['7480c626-890b-4b33-bc48-50f6e1a774bf', 'LIN'],
  ['4b4949fa-db79-4978-b04d-e1b58bb5e980', 'SET'],
  ['7a35e671-3cb4-4473-8660-be80c365651f', 'LIN'],
  ['8a5a56fc-c9c6-41df-9c2d-8d9525f28de7', 'SET'],
  ['02df5e63-5357-4a68-9532-83a8cb6453d8', 'LIN'],
]);

const parts = xml.split(/(?=<chunk name="Object")/);
const out = parts.map((chunk, i) => {
  if (i === 0) return chunk;
  const guid = [...moves.keys()].find((g) => chunk.includes(g));
  if (!guid) return chunk;
  const type = moves.get(guid);
  let c = chunk.replace(
    /(<item name="MotionType" type_name="gh_string" type_code="10">)[^<]+(<\/item>)/,
    `$1${type}$2`,
  );
  c = c.replace(
    /(<item name="string" type_name="gh_string" type_code="10">)PTP(<\/item>)/,
    `$1${type}$2`,
  );
  return c;
});

fs.writeFileSync(ghxPath, out.join(''));
console.log(`patched ${moves.size} moves in ${ghxPath}`);
