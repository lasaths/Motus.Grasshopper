#!/usr/bin/env node
/** Optional post-patch for legacy 10_pick_place.ghx. Prefer: node scripts/generate-examples.mjs --only=10 */
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

/** UR10e home-relative poses in Rhino base frame (match Motus TCP Pose / DePlane origin signs). */
const points = new Map([
  ['49d1a48b-0d08-428e-bf9c-03db80ca7bc8', [0.8514, 0.1741, 0.6136]],
  ['8fd502c7-f1d3-4313-ab5b-9160cf9f6db0', [0.8014, 0.2241, 0.6735]],
  ['c5eb071e-413d-454c-85ee-c2c16cb92b0d', [0.8014, 0.2241, 0.7735]],
]);

const HOME_JOINTS = [0, -1.5708, 1.5708, 0, 1.5708, 0];
const HOME_JOINTS_GUID = 'a2510467-9765-47bf-8e0a-5566826a1c9d';
/** DePlane pick TCP + Amplitude offsets (m) → place/retract LIN goals. */
const amplitudes = new Map([
  ['3b104834-0363-4bdf-959c-31f9e94c430e', -0.05],
  ['959a3baf-afbb-4137-a509-64f7f87a58c6', 0.05],
  ['4fc0f0dc-b41b-46b8-b08b-5b3b7e7cff8a', 0.1],
]);
const LIN_SEG_OUT = 'bde1b78e-30d2-4c70-acbd-d0affc80556a';
const SET_CLOSE_OUT = '2600b781-6842-4ffa-a4ae-eed71147d347';
const SET_OPEN_OUT = '04a38157-a439-4a40-aa06-5ada0d76f6c0';

function patchConstructPoint(chunk, [x, y, z]) {
  const vals = [x, y, z];
  return chunk.replace(
    /(<chunk name="param_input" index="([012])">[\s\S]*?<chunk name="PersistentData">[\s\S]*?<item name="number" type_name="gh_double" type_code="6">)[^<]+(<\/item>)/g,
    (m, pre, idx, post) => {
      const i = Number(idx);
      if (i > 2) return m;
      return `${pre}${vals[i]}${post}`;
    },
  );
}

function patchHomeJoints(chunk) {
  if (!chunk.includes(HOME_JOINTS_GUID)) return chunk;
  const items = HOME_JOINTS.map(
    (v, i) => `
                                <chunk name="Item" index="${i}">
                                  <items count="1">
                                    <item name="number" type_name="gh_double" type_code="6">${v}</item>
                                  </items>
                                </chunk>`,
  ).join('');
  const paramInput = `<chunk name="param_input" index="0">
                      <items count="7">
                        <item name="Access" type_name="gh_int32" type_code="3">1</item>
                        <item name="Description" type_name="gh_string" type_code="10">Joint angles (right-click J input to toggle °)</item>
                        <item name="InstanceGuid" type_name="gh_guid" type_code="9">3f881a2c-29b0-4912-ac4f-68ea5662868e</item>
                        <item name="Name" type_name="gh_string" type_code="10">Joints</item>
                        <item name="NickName" type_name="gh_string" type_code="10">J</item>
                        <item name="Optional" type_name="gh_bool" type_code="1">false</item>
                        <item name="SourceCount" type_name="gh_int32" type_code="3">0</item>
                      </items>
                      <chunks count="3">
                        <chunk name="Attributes">
                          <items count="2">
                            <item name="Bounds" type_name="gh_drawing_rectanglef" type_code="35">
                              <X>374</X>
                              <Y>208</Y>
                              <W>11</W>
                              <H>24</H>
                            </item>
                            <item name="Pivot" type_name="gh_drawing_pointf" type_code="31">
                              <X>381</X>
                              <Y>220</Y>
                            </item>
                          </items>
                        </chunk>
                        <chunk name="PersistentData">
                          <items count="1">
                            <item name="Count" type_name="gh_int32" type_code="3">1</item>
                          </items>
                          <chunks count="1">
                            <chunk name="Branch" index="0">
                              <items count="2">
                                <item name="Count" type_name="gh_int32" type_code="3">${HOME_JOINTS.length}</item>
                                <item name="Path" type_name="gh_string" type_code="10">{0}</item>
                              </items>
                              <chunks count="${HOME_JOINTS.length}">${items}
                              </chunks>
                            </chunk>
                          </chunks>
                        </chunk>
                        <chunk name="FixedSettings">
                          <items count="1">
                            <item name="Angle" type_name="gh_bool" type_code="1">false</item>
                          </items>
                        </chunk>
                      </chunks>
                    </chunk>`;
  return chunk.replace(
    /<chunk name="param_input" index="0">[\s\S]*?<\/chunk>\s*<chunk name="param_output"/,
    `${paramInput}\n                    <chunk name="param_output"`,
  );
}

function patchColBox(chunk) {
  let c = chunk;
  c = c.replace(
    /(<item name="NickName" type_name="gh_string" type_code="10">N<\/item>[\s\S]*?<item name="string" type_name="gh_string" type_code="10">)[^<]+(<\/item>)/,
    '$1workpiece$2',
  );
  c = c.replace(
    /(<item name="NickName" type_name="gh_string" type_code="10">[XYZ]<\/item>[\s\S]*?<item name="number" type_name="gh_double" type_code="6">)0\.1(<\/item>)/g,
    '$10.03$2',
  );
  return c;
}

function patchAmplitude(chunk, value) {
  return chunk.replace(
    /(<chunk name="param_input" index="1">[\s\S]*?<chunk name="PersistentData">[\s\S]*?<item name="number" type_name="gh_double" type_code="6">)[^<]+(<\/item>)/,
    `$1${value}$2`,
  );
}

function patchCarryMerge(chunk) {
  if (!chunk.includes('99f317f0-8787-4392-98a0-50a0be43c2d1')) return chunk;
  let c = chunk.replace(
    /<item name="InputCount" type_name="gh_int32" type_code="3">2<\/item>/,
    '<item name="InputCount" type_name="gh_int32" type_code="3">3</item>',
  );
  c = c.replace(
    /(<chunk name="InputParam" index="0">[\s\S]*?<item name="Source" index="0" type_name="gh_guid" type_code="9">)[^<]+(<\/item>)/,
    `$1${SET_CLOSE_OUT}$2`,
  );
  c = c.replace(
    /(<chunk name="InputParam" index="1">[\s\S]*?<item name="Source" index="0" type_name="gh_guid" type_code="9">)[^<]+(<\/item>)/,
    `$1${LIN_SEG_OUT}$2`,
  );
  if (!c.includes('<chunk name="InputParam" index="2">')) {
    const thirdInput = `
                        <chunk name="InputParam" index="2">
                          <items count="9">
                            <item name="Access" type_name="gh_int32" type_code="3">2</item>
                            <item name="Description" type_name="gh_string" type_code="10">Data stream 3</item>
                            <item name="InstanceGuid" type_name="gh_guid" type_code="9">a1b2c3d4-e5f6-7890-abcd-ef1234567890</item>
                            <item name="Mutable" type_name="gh_bool" type_code="1">false</item>
                            <item name="Name" type_name="gh_string" type_code="10">Data 3</item>
                            <item name="NickName" type_name="gh_string" type_code="10">D3</item>
                            <item name="Optional" type_name="gh_bool" type_code="1">true</item>
                            <item name="Source" index="0" type_name="gh_guid" type_code="9">${SET_OPEN_OUT}</item>
                            <item name="SourceCount" type_name="gh_int32" type_code="3">1</item>
                          </items>
                          <chunks count="1">
                            <chunk name="Attributes">
                              <items count="2">
                                <item name="Bounds" type_name="gh_drawing_rectanglef" type_code="35">
                                  <X>775</X>
                                  <Y>512</Y>
                                  <W>21</W>
                                  <H>20</H>
                                </item>
                                <item name="Pivot" type_name="gh_drawing_pointf" type_code="31">
                                  <X>787</X>
                                  <Y>522</Y>
                                </item>
                              </items>
                            </chunk>
                          </chunks>
                        </chunk>`;
    c = c.replace('<chunk name="OutputParam" index="0">', `${thirdInput}\n                        <chunk name="OutputParam" index="0">`);
  } else {
    c = c.replace(
      /(<chunk name="InputParam" index="2">[\s\S]*?<item name="Source" index="0" type_name="gh_guid" type_code="9">)[^<]+(<\/item>)/,
      `$1${SET_OPEN_OUT}$2`,
    );
  }
  return c;
}

const parts = xml.split(/(?=<chunk name="Object")/);
let out = parts.map((chunk, i) => {
  if (i === 0) return chunk;

  const moveGuid = [...moves.keys()].find((g) => chunk.includes(g));
  if (moveGuid) {
    const type = moves.get(moveGuid);
    let c = chunk.replace(
      /(<item name="MotionType" type_name="gh_string" type_code="10">)[^<]+(<\/item>)/,
      `$1${type}$2`,
    );
    c = c.replace(
      /(<item name="string" type_name="gh_string" type_code="10">)PTP(<\/item>)/,
      `$1${type}$2`,
    );
    return c;
  }

  const ptGuid = [...points.keys()].find((g) => chunk.includes(g));
  if (ptGuid) return patchConstructPoint(chunk, points.get(ptGuid));

  if (chunk.includes(HOME_JOINTS_GUID)) return patchHomeJoints(chunk);

  if (chunk.includes('242880eb-18dd-45e1-977a-9bba6b756024')) return patchColBox(chunk);

  if (chunk.includes('99f317f0-8787-4392-98a0-50a0be43c2d1')) return patchCarryMerge(chunk);

  const ampGuid = [...amplitudes.keys()].find((g) => chunk.includes(g));
  if (ampGuid) return patchAmplitude(chunk, amplitudes.get(ampGuid));

  return chunk;
});

out = out.join('');
out = out.replace(
  /<item name="AutoPlan" type_name="gh_bool" type_code="1">false<\/item>/g,
  '<item name="AutoPlan" type_name="gh_bool" type_code="1">true</item>',
);

fs.writeFileSync(ghxPath, out);
console.log(`patched ${ghxPath}`);
