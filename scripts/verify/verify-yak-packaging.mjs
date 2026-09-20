#!/usr/bin/env node
/**
 * Yak packaging readiness (no Rhino / no yak CLI required).
 *
 * - Identity surfaces agree with each other (whatever SemVer is currently on the tree)
 * - manifest + icon anatomy ready for McNeel Package Manager
 * - Docs still state first public Yak = 2.0.0 (do not claim published)
 *
 * Does NOT bump or require 1.8 / 2.0 SemVer — safe to run while Milestone 1.8 is in flight.
 */
import { readFileSync, existsSync } from "node:fs";
import { dirname, join } from "node:path";
import { fileURLToPath } from "node:url";

const root = join(dirname(fileURLToPath(import.meta.url)), "../..");
const FIRST_PUBLIC_YAK = "2.0.0";

function fail(msg) {
  console.error(`verify-yak-packaging: ${msg}`);
  process.exit(1);
}

function read(rel) {
  const p = join(root, rel);
  if (!existsSync(p)) fail(`missing ${rel}`);
  return readFileSync(p, "utf8");
}

function parseCsprojVersion(text) {
  const m = text.match(/<Version>([^<]+)<\/Version>/);
  if (!m) fail("could not parse <Version> from Motus.GH.csproj");
  return m[1].trim();
}

function parsePluginVersion(text) {
  const m = text.match(/public override string Version\s*=>\s*"([^"]+)"/);
  if (!m) fail("could not parse MotusGhPlugin.Version string");
  return m[1].trim();
}

function parseMotusNetVersion(text) {
  const m = text.match(
    /<MotusNetVersion Condition="'\$\(MotusNetVersion\)' == ''">([^<]+)<\/MotusNetVersion>/,
  );
  if (!m) fail("could not parse MotusNetVersion from MotusNetPackages.props");
  return m[1].trim();
}

function parseManifestVersion(text) {
  const m = text.match(/^version:\s*(.+)$/m);
  if (!m) fail("could not parse version from packaging/yak/manifest.yml");
  return m[1].trim();
}

const csproj = read("src/Motus.GH/Motus.GH.csproj");
const plugin = read("src/Motus.GH/MotusGhPlugin.cs");
const props = read("build/MotusNetPackages.props");
const manifest = read("packaging/yak/manifest.yml");

const vCsproj = parseCsprojVersion(csproj);
const vPlugin = parsePluginVersion(plugin);
const vNet = parseMotusNetVersion(props);
const vManifest = parseManifestVersion(manifest);

if (vCsproj !== vPlugin) {
  fail(`Motus.GH.csproj Version (${vCsproj}) != MotusGhPlugin.Version (${vPlugin})`);
}
if (vCsproj !== vManifest) {
  fail(
    `Motus.GH.csproj Version (${vCsproj}) != packaging/yak/manifest.yml (${vManifest}) — keep template in sync (build.ps1 -Yak also rewrites stage copy)`,
  );
}
if (vCsproj !== vNet) {
  fail(
    `plugin Version (${vCsproj}) != MotusNetVersion pin (${vNet}) — Yak packs NuGet Motus.NET; pin must match ship identity`,
  );
}

if (!/^name:\s*motus\s*$/m.test(manifest)) fail("manifest name must be motus");
if (!/^icon:\s*icon\.png\s*$/m.test(manifest)) fail("manifest must declare icon: icon.png");
if (!/^authors:/m.test(manifest)) fail("manifest missing authors");
if (!/^description:/m.test(manifest)) fail("manifest missing description");
if (!/url:\s*https:\/\/github\.com\/lasaths\/Motus\.Grasshopper/.test(manifest)) {
  fail("manifest url must point at Motus.Grasshopper");
}
if (!existsSync(join(root, "packaging/yak/icon.png"))) fail("missing packaging/yak/icon.png");
if (!existsSync(join(root, "LICENSE"))) fail("missing LICENSE (staged into .yak)");

const buildPs1 = read("build.ps1");
if (!/Resolve-YakExe|Contents\/Resources\/bin\/yak|net8\.0-windows/.test(buildPs1)) {
  fail("build.ps1 Yak path looks incomplete (expected dual-TFM + yak resolve)");
}
if (!existsSync(join(root, "scripts/pack-yak.sh"))) {
  fail("missing scripts/pack-yak.sh (Mac / no-pwsh pack path)");
}

const docs = ["README.md", "AGENTS.md", "packaging/yak/README.md", "docs/regression-matrix.md"];
for (const rel of docs) {
  const text = read(rel);
  if (!new RegExp(`2\\.0\\.0|first public Yak`, "i").test(text)) {
    fail(`${rel} must mention first public Yak / 2.0.0 policy`);
  }
  if (/yak\.rhino3d\.com\/packages\/motus/i.test(text) && /published|live|available/i.test(text)) {
    fail(`${rel} must not claim production Yak publish before ${FIRST_PUBLIC_YAK}`);
  }
}

const isGa = vCsproj === FIRST_PUBLIC_YAK;
const status = isGa
  ? `GA identity ${FIRST_PUBLIC_YAK} — production yak push allowed after auth + clean pack`
  : `pre-GA identity ${vCsproj} — pack/test-server OK; do NOT push production Yak until ${FIRST_PUBLIC_YAK}`;

console.log(`verify-yak-packaging: OK — ${status}`);
console.log(`  csproj/plugin/manifest/MotusNetVersion = ${vCsproj}`);
console.log(`  icon + dual-TFM pack scripts present`);
