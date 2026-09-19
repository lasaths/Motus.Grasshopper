#!/usr/bin/env node
/**
 * Non-Rhino host-readiness check for Milestone 1.8:
 * - MotusNetPackages.props pin is the expected Motus.NET NuGet version
 * - that version exists on nuget.org
 * - key docs no longer claim NuGet unreleased / UseLocal-only for the pin
 */
import { readFileSync, existsSync } from "node:fs";
import { dirname, join } from "node:path";
import { fileURLToPath } from "node:url";

const root = join(dirname(fileURLToPath(import.meta.url)), "..");
const EXPECTED = "0.17.0";

function fail(msg) {
  console.error(`verify-motus-net-pin: ${msg}`);
  process.exit(1);
}

function read(rel) {
  const p = join(root, rel);
  if (!existsSync(p)) fail(`missing ${rel}`);
  return readFileSync(p, "utf8");
}

const props = read("build/MotusNetPackages.props");
const m = props.match(/<MotusNetVersion Condition="'\$\(MotusNetVersion\)' == ''">([^<]+)<\/MotusNetVersion>/);
if (!m) fail("could not parse MotusNetVersion from build/MotusNetPackages.props");
if (m[1].trim() !== EXPECTED) {
  fail(`MotusNetPackages.props pin is ${m[1].trim()}, expected ${EXPECTED}`);
}

const stalePatterns = [
  { re: /0\.17\.0\s*\(unreleased\)/i, label: "0.17.0 (unreleased)" },
  { re: /NuGet not published yet/i, label: "NuGet not published yet" },
  { re: /UseLocal until (that )?NuGet (is )?publish/i, label: "UseLocal until NuGet publish" },
  { re: /until that version is on nuget\.org/i, label: "until that version is on nuget.org" },
  { re: /required until NuGet publish/i, label: "required until NuGet publish" },
  { re: /UseLocal until published/i, label: "UseLocal until published" },
  { re: /until publication/i, label: "until publication" },
];

const docFiles = [
  "AGENTS.md",
  "README.md",
  "examples/README.md",
  "docs/regression-matrix.md",
  "CHANGELOG.md",
];

for (const rel of docFiles) {
  const text = read(rel);
  // CHANGELOG historical entries for older pins may say "until NuGet publishes 0.15.0" — only flag 0.17-era stale claims.
  const slice = rel === "CHANGELOG.md" ? text.split("\n## 0.16.0")[0] : text;
  for (const { re, label } of stalePatterns) {
    if (re.test(slice)) fail(`${rel} still claims "${label}"`);
  }
}

// Yak / Package Manager honesty: do not claim a published Package Manager release for 0.17/2.0.
const readme = read("README.md");
if (/Package Manager.*(0\.17|2\.0).*published/i.test(readme) || /yak\.rhino3d\.com\/packages\/motus/i.test(readme)) {
  fail("README must not claim Yak / Package Manager publish for 0.17 or 2.0 yet");
}

const url = `https://api.nuget.org/v3-flatcontainer/motus.core/index.json`;
let versions;
try {
  const res = await fetch(url);
  if (!res.ok) fail(`nuget.org Motus.Core index HTTP ${res.status}`);
  versions = (await res.json()).versions ?? [];
} catch (e) {
  fail(`nuget.org fetch failed: ${e.message}`);
}

if (!versions.includes(EXPECTED)) {
  fail(`nuget.org Motus.Core versions do not include ${EXPECTED} (got ${versions.slice(-5).join(", ")}…)`);
}

console.log(`verify-motus-net-pin: OK — pin ${EXPECTED} on nuget.org; docs aligned; Yak unpublished (expected).`);
