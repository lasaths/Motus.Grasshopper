#!/usr/bin/env node
/**
 * Non-Rhino host-readiness check for Milestone 2.0:
 * - MotusNetPackages.props pin is the expected Motus.NET NuGet version
 * - if that version exists on nuget.org: docs must not claim unreleased / UseLocal-only
 * - if not yet on nuget.org: docs must acknowledge UseLocal until publish (pre-cut gate)
 */
import { readFileSync, existsSync } from "node:fs";
import { dirname, join } from "node:path";
import { fileURLToPath } from "node:url";

const root = join(dirname(fileURLToPath(import.meta.url)), "..");
const EXPECTED = "2.0.0";

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
  { re: /2\.0\.0\s*\(unreleased\)/i, label: "2.0.0 (unreleased)" },
  { re: /NuGet not published yet/i, label: "NuGet not published yet" },
  { re: /UseLocal until (that )?NuGet (is )?publish/i, label: "UseLocal until NuGet publish" },
  { re: /until that version is on nuget\.org/i, label: "until that version is on nuget.org" },
  { re: /required until NuGet publish/i, label: "required until NuGet publish" },
  { re: /UseLocal until published/i, label: "UseLocal until published" },
  { re: /until publication/i, label: "until publication" },
  { re: /NuGet publish pending/i, label: "NuGet publish pending" },
  { re: /UseLocal until nuget\.org/i, label: "UseLocal until nuget.org" },
];

const docFiles = [
  "AGENTS.md",
  "README.md",
  "examples/README.md",
  "docs/regression-matrix.md",
  "CHANGELOG.md",
];

const url = `https://api.nuget.org/v3-flatcontainer/motus.core/index.json`;
let versions;
try {
  const res = await fetch(url);
  if (!res.ok) fail(`nuget.org Motus.Core index HTTP ${res.status}`);
  versions = (await res.json()).versions ?? [];
} catch (e) {
  fail(`nuget.org fetch failed: ${e.message}`);
}

const onNuget = versions.includes(EXPECTED);

if (onNuget) {
  for (const rel of docFiles) {
    const text = read(rel);
    // Historical older pins may say "until NuGet publishes 0.15.0" — only flag current-cut section.
    const slice = rel === "CHANGELOG.md" ? text.split("\n## 1.9.0")[0] : text;
    for (const { re, label } of stalePatterns) {
      if (re.test(slice)) fail(`${rel} still claims "${label}" after ${EXPECTED} is on nuget.org`);
    }
  }
  console.log(`verify-motus-net-pin: OK — pin ${EXPECTED} on nuget.org; docs aligned; Yak unpublished (expected).`);
} else {
  // Pre-publish: require at least one doc to mention UseLocal / pending publish for this cut.
  const joined = docFiles.map(read).join("\n");
  const prePublishOk =
    /UseLocal until/i.test(joined) ||
    /NuGet publish pending/i.test(joined) ||
    /until that NuGet is published/i.test(joined) ||
    /until that version is on nuget\.org/i.test(joined);
  if (!prePublishOk) {
    fail(`pin ${EXPECTED} not on nuget.org yet, but docs do not acknowledge UseLocal / publish-pending`);
  }
  console.log(
    `verify-motus-net-pin: OK — pin ${EXPECTED} (not on nuget.org yet); docs acknowledge UseLocal until publish; Yak unpublished (expected).`,
  );
}

// Yak / Package Manager honesty: do not claim a published Package Manager release for 1.8/2.0.
const readme = read("README.md");
if (/Package Manager.*(1\.8|2\.0).*published/i.test(readme) || /yak\.rhino3d\.com\/packages\/motus/i.test(readme)) {
  fail("README must not claim Yak / Package Manager publish for 1.8 or 2.0 yet");
}
