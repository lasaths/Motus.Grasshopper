#!/usr/bin/env bash
# Stage dual-TFM Motus.GH outputs into a Yak package without pwsh.
# Prefer: pwsh ./build.ps1 -Configuration Release -Yak
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
CONFIG="${CONFIGURATION:-Release}"
DIST="$ROOT/dist"
STAGE="${TMPDIR:-/tmp}/motus-yak-stage-$$"
BIN="$ROOT/src/Motus.GH/bin/$CONFIG"

resolve_yak() {
  if command -v yak >/dev/null 2>&1; then
    command -v yak
    return
  fi
  local candidates=()
  if [[ -n "${Rhino8App:-}" ]]; then
    candidates+=("${Rhino8App}/Contents/Resources/bin/yak")
  fi
  candidates+=(
    "/Applications/Rhino 8.app/Contents/Resources/bin/yak"
    "/Volumes/Storage/00_Applications/Rhino 8.app/Contents/Resources/bin/yak"
  )
  local c
  for c in "${candidates[@]}"; do
    if [[ -x "$c" ]]; then
      echo "$c"
      return
    fi
  done
  echo "yak not found — install Rhino 8 or add yak to PATH (set Rhino8App for custom Mac installs)." >&2
  exit 1
}

version_from_csproj() {
  sed -n 's/.*<Version>\([^<]*\)<\/Version>.*/\1/p' "$ROOT/src/Motus.GH/Motus.GH.csproj" | head -1
}

plugin_version() {
  sed -n 's/.*public override string Version => "\([^"]*\)".*/\1/p' "$ROOT/src/Motus.GH/MotusGhPlugin.cs" | head -1
}

VERSION="$(version_from_csproj)"
PLUGIN="$(plugin_version)"
if [[ -z "$VERSION" ]]; then
  echo "Could not read <Version> from Motus.GH.csproj" >&2
  exit 1
fi
if [[ "$VERSION" != "$PLUGIN" ]]; then
  echo "Version mismatch: csproj=$VERSION MotusGhPlugin=$PLUGIN" >&2
  exit 1
fi

for tfm in net8.0-windows net8.0; do
  if [[ ! -f "$BIN/$tfm/Motus.GH.gha" ]]; then
    echo "Missing $BIN/$tfm/Motus.GH.gha — build both TFMs (Release) before packing." >&2
    exit 1
  fi
done

YAK="$(resolve_yak)"
rm -rf "$STAGE"
mkdir -p "$STAGE" "$DIST"

stage_tfm() {
  local tfm="$1"
  local dest="$STAGE/$tfm"
  mkdir -p "$dest/resources"
  cp -f "$BIN/$tfm/Motus.GH.gha" "$dest/"
  cp -f "$BIN/$tfm"/Motus.*.dll "$dest/" 2>/dev/null || true
  cp -R "$BIN/$tfm/resources/"* "$dest/resources/" 2>/dev/null || true
}

stage_tfm net8.0-windows
stage_tfm net8.0

cp -f "$ROOT/LICENSE" "$STAGE/LICENSE"
cp -f "$ROOT/packaging/yak/icon.png" "$STAGE/icon.png"
# Rewrite version from csproj (manifest.yml is a template).
sed "s/^version:.*/version: ${VERSION}/" "$ROOT/packaging/yak/manifest.yml" > "$STAGE/manifest.yml"

(
  cd "$STAGE"
  "$YAK" build
)

shopt -s nullglob
moved=0
for yakfile in "$STAGE"/*.yak; do
  base="$(basename "$yakfile")"
  mv -f "$yakfile" "$DIST/$base"
  echo "Yak package (Win+Mac): $DIST/$base"
  moved=1
done
rm -rf "$STAGE"

if [[ "$moved" -eq 0 ]]; then
  echo "yak build produced no .yak file" >&2
  exit 1
fi

if [[ "$VERSION" != "2.0.0" ]]; then
  echo
  echo "Pre-GA pack ($VERSION): OK for local/test-server."
  echo "Do NOT push to production Yak until Version is 2.0.0 (first public Package Manager release)."
  echo "Test server: yak push --source https://test.yak.rhino3d.com dist/motus-${VERSION}-*.yak"
else
  echo
  echo "GA pack 2.0.0 — after auth: yak push dist/motus-2.0.0-*.yak"
fi
