#!/usr/bin/env bash
# Build Grasshopper plugin (Motus.NET from NuGet or sibling repo).
set -euo pipefail

ROOT="$(cd "$(dirname "$0")" && pwd)"
CONFIG="${1:-Release}"
SLN="$ROOT/Motus.Grasshopper.slnx"

if [[ "$(uname -s)" == "Darwin" ]]; then
  OUT="$ROOT/src/Motus.GH/bin/$CONFIG/net8.0"
else
  OUT="$ROOT/src/Motus.GH/bin/$CONFIG/net8.0-windows"
fi

echo "Building Motus.Grasshopper ($CONFIG)..."
dotnet restore "$SLN" --force-evaluate
dotnet build "$SLN" -c "$CONFIG" --no-restore

echo
echo "Built: $OUT/Motus.GH.gha"
echo "Copy to Grasshopper Libraries/Motus:"
echo "  Motus.GH.gha"
echo "  Motus.*.dll (from NuGet)"
echo "  resources/robots/ (from Motus.Presets package)"

if [[ "${INSTALL:-}" == "1" ]]; then
  GH_LIB="$HOME/Library/Application Support/McNeel/Rhinoceros/8.0/Plug-ins/Grasshopper/Libraries/Motus"
  mkdir -p "$GH_LIB/resources"
  cp -f "$OUT"/Motus.GH.gha "$GH_LIB/"
  cp -f "$OUT"/Motus.*.dll "$GH_LIB/" 2>/dev/null || true
  cp -R "$OUT/resources/"* "$GH_LIB/resources/" 2>/dev/null || true
  echo
  echo "Installed to: $GH_LIB"
  echo "Restart Grasshopper (or Rhino) to load Motus."
fi
