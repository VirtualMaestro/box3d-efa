#!/usr/bin/env bash
# Fetch upstream Box3D source at the SHA pinned in scripts/versions.json,
# then apply scripts/box3d-simd.patch (wasm SIMD flags only when
# BOX3D_DISABLE_SIMD is off, which the scalar and legacy flavours need).
# Idempotent: skips the fetch when deps/box3d is already at the pin, and the
# patch when it is already applied.
set -euo pipefail

DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" >/dev/null 2>&1 && pwd)"
ROOT="$(dirname "$DIR")"

# The path goes in as an argument, so Git Bash (MSYS) converts it for a Windows node.
SHA="$(node -p "require(process.argv[1]).box3d.sha" "$DIR/versions.json")"
REPO="$(node -p "require(process.argv[1]).box3d.repo" "$DIR/versions.json")"
DEST="$ROOT/deps/box3d"
PATCH="$DIR/box3d-simd.patch"

if [ -d "$DEST/.git" ] && [ "$(git -C "$DEST" rev-parse HEAD 2>/dev/null)" = "$SHA" ]; then
  echo "box3d already at $SHA"
else
  rm -rf "$DEST"
  mkdir -p "$DEST"
  git -C "$DEST" init -q
  git -C "$DEST" remote add origin "$REPO"
  git -C "$DEST" fetch -q --depth 1 origin "$SHA"
  git -C "$DEST" checkout -q FETCH_HEAD
  echo "box3d fetched at $SHA"
fi

if git -C "$DEST" apply --reverse --check "$PATCH" 2>/dev/null; then
  echo "box3d-simd.patch already applied"
else
  git -C "$DEST" apply "$PATCH"
  echo "box3d-simd.patch applied"
fi
