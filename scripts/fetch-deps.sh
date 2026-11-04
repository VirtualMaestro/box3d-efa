#!/usr/bin/env bash
# Fetch Box3D's source at the SHA pinned in scripts/versions.json. Idempotent: skips the fetch when deps/box3d is
# already at the pin.
set -euo pipefail

DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" >/dev/null 2>&1 && pwd)"
ROOT="$(dirname "$DIR")"

# The path goes in as an argument, so Git Bash (MSYS) converts it for a Windows node.
SHA="$(node -p "require(process.argv[1]).box3d.sha" "$DIR/versions.json")"
REPO="$(node -p "require(process.argv[1]).box3d.repo" "$DIR/versions.json")"
DEST="$ROOT/deps/box3d"

if [ -d "$DEST/.git" ] && [ "$(git -C "$DEST" rev-parse HEAD 2>/dev/null)" = "$SHA" ]; then
  # The same SHA with changed files would build something other than the pin.
  if [ -n "$(git -C "$DEST" status --porcelain)" ]; then
    echo "deps/box3d has local changes; the build must use the pin as it is. Remove deps/box3d to fetch it again." >&2
    exit 1
  fi
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
