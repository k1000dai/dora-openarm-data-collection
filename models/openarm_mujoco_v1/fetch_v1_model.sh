#!/usr/bin/env bash
# Copyright 2026 Enactic, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# Fetch the OpenArm v1 MuJoCo mesh assets from upstream.
#
# The patched MJCF files (openarm_bimanual.xml, scene.xml) are committed to this
# repo; the large binary meshes/ tree is not, so this script downloads it from
# the pinned upstream ref. The meshes are what MuJoCo loads via the XML's
# meshdir="meshes". Re-run this after a fresh checkout before running the v1
# dataflow.

set -euo pipefail

REPO="https://github.com/enactic/openarm_mujoco.git"
# Pinned to the upstream commit the vendored XMLs were derived from.
REF="9eadf86d5b9a0713fdc097019302e02e4b303083"

DEST="$(cd "$(dirname "$0")" && pwd)"
TMP="$(mktemp -d)"
trap 'rm -rf "$TMP"' EXIT

echo "Fetching OpenArm v1 meshes @ ${REF} ..."
git clone --filter=blob:none --sparse "$REPO" "$TMP/repo"
git -C "$TMP/repo" sparse-checkout set v1/meshes
git -C "$TMP/repo" checkout --quiet "$REF"

rm -rf "$DEST/meshes"
cp -r "$TMP/repo/v1/meshes" "$DEST/meshes"

echo "Done. v1 meshes installed at: $DEST/meshes"
