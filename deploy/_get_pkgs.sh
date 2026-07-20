#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
# Populate /ros_ws/src/oomwoo-m1 either from the self-hosted git repo (default)
# or from the local build context copied to /tmp/local_src (USE_LOCAL=1). Kept
# in a script so the Dockerfile stays readable and the choice is one build-arg.
# Packages land inside the stock /ros_ws workspace, per oomwoo-install
# convention, so one `colcon build` covers stock + these.
set -euo pipefail

DEST=/ros_ws/src/oomwoo-m1

if [ "${USE_LOCAL:-0}" = "1" ]; then
    echo "[_get_pkgs] using local build context"
    mkdir -p "$DEST"
    cp -r /tmp/local_src/oomwoo_coverage \
          /tmp/local_src/oomwoo_nav_localize \
          /tmp/local_src/oomwoo_sim_support "$DEST/"
else
    echo "[_get_pkgs] cloning ${PKG_REPO} (${PKG_BRANCH})"
    git clone --depth 1 -b "${PKG_BRANCH}" "${PKG_REPO}" "$DEST"
fi

echo "[_get_pkgs] packages present:"
find "$DEST" -maxdepth 3 -name package.xml -printf '  %h\n'
