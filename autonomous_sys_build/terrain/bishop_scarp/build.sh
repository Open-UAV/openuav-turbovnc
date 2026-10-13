#!/bin/bash
# Regenerate the bishop_scarp Gazebo model from the Sep 2019 Bishop survey.
# Needs the Agisoft export (bishop.obj in WGS84 lon/lat/h + bishop.1001.jpg) in $BISHOP_SRC,
# internet access (register.py fetches USGS 3DEP), and python3 with numpy, opencv, pillow,
# pyproj. Takes ~4 min.
#
#   [BISHOP_SRC=dir] terrain/bishop_scarp/build.sh [out_dir]      default ~/twin-assets/bishop_scarp
#
# The output directory is the "terrain" named context for Dockerfile.twin.
set -euo pipefail
export BISHOP_SRC="${BISHOP_SRC:-/mnt/22tb-hdd/Samsung_T5_Dios/bishop2019/bishop/bishop-gray}"
export BISHOP_OUT="${1:-$HOME/twin-assets/bishop_scarp}"
cd "$(dirname "$0")"
rm -rf "$BISHOP_OUT"
python3 register.py
python3 build_model.py
python3 finish_model.py
du -sh "$BISHOP_OUT"/meshes/* "$BISHOP_OUT"
