"""Register the Agisoft Bishop mesh against the USGS 3DEP 1 m lidar DEM (translation only).

Writes $BISHOP_OUT/registration.json: the horizontal shift that best fits the lidar and the
vertical offset (mesh height minus NAVD88), which build_model.py removes.
"""
import io
import json
import math
import os
import urllib.request

import numpy as np
from PIL import Image
from pyproj import Transformer

SRC_DIR = os.environ.get("BISHOP_SRC", "/mnt/22tb-hdd/Samsung_T5_Dios/bishop2019/bishop/bishop-gray")
GEO = os.path.join(SRC_DIR, "bishop.obj")
OUT = os.environ.get("BISHOP_OUT", os.path.expanduser("~/twin-assets/bishop_scarp"))
RES = 0.5
PAD = 25.0

os.makedirs(OUT, exist_ok=True)
to_utm = Transformer.from_crs(4326, 32611, always_xy=True)
v = []
with open(GEO) as f:
    for line in f:
        if line.startswith("v "):
            p = line.split()
            v.append((float(p[1]), float(p[2]), float(p[3])))
v = np.array(v)
E, N = to_utm.transform(v[:, 0], v[:, 1])
Z = v[:, 2]

# mesh DEM on a UTM grid (highest vertex per cell = top surface)
e0, n1 = E.min() - PAD, N.max() + PAD
W = int((E.max() + PAD - e0) / RES)
H = int((n1 - (N.min() - PAD)) / RES)
mesh = np.full((H, W), np.nan)
ci = ((n1 - N) / RES).astype(int)
cj = ((E - e0) / RES).astype(int)
o = np.argsort(Z)
mesh[ci[o], cj[o]] = Z[o]

bbox = f"{e0},{n1 - H * RES},{e0 + W * RES},{n1}"
url = ("https://elevation.nationalmap.gov/arcgis/rest/services/3DEPElevation/ImageServer/exportImage?"
       f"bbox={bbox}&bboxSR=32611&imageSR=32611&size={W},{H}&format=tiff&pixelType=F32"
       "&interpolation=RSP_BilinearInterpolation&noDataInterpretation=esriNoDataMatchAny&f=image")
lidar = np.array(Image.open(io.BytesIO(urllib.request.urlopen(url, timeout=120).read())), dtype=np.float64)


def score(dx, dy):
    """mesh shifted by (dx east, dy north) metres -> (rms after removing the median dz, dz, cells)."""
    sj, si = int(round(dx / RES)), int(round(-dy / RES))
    m = np.full_like(mesh, np.nan)
    m[max(si, 0):H + min(si, 0), max(sj, 0):W + min(sj, 0)] = \
        mesh[max(-si, 0):H + min(-si, 0), max(-sj, 0):W + min(-sj, 0)]
    d = (m - lidar)[np.isfinite(m - lidar)]
    dz = np.median(d)
    return math.sqrt(np.mean((d - dz) ** 2)), dz, d.size


best = min(((score(dx, dy), dx, dy) for dx in np.arange(-20, 20.01, RES) for dy in np.arange(-20, 20.01, RES)),
           key=lambda r: r[0][0])
(r0, dz0, _), ((rb, dzb, nb), dxb, dyb) = score(0, 0), best
reg = {
    "as_is": {"rms_m": round(r0, 3), "dz_m": round(dz0, 3)},
    "best_shift": {"east_m": float(dxb), "north_m": float(dyb), "dz_m": round(float(dzb), 3),
                   "rms_m": round(rb, 3), "cells": int(nb)},
    "dz": float(dzb),
    "lon_c": float((v[:, 0].min() + v[:, 0].max()) / 2),
    "lat_c": float((v[:, 1].min() + v[:, 1].max()) / 2),
    "h_c": float((Z.min() + Z.max()) / 2),
}
json.dump(reg, open(f"{OUT}/registration.json", "w"), indent=1)
print(json.dumps(reg, indent=1))
