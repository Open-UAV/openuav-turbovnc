"""Visual and collision meshes of the Bishop fault scarp from the georeferenced Agisoft export.

Frame: ENU tangent plane at the chosen home point (the world's spherical_coordinates), x = east,
y = north, z = up, z = 0 at the home ground surface. Heights lose the offset register.py measured
against USGS 3DEP, so the world elevation is metres above NAVD88 (~ AMSL).

Writes into $BISHOP_OUT: meshes/bishop_scarp.obj (with vertex normals, which ogre2 PBR needs),
meshes/bishop_scarp_collision.stl (0.5 m grid with a flat 1.5 m pad at home), home.json, model.config.
"""
import json
import math
import os
import struct

import numpy as np
from pyproj import Transformer

SRC_DIR = os.environ.get("BISHOP_SRC", "/mnt/22tb-hdd/Samsung_T5_Dios/bishop2019/bishop/bishop-gray")
SRC = os.path.join(SRC_DIR, "bishop.obj")
OUT = os.environ.get("BISHOP_OUT", os.path.expanduser("~/twin-assets/bishop_scarp"))
REG = json.load(open(f"{OUT}/registration.json"))
PILLAR = (5.0, 0.0)   # terrain_mapping.launch.py puts the AprilTag pillar at x = 5, y = 0
GRID = 0.5
PAD_HALF = 0.75       # flat 1.5 m square under the vehicle at home (collision only)

V, VT, F = [], [], []
with open(SRC) as f:
    for line in f:
        if line.startswith("v "):
            p = line.split(); V.append((float(p[1]), float(p[2]), float(p[3])))
        elif line.startswith("vt "):
            p = line.split(); VT.append((float(p[1]), float(p[2])))
        elif line.startswith("f "):
            F.append([int(i) for c in line.split()[1:] for i in c.split("/")[:2]])
V, VT, F = np.array(V), np.array(VT), np.array(F, dtype=np.int64)  # F: v1 t1 v2 t2 v3 t3 (1-based)
print("parsed", len(V), "v", len(VT), "vt", len(F), "f")

geod2ecef = Transformer.from_crs(4979, 4978, always_xy=True)
ecef2geod = Transformer.from_crs(4978, 4979, always_xy=True)


def enu_rot(lon0, lat0):
    lo, la = math.radians(lon0), math.radians(lat0)
    return np.array([[-math.sin(lo), math.cos(lo), 0],
                     [-math.sin(la) * math.cos(lo), -math.sin(la) * math.sin(lo), math.cos(la)],
                     [math.cos(la) * math.cos(lo), math.cos(la) * math.sin(lo), math.sin(la)]])


def enu(lon0, lat0, h0):
    x, y, z = geod2ecef.transform(V[:, 0], V[:, 1], V[:, 2])
    x0, y0, z0 = geod2ecef.transform(lon0, lat0, h0)
    return (enu_rot(lon0, lat0) @ np.vstack([x - x0, y - y0, z - z0])).T


def grid(P):
    """Top-surface DEM; vertices sit at odd multiples of GRID/2 so the pad edges land on +-0.75 m."""
    x0 = (math.floor(P[:, 0].min() / GRID - 0.5) + 0.5) * GRID
    y1 = (math.ceil(P[:, 1].max() / GRID - 0.5) + 0.5) * GRID
    W = int(round((P[:, 0].max() - x0) / GRID)) + 2
    H = int(round((y1 - P[:, 1].min()) / GRID)) + 2
    d = np.full((H, W), np.nan)
    i = np.floor((y1 - P[:, 1]) / GRID + 0.5).astype(int)
    j = np.floor((P[:, 0] - x0) / GRID + 0.5).astype(int)
    o = np.argsort(P[:, 2])
    d[i[o], j[o]] = P[o, 2]
    return d, x0, y1


# ---------- pick home: flat, the pillar spot level with it, 25 m of scan all around
P = enu(REG["lon_c"], REG["lat_c"], REG["h_c"])
dem, x0, y1 = grid(P)
H, W = dem.shape
k = int(3 / GRID) // 2
rough = np.std(np.lib.stride_tricks.sliding_window_view(np.pad(dem, k, constant_values=np.nan),
                                                        (2 * k + 1, 2 * k + 1)), axis=(-1, -2))
m = int(25 / GRID)
cover = np.lib.stride_tricks.sliding_window_view(np.pad(np.isfinite(dem), m, constant_values=False),
                                                 (2 * m + 1, 2 * m + 1)).all(axis=(-1, -2))
pj = int(PILLAR[0] / GRID)
score = np.full(dem.shape, np.inf)
ok = cover & np.isfinite(rough)
ok[:, W - pj:] = False
sh = np.full(dem.shape, np.nan); sh[:, :W - pj] = rough[:, pj:]
dz = np.full(dem.shape, np.nan); dz[:, :W - pj] = dem[:, pj:] - dem[:, :W - pj]
score[ok] = (rough + sh + np.abs(dz))[ok]
score[~np.isfinite(score)] = np.inf
i, j = np.unravel_index(np.argmin(score), score.shape)
hx, hy = x0 + j * GRID, y1 - i * GRID
hz = float(np.nanmedian(dem[i - 2:i + 3, j - 2:j + 3]))
c0 = np.array(geod2ecef.transform(REG["lon_c"], REG["lat_c"], REG["h_c"]))
lon_h, lat_h, h_h = ecef2geod.transform(*(c0 + enu_rot(REG["lon_c"], REG["lat_c"]).T @ np.array([hx, hy, hz])))
amsl = h_h - REG["dz"]
print(f"home lat {lat_h:.7f} lon {lon_h:.7f} AMSL {amsl:.2f}  rough {rough[i, j]:.3f}  pillar dz {dz[i, j]:+.2f}")

# ---------- final frame about home
P = enu(lon_h, lat_h, h_h)
os.makedirs(f"{OUT}/meshes", exist_ok=True)

fv = F[:, [0, 2, 4]] - 1
n = np.cross(P[fv[:, 1]] - P[fv[:, 0]], P[fv[:, 2]] - P[fv[:, 0]])   # area-weighted
N = np.zeros_like(P)
for c in range(3):
    np.add.at(N, fv[:, c], n)
N /= np.linalg.norm(N, axis=1, keepdims=True) + 1e-12
with open(f"{OUT}/meshes/bishop_scarp.obj", "w") as f:
    f.write("# Bishop CA fault scarp, Sep 2019 (Agisoft Metashape, MicaSense RedEdge)\n")
    f.write(f"# ENU metres about lat {lat_h:.7f} lon {lon_h:.7f}; z = 0 at {amsl:.2f} m NAVD88\n")
    np.savetxt(f, P, fmt="v %.4f %.4f %.4f")
    np.savetxt(f, VT, fmt="vt %.6f %.6f")
    np.savetxt(f, N, fmt="vn %.4f %.4f %.4f")
    np.savetxt(f, np.column_stack([F[:, 0], F[:, 1], F[:, 0], F[:, 2], F[:, 3], F[:, 2], F[:, 4], F[:, 5], F[:, 4]]),
               fmt="f %d/%d/%d %d/%d/%d %d/%d/%d")

# ---------- collision grid: holes filled from neighbours, pad flattened, binary STL
dem, gx0, gy1 = grid(P)
for _ in range(6):
    nan = np.isnan(dem)
    if not nan.any():
        break
    p = np.pad(dem, 1, constant_values=np.nan)
    nb = np.stack([p[:-2, 1:-1], p[2:, 1:-1], p[1:-1, :-2], p[1:-1, 2:]])
    with np.errstate(invalid="ignore"):
        fill = np.nanmean(nb, axis=0)
    dem[nan] = fill[nan]
Hh, Ww = dem.shape
X = gx0 + np.arange(Ww) * GRID
Y = gy1 - np.arange(Hh) * GRID
pad = (np.abs(Y)[:, None] <= PAD_HALF + 1e-6) & (np.abs(X)[None, :] <= PAD_HALF + 1e-6)
print("pad vertices", int(pad.sum()), "max |z| flattened %.3f m" % np.nanmax(np.abs(dem[pad])))
dem[pad] = 0.0
tris = []
for i in range(Hh - 1):
    for j in range(Ww - 1):
        q = dem[i:i + 2, j:j + 2]
        if np.isnan(q).any():
            continue
        a, b = (X[j], Y[i], q[0, 0]), (X[j + 1], Y[i], q[0, 1])
        c, d = (X[j], Y[i + 1], q[1, 0]), (X[j + 1], Y[i + 1], q[1, 1])
        tris += [(a, c, b), (b, c, d)]
with open(f"{OUT}/meshes/bishop_scarp_collision.stl", "wb") as f:
    f.write(b"bishop scarp collision, flat 1.5 m pad at home".ljust(80, b" "))
    f.write(struct.pack("<I", len(tris)))
    for t in tris:
        f.write(struct.pack("<12fH", 0, 0, 0, *t[0], *t[1], *t[2], 0))
print("collision tris", len(tris))

home = {"lat": lat_h, "lon": lon_h, "amsl": amsl, "zmin": float(P[:, 2].min()), "zmax": float(P[:, 2].max()),
        "xmin": float(P[:, 0].min()), "xmax": float(P[:, 0].max()),
        "ymin": float(P[:, 1].min()), "ymax": float(P[:, 1].max())}
json.dump(home, open(f"{OUT}/home.json", "w"), indent=1)
with open(f"{OUT}/model.config", "w") as f:
    f.write(f"""<?xml version="1.0"?>
<model>
  <name>bishop_scarp</name>
  <version>1.0</version>
  <sdf version="1.8">model.sdf</sdf>
  <author><name>DREAMS Lab</name></author>
  <description>Bishop CA fault scarp (Volcanic Tablelands), Sep 2019 survey. ENU about lat {lat_h:.7f}
    lon {lon_h:.7f}; z = 0 is the ground at that point, {amsl:.2f} m NAVD88. Pair with a world whose
    spherical_coordinates use the same point (set_world.py).</description>
</model>
""")
print(json.dumps(home, indent=1))
