"""Camera-friendly textures and model.sdf for the Bishop scarp (run after build_model.py).

Writes into $BISHOP_OUT:
  meshes/bishop_scarp_albedo.jpg   8192^2 RGB: scan texture upsampled, CLAHE, fixed fine detail, desert colours
  meshes/ground_gravel.jpg         2048^2 seamless gravel, one tile per 16 m
  meshes/ground.obj                600 x 600 m quad with tiled UVs (visual only, above the world's ground plane)
  model.sdf
The detail is seeded, so rebuilds are identical and features stay fixed for SLAM.
"""
import json
import os

import cv2
import numpy as np

SRC_DIR = os.environ.get("BISHOP_SRC", "/mnt/22tb-hdd/Samsung_T5_Dios/bishop2019/bishop/bishop-gray")
SRC_TEX = os.path.join(SRC_DIR, "bishop.1001.jpg")
OUT = os.environ.get("BISHOP_OUT", os.path.expanduser("~/twin-assets/bishop_scarp"))
HOME = json.load(open(f"{OUT}/home.json"))
SPAWN_Z = -1.5        # terrain_mapping.launch.py spawns the terrain model at z = -1.5
GROUND_Z = -1.95      # 5 cm above the world's ground plane (set_world.py puts it at -2.0)
TILE_M = 16.0
rng = np.random.default_rng(2019)

# desert colour map: shadow / shrub -> soil -> pale tuff
stops = [(0.00, (38, 33, 28)), (0.30, (88, 80, 58)), (0.55, (150, 128, 96)), (0.80, (200, 178, 140)),
         (1.00, (236, 224, 198))]
lut = np.zeros((256, 3), np.uint8)
for ch in range(3):
    lut[:, ch] = np.interp(np.linspace(0, 1, 256), [s[0] for s in stops], [s[1][ch] for s in stops])


def blur(x, sigma, wrap):
    """Gaussian blur; periodic (FFT) when wrap, so tiles are seamless."""
    if not wrap:
        return cv2.GaussianBlur(x, (0, 0), sigma, borderType=cv2.BORDER_REFLECT)
    fy = np.fft.fftfreq(x.shape[0])[:, None]
    fx = np.fft.fftfreq(x.shape[1])[None, :]
    return np.real(np.fft.ifft2(np.fft.fft2(x) * np.exp(-2 * (np.pi * sigma) ** 2 * (fx ** 2 + fy ** 2)))).astype(np.float32)


def detail(shape, pebbles_per_px, wrap=False):
    """Multi-octave noise plus sparse pebbles, zero mean, in grey levels."""
    h, w = shape
    d = np.zeros(shape, np.float32)
    for sigma, amp in ((0.8, 9.0), (2.0, 8.0), (5.0, 6.0), (14.0, 5.0)):
        n = blur(rng.standard_normal(shape).astype(np.float32), sigma, wrap)
        d += amp * n / (n.std() + 1e-6)
    peb = np.zeros(shape, np.float32)
    k = int(h * w * pebbles_per_px)
    xs, ys = rng.integers(0, w, k), rng.integers(0, h, k)
    rs = rng.integers(1, 5, k)
    vs = rng.choice([-1, 1], k) * rng.uniform(18, 55, k)
    for x, y, r, v in zip(xs.tolist(), ys.tolist(), rs.tolist(), vs.tolist()):
        cv2.circle(peb, (x, y), r, v, -1, lineType=cv2.LINE_AA)
        if wrap:
            for dx in (-w, 0, w):
                for dy in (-h, 0, h):
                    if (dx or dy) and -r <= x + dx < w + r and -r <= y + dy < h + r:
                        cv2.circle(peb, (x + dx, y + dy), r, v, -1, lineType=cv2.LINE_AA)
    return d + blur(peb, 0.6, wrap)


def colourise(lum, warm):
    rgb = lut[np.clip(lum, 0, 255).astype(np.uint8)].astype(np.float32)
    rgb[..., 0] *= 1 + 0.06 * warm      # slight large-scale hue drift for colour descriptors
    rgb[..., 2] *= 1 - 0.06 * warm
    return np.clip(rgb, 0, 255).astype(np.uint8)


os.makedirs(f"{OUT}/meshes", exist_ok=True)

# ---------- terrain albedo
src = cv2.imread(SRC_TEX, cv2.IMREAD_GRAYSCALE)
S = 8192
up = cv2.resize(src, (S, S), interpolation=cv2.INTER_CUBIC)
clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(32, 32)).apply(up).astype(np.float32)
lum = 0.85 * clahe + 0.15 * up + detail((S, S), 1 / 140)
warm = cv2.GaussianBlur(rng.standard_normal((S // 16, S // 16)).astype(np.float32), (0, 0), 6)
warm = cv2.resize(warm / (np.abs(warm).max() + 1e-6), (S, S), interpolation=cv2.INTER_LINEAR)
cv2.imwrite(f"{OUT}/meshes/bishop_scarp_albedo.jpg", cv2.cvtColor(colourise(lum, warm), cv2.COLOR_RGB2BGR),
            [cv2.IMWRITE_JPEG_QUALITY, 92])
print("albedo: grey std %.1f -> %.1f" % (up.std(), np.clip(lum, 0, 255).std()))

# ---------- ground beyond the scan
G = 2048
base = blur(rng.standard_normal((G, G)).astype(np.float32), 40, True)
glum = 150 + 25 * base / (base.std() + 1e-6) + detail((G, G), 1 / 90, wrap=True)
gwarm = blur(rng.standard_normal((G, G)).astype(np.float32), 120, True)
cv2.imwrite(f"{OUT}/meshes/ground_gravel.jpg",
            cv2.cvtColor(colourise(glum, gwarm / (np.abs(gwarm).max() + 1e-6)), cv2.COLOR_RGB2BGR),
            [cv2.IMWRITE_JPEG_QUALITY, 92])
r, t = 300.0, 600.0 / TILE_M
with open(f"{OUT}/meshes/ground.obj", "w") as f:
    f.write(f"v {-r} {-r} 0\nv {r} {-r} 0\nv {r} {r} 0\nv {-r} {r} 0\n")
    f.write(f"vt 0 0\nvt {t} 0\nvt {t} {t}\nvt 0 {t}\nvn 0 0 1\n")
    f.write("f 1/1/1 2/2/1 3/3/1\nf 1/1/1 3/3/1 4/4/1\n")


def material(tex):
    return f"""<material>
          <ambient>0.35 0.35 0.35 1</ambient>
          <diffuse>0.85 0.85 0.85 1</diffuse>
          <specular>0.02 0.02 0.02 1</specular>
          <pbr><metal>
            <albedo_map>meshes/{tex}</albedo_map>
            <roughness>0.95</roughness>
            <metalness>0.0</metalness>
          </metal></pbr>
        </material>"""


with open(f"{OUT}/model.sdf", "w") as f:
    f.write(f"""<?xml version="1.0" ?>
<sdf version="1.8">
  <model name="bishop_scarp">
    <static>true</static>
    <!-- Mesh origin = home (lat {HOME['lat']:.7f}, lon {HOME['lon']:.7f}, {HOME['amsl']:.2f} m NAVD88), ENU.
         The link is raised {-SPAWN_Z} m because terrain_mapping.launch.py spawns the terrain at z = {SPAWN_Z}. -->
    <link name="link">
      <pose>0 0 {-SPAWN_Z} 0 0 0</pose>
      <visual name="visual">
        <geometry><mesh><uri>meshes/bishop_scarp.obj</uri></mesh></geometry>
        {material("bishop_scarp_albedo.jpg")}
      </visual>
      <!-- Textured ground beyond the scan; the world's ground plane under it keeps the collision. -->
      <visual name="ground">
        <pose>0 0 {GROUND_Z} 0 0 0</pose>
        <cast_shadows>false</cast_shadows>
        <geometry><mesh><uri>meshes/ground.obj</uri></mesh></geometry>
        {material("ground_gravel.jpg")}
      </visual>
      <!-- 0.5 m grid of the scan's top surface with a flat 1.5 m pad at home, so the vehicle sits
           level; the visual mesh keeps the real ground there. -->
      <collision name="collision">
        <geometry><mesh><uri>meshes/bishop_scarp_collision.stl</uri></mesh></geometry>
      </collision>
    </link>
  </model>
</sdf>
""")
print("wrote", f"{OUT}/model.sdf")
