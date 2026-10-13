"""Point a PX4 gz world at the Bishop scarp: python3 set_world.py <world.sdf> <home.json>

GPS origin (spherical_coordinates) = the model's home, so PX4 reports real coordinates and
altitude; the ground plane drops to z = -2.0 (just under the scan's lowest point) and loses its
visual, which the model's gravel layer replaces. Its collision stays as the floor beyond the scan.
"""
import json
import re
import sys

world, home = sys.argv[1], json.load(open(sys.argv[2]))
s = open(world, encoding="utf-8-sig").read()


def sub(pattern, repl, text, flags=0):
    out, n = re.subn(pattern, repl, text, count=1, flags=flags)
    if n != 1:
        sys.exit(f"set_world: no match for {pattern!r} in {world}")
    return out


s = sub(r"<latitude_deg>.*?</latitude_deg>", f"<latitude_deg>{home['lat']:.7f}</latitude_deg>", s)
s = sub(r"<longitude_deg>.*?</longitude_deg>", f"<longitude_deg>{home['lon']:.7f}</longitude_deg>", s)
s = sub(r"<elevation>.*?</elevation>", f"<elevation>{home['amsl']:.2f}</elevation>", s)
s = sub(r'(<model name="ground_plane">.*?</link>\s*)<pose>[^<]*</pose>', r"\1<pose>0 0 -2.0 0 0 0</pose>", s, re.S)
s = sub(r'(<model name="ground_plane">.*?<visual name="visual">.*?)<size>[^<]*</size>', r"\1<size>0.01 0.01</size>", s, re.S)
open(world, "w").write(s)
print(f"set_world: {world} -> lat {home['lat']:.7f} lon {home['lon']:.7f} elevation {home['amsl']:.2f} m")
