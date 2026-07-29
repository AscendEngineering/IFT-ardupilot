#!/usr/bin/env python3
"""Fetch a tile-aligned orthophoto mosaic around a lat/lng and emit
geo-registration data for a Gazebo ground-plane model.

Usage: fetch_ortho.py <lat> <lng> <zoom> <n_tiles_per_side> <out_jpg>
Prints JSON metadata (center lat/lng, plane size in meters) to stdout.
"""
import io
import json
import math
import sys

import requests
from PIL import Image

TILE_URL = ("https://server.arcgisonline.com/ArcGIS/rest/services/"
            "World_Imagery/MapServer/tile/{z}/{y}/{x}")
TILE_PX = 256


def latlng_to_tile(lat, lng, z):
    n = 2 ** z
    x = (lng + 180.0) / 360.0 * n
    lat_r = math.radians(lat)
    y = (1.0 - math.asinh(math.tan(lat_r)) / math.pi) / 2.0 * n
    return x, y


def tile_to_latlng(x, y, z):
    n = 2 ** z
    lng = x / n * 360.0 - 180.0
    lat = math.degrees(math.atan(math.sinh(math.pi * (1 - 2 * y / n))))
    return lat, lng


def main():
    lat, lng = float(sys.argv[1]), float(sys.argv[2])
    zoom, nside = int(sys.argv[3]), int(sys.argv[4])
    out = sys.argv[5]

    cx, cy = latlng_to_tile(lat, lng, zoom)
    x0 = int(cx) - nside // 2
    y0 = int(cy) - nside // 2

    mosaic = Image.new("RGB", (nside * TILE_PX, nside * TILE_PX))
    sess = requests.Session()
    for j in range(nside):
        for i in range(nside):
            r = sess.get(TILE_URL.format(z=zoom, x=x0 + i, y=y0 + j),
                         timeout=30)
            r.raise_for_status()
            mosaic.paste(Image.open(io.BytesIO(r.content)),
                         (i * TILE_PX, j * TILE_PX))
        print(f"row {j+1}/{nside}", file=sys.stderr)

    mosaic.save(out, quality=92)

    # Geographic bounds of the tile block (image top edge = north = y0)
    lat_n, lng_w = tile_to_latlng(x0, y0, zoom)
    lat_s, lng_e = tile_to_latlng(x0 + nside, y0 + nside, zoom)
    lat_c = (lat_n + lat_s) / 2.0
    lng_c = (lng_w + lng_e) / 2.0

    # Ground extents in meters (local flat-earth at center latitude)
    m_per_deg_lat = 111132.92 - 559.82 * math.cos(2 * math.radians(lat_c))
    m_per_deg_lng = 111412.84 * math.cos(math.radians(lat_c))
    size_ns = (lat_n - lat_s) * m_per_deg_lat
    size_ew = (lng_e - lng_w) * m_per_deg_lng

    print(json.dumps({
        "center_lat": round(lat_c, 8),
        "center_lng": round(lng_c, 8),
        "bounds": {"north": lat_n, "south": lat_s,
                   "west": lng_w, "east": lng_e},
        "size_ew_m": round(size_ew, 2),
        "size_ns_m": round(size_ns, 2),
        "px": nside * TILE_PX,
        "gsd_m": round(size_ew / (nside * TILE_PX), 4),
        "zoom": zoom,
    }, indent=2))


if __name__ == "__main__":
    main()
