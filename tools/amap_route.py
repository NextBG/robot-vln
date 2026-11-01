#!/usr/bin/env python3
"""Plan a closed walking loop around a lake (default: 滴水湖, Shanghai) with the AMap (高德) Web API
and export GPS waypoints for gps_waypoint_follower.

AMap returns GCJ-02 coordinates; GNSS receivers (WHEELTEC G60) output WGS-84. The output file is WGS-84.
Only the Python standard library is used.

  export AMAP_KEY=<your web-service key>      # never commit the key
  python3 tools/amap_route.py --out ros2_ws/src/vln_robot_nav/routes/dishui_lake.csv \
      [--start 121.933527,30.909469] [--center 121.941304,30.897209] [--radius 1350] [--n 16] \
      [--spacing 5] [--staticmap loop.png]

With --start (default: 上海临港凯悦嘉轩酒店, 水芸北路99号) the route is
start -> nearest point of the lake ring -> full loop -> same ring point -> start.

Outputs:
  <out>.csv      lat,lon (WGS-84), one waypoint per --spacing metres, closed loop
  <out>.geojson  same route as a LineString (open in geojson.io to inspect)
"""
import argparse
import json
import math
import os
import sys
import time
import urllib.parse
import urllib.request

API = "https://restapi.amap.com"
EARTH_R = 6378137.0

# ---------------------------------------------------------------- GCJ-02 <-> WGS-84
_A = 6378245.0
_EE = 0.00669342162296594323


def _out_of_china(lon, lat):
    return not (72.004 <= lon <= 137.8347 and 0.8293 <= lat <= 55.8271)


def _tlat(x, y):
    r = -100.0 + 2.0 * x + 3.0 * y + 0.2 * y * y + 0.1 * x * y + 0.2 * math.sqrt(abs(x))
    r += (20.0 * math.sin(6.0 * x * math.pi) + 20.0 * math.sin(2.0 * x * math.pi)) * 2.0 / 3.0
    r += (20.0 * math.sin(y * math.pi) + 40.0 * math.sin(y / 3.0 * math.pi)) * 2.0 / 3.0
    r += (160.0 * math.sin(y / 12.0 * math.pi) + 320 * math.sin(y * math.pi / 30.0)) * 2.0 / 3.0
    return r


def _tlon(x, y):
    r = 300.0 + x + 2.0 * y + 0.1 * x * x + 0.1 * x * y + 0.1 * math.sqrt(abs(x))
    r += (20.0 * math.sin(6.0 * x * math.pi) + 20.0 * math.sin(2.0 * x * math.pi)) * 2.0 / 3.0
    r += (20.0 * math.sin(x * math.pi) + 40.0 * math.sin(x / 3.0 * math.pi)) * 2.0 / 3.0
    r += (150.0 * math.sin(x / 12.0 * math.pi) + 300.0 * math.sin(x / 30.0 * math.pi)) * 2.0 / 3.0
    return r


def wgs84_to_gcj02(lon, lat):
    if _out_of_china(lon, lat):
        return lon, lat
    dlat = _tlat(lon - 105.0, lat - 35.0)
    dlon = _tlon(lon - 105.0, lat - 35.0)
    radlat = lat / 180.0 * math.pi
    magic = 1 - _EE * math.sin(radlat) ** 2
    sq = math.sqrt(magic)
    dlat = (dlat * 180.0) / ((_A * (1 - _EE)) / (magic * sq) * math.pi)
    dlon = (dlon * 180.0) / (_A / sq * math.cos(radlat) * math.pi)
    return lon + dlon, lat + dlat


def gcj02_to_wgs84(lon, lat, iters=3):
    """Iterative inverse (error < 1 cm)."""
    wlon, wlat = lon, lat
    for _ in range(iters):
        glon, glat = wgs84_to_gcj02(wlon, wlat)
        wlon -= glon - lon
        wlat -= glat - lat
    return wlon, wlat


# ---------------------------------------------------------------- geometry helpers
def haversine(lat1, lon1, lat2, lon2):
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dp, dl = p2 - p1, math.radians(lon2 - lon1)
    a = math.sin(dp / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2) ** 2
    return 2 * EARTH_R * math.asin(math.sqrt(a))


def offset(lat, lon, north_m, east_m):
    return (lat + math.degrees(north_m / EARTH_R),
            lon + math.degrees(east_m / (EARTH_R * math.cos(math.radians(lat)))))


def resample(pts, spacing):
    """pts: [(lat, lon)] -> evenly spaced points (spacing metres)."""
    out = [pts[0]]
    carry = 0.0
    for (a_lat, a_lon), (b_lat, b_lon) in zip(pts, pts[1:]):
        seg = haversine(a_lat, a_lon, b_lat, b_lon)
        if seg < 1e-6:
            continue
        d = spacing - carry
        while d <= seg:
            t = d / seg
            out.append((a_lat + t * (b_lat - a_lat), a_lon + t * (b_lon - a_lon)))
            d += spacing
        carry = seg - (d - spacing)
    if haversine(*out[-1], *pts[-1]) > spacing * 0.3:
        out.append(pts[-1])
    return out


def path_length(pts):
    return sum(haversine(*a, *b) for a, b in zip(pts, pts[1:]))


# ---------------------------------------------------------------- AMap API
def amap(path, **params):
    params["key"] = os.environ["AMAP_KEY"]
    url = f"{API}{path}?{urllib.parse.urlencode(params)}"
    for attempt in range(3):
        try:
            with urllib.request.urlopen(url, timeout=15) as r:
                data = json.loads(r.read().decode())
            if str(data.get("status")) == "1":
                return data
            raise RuntimeError(f"AMap error: {data.get('info')} ({data.get('infocode')})")
        except Exception as e:  # noqa: BLE001
            if attempt == 2:
                raise
            print(f"retry {path}: {e}", file=sys.stderr)
            time.sleep(1.0)
    return None


def walking(o_lon, o_lat, d_lon, d_lat):
    """GCJ-02 in/out. Returns [(lon, lat)] polyline."""
    data = amap("/v3/direction/walking", origin=f"{o_lon:.6f},{o_lat:.6f}",
                destination=f"{d_lon:.6f},{d_lat:.6f}")
    pts = []
    for step in data["route"]["paths"][0]["steps"]:
        for p in step["polyline"].split(";"):
            lon, lat = map(float, p.split(","))
            if not pts or pts[-1] != (lon, lat):
                pts.append((lon, lat))
    return pts


def remove_spurs(pts, tol_m=4.0, window=80):
    """Drop out-and-back detours: when the path returns within tol_m of a recent vertex, cut the loop."""
    out = []
    for p in pts:
        cut = None
        for j in range(len(out) - 2, max(-1, len(out) - window), -1):
            if haversine(out[j][1], out[j][0], p[1], p[0]) < tol_m:
                cut = j
                break
        if cut is not None:
            del out[cut + 1:]
        else:
            out.append(p)
    return out


def staticmap(gcj_pts, out_png, center):
    # AMap static map: max ~ 100 path points
    step = max(1, len(gcj_pts) // 90)
    sub = gcj_pts[::step] + [gcj_pts[0]]
    path = ";".join(f"{lon:.6f},{lat:.6f}" for lon, lat in sub)
    params = {"location": f"{center[0]:.6f},{center[1]:.6f}", "zoom": 14, "size": "1024*1024",
              "scale": 2, "paths": f"6,0xFF0000,1,,:{path}",
              "markers": f"mid,0x00AA00,S:{sub[0][0]:.6f},{sub[0][1]:.6f}",
              "key": os.environ["AMAP_KEY"]}
    url = f"{API}/v3/staticmap?{urllib.parse.urlencode(params)}"
    with urllib.request.urlopen(url, timeout=20) as r, open(out_png, "wb") as f:
        f.write(r.read())


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--center", default="121.941304,30.897209", help="lake centre lon,lat (GCJ-02, AMap)")
    ap.add_argument("--radius", type=float, default=1350.0, help="ring radius in metres for seed points")
    ap.add_argument("--n", type=int, default=16, help="number of seed points on the ring")
    ap.add_argument("--start", default="121.933527,30.909469",
                    help="start/finish lon,lat (GCJ-02). Default: 上海临港凯悦嘉轩酒店. Use \"none\" for a pure ring")
    ap.add_argument("--start_bearing", type=float, default=None,
                    help="deg from north for the first seed point (default: towards --start, else 0)")
    ap.add_argument("--clockwise", action="store_true")
    ap.add_argument("--spacing", type=float, default=5.0, help="output waypoint spacing (m)")
    ap.add_argument("--out", required=True, help="output path without or with .csv")
    ap.add_argument("--staticmap", help="also save an AMap static map PNG of the loop")
    a = ap.parse_args()
    if "AMAP_KEY" not in os.environ:
        sys.exit("set AMAP_KEY")

    c_lon, c_lat = map(float, a.center.split(","))
    start = None if a.start.lower() == "none" else tuple(map(float, a.start.split(",")))
    if a.start_bearing is None:
        a.start_bearing = 0.0
        if start:
            dn = math.radians(start[1] - c_lat) * EARTH_R
            de = math.radians(start[0] - c_lon) * EARTH_R * math.cos(math.radians(c_lat))
            a.start_bearing = math.degrees(math.atan2(de, dn))
    seeds = []
    for i in range(a.n):
        b = math.radians(a.start_bearing + (1 if a.clockwise else -1) * 360.0 * i / a.n)
        lat, lon = offset(c_lat, c_lon, a.radius * math.cos(b), a.radius * math.sin(b))
        seeds.append((lon, lat))
    seeds.append(seeds[0])

    gcj = []
    for (o_lon, o_lat), (d_lon, d_lat) in zip(seeds, seeds[1:]):
        seg = walking(o_lon, o_lat, d_lon, d_lat)
        gcj.extend(seg if not gcj else seg[1:])
        time.sleep(0.35)  # personal keys are QPS-limited
    if gcj[0] != gcj[-1]:
        gcj.append(gcj[0])
    n_raw = len(gcj)
    gcj = remove_spurs(gcj[:-1]) + [gcj[0]]
    print(f"spur removal: {n_raw} -> {len(gcj)} vertices")
    if start:
        ring0 = gcj[0]
        access = walking(start[0], start[1], ring0[0], ring0[1])
        time.sleep(0.35)
        back = walking(ring0[0], ring0[1], start[0], start[1])
        print(f"access leg start -> ring: {path_length([(la, lo) for lo, la in access]):.0f} m")
        gcj = [start] + access + gcj[1:] + back + [start]
        gcj = [p for i, p in enumerate(gcj) if i == 0 or p != gcj[i - 1]]

    wgs = [gcj02_to_wgs84(lon, lat) for lon, lat in gcj]
    pts = resample([(lat, lon) for lon, lat in wgs], a.spacing)
    print(f"route: {len(gcj)} vertices, {path_length([(la, lo) for lo, la in wgs]):.0f} m; "
          f"resampled: {len(pts)} pts")

    base = a.out[:-4] if a.out.endswith(".csv") else a.out
    os.makedirs(os.path.dirname(base) or ".", exist_ok=True)
    with open(base + ".csv", "w") as f:
        f.write("# WGS-84 waypoints generated by tools/amap_route.py\nlat,lon\n")
        for lat, lon in pts:
            f.write(f"{lat:.8f},{lon:.8f}\n")
    with open(base + ".geojson", "w") as f:
        json.dump({"type": "Feature", "properties": {"crs": "WGS-84"},
                   "geometry": {"type": "LineString", "coordinates": [[lon, lat] for lat, lon in pts]}}, f)
    if a.staticmap:
        staticmap(gcj, a.staticmap, (c_lon, c_lat))  # S marker = start/finish
    print("wrote", base + ".csv", base + ".geojson", a.staticmap or "")


if __name__ == "__main__":
    main()
