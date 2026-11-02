"""AMap (高德) walking directions in WGS-84, standard library only.

The web-service key is read from $AMAP_KEY or ~/.config/vln_robot_web/amap_key (never commit it).
"""
import json
import math
import os
import urllib.parse
import urllib.request

KEY_FILE = '~/.config/vln_robot_web/amap_key'
_A, _EE = 6378245.0, 0.00669342162296594323


def load_key():
    key = os.environ.get('AMAP_KEY', '').strip()
    path = os.path.expanduser(KEY_FILE)
    if not key and os.path.exists(path):
        with open(path) as f:
            key = f.read().strip()
    return key or None


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


def wgs2gcj(lat, lon):
    if not (72.004 <= lon <= 137.8347 and 0.8293 <= lat <= 55.8271):
        return lat, lon
    dlat, dlon = _tlat(lon - 105.0, lat - 35.0), _tlon(lon - 105.0, lat - 35.0)
    rad = math.radians(lat)
    magic = 1 - _EE * math.sin(rad) ** 2
    sq = math.sqrt(magic)
    dlat = dlat * 180.0 / ((_A * (1 - _EE)) / (magic * sq) * math.pi)
    dlon = dlon * 180.0 / (_A / sq * math.cos(rad) * math.pi)
    return lat + dlat, lon + dlon


def gcj2wgs(lat, lon):
    wlat, wlon = lat, lon
    for _ in range(3):
        glat, glon = wgs2gcj(wlat, wlon)
        wlat, wlon = wlat - (glat - lat), wlon - (glon - lon)
    return wlat, wlon


def walking(start, end, key, timeout=15):
    """start/end (lat, lon) WGS-84 -> [(lat, lon)] WGS-84 walking polyline."""
    o, d = wgs2gcj(*start), wgs2gcj(*end)
    q = urllib.parse.urlencode({'key': key, 'origin': f'{o[1]:.6f},{o[0]:.6f}',
                                'destination': f'{d[1]:.6f},{d[0]:.6f}'})
    with urllib.request.urlopen(f'https://restapi.amap.com/v3/direction/walking?{q}', timeout=timeout) as r:
        data = json.loads(r.read().decode())
    if str(data.get('status')) != '1':
        raise RuntimeError(f"高德接口错误：{data.get('info')} ({data.get('infocode')})")
    pts = [start]
    for step in data['route']['paths'][0]['steps']:
        for p in step['polyline'].split(';'):
            lon, lat = map(float, p.split(','))
            w = gcj2wgs(lat, lon)
            if w != pts[-1]:
                pts.append(w)
    pts.append(end)
    return pts
