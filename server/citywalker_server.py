"""CityWalker (CVPR 2025, ai4ce/CityWalker, Apache-2.0) inference server.

Learned local planner: 5 camera frames at 1 Hz + 5 past positions + a GPS target -> 5 future
waypoints (~1 s apart) and an "arrived" probability.

    POST /citywalker  multipart:
        image0..image4  JPEG, oldest first (image4 = current frame), any size (resized to 640 wide)
        json            {"hist": [[x, y] x5]   past positions, oldest first, last = [0, 0],
                         "target": [x, y],     goal position,
                         "step_scale": 0.5}    metres per 1 s step (normalisation used in training)
                        all positions in the robot body frame at the current frame, metres,
                        x forward, y left
    -> {"waypoints": [[x, y] x5] (body frame, metres), "arrive_prob": p, "infer_ms": t}
    GET  /health

Model frame (pretraining data, camera poses from visual SLAM): x right, z forward, positions
divided by step_scale.  Lightning is not needed: the checkpoint's state_dict is loaded directly,
the (unused) efficientnet_pytorch import is stubbed and DINOv2 is built from a local hub checkout
(its weights are part of the checkpoint).

    python server/citywalker_server.py --citywalker_root /root/CityWalker \
        --ckpt /root/CityWalker/checkpoints/CityWalker_2000hr.ckpt --dinov2_root /root/dinov2 --port 5802
"""
import argparse
import json
import os
import sys
import threading
import time
import types

import cv2
import numpy as np
import torch
import yaml
from flask import Flask, jsonify, request

CONTEXT = 5
H, W = 360, 640   # CityWalkFeatDataset.process_frames


class NS(types.SimpleNamespace):
    @classmethod
    def build(cls, d):
        return cls(**{k: cls.build(v) if isinstance(v, dict) else v for k, v in d.items()})


def body_to_model(xy):
    """body (x fwd, y left) -> model (x right, z fwd)"""
    xy = np.asarray(xy, np.float32).reshape(-1, 2)
    return np.stack([-xy[:, 1], xy[:, 0]], 1)


def model_to_body(xz):
    xz = np.asarray(xz, np.float32).reshape(-1, 2)
    return np.stack([xz[:, 1], -xz[:, 0]], 1)


def prep_frame(bgr):
    """BGR uint8 -> float RGB 0..1 (3, 360, 640): scale to 640 wide, then centre crop / pad height."""
    h, w = bgr.shape[:2]
    if w != W:
        bgr = cv2.resize(bgr, (W, max(1, round(h * W / w))), interpolation=cv2.INTER_AREA)
    h = bgr.shape[0]
    if h > H:
        y0 = (h - H) // 2
        bgr = bgr[y0:y0 + H]
    elif h < H:
        top = (H - h) // 2
        bgr = cv2.copyMakeBorder(bgr, top, H - h - top, 0, 0, cv2.BORDER_CONSTANT, value=0)
    rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
    return torch.from_numpy(rgb).permute(2, 0, 1).float() / 255.0


def load_model(args, device):
    sys.path.insert(0, args.citywalker_root)
    try:
        import efficientnet_pytorch  # noqa: F401
    except ImportError:
        stub = types.ModuleType('efficientnet_pytorch')
        stub.EfficientNet = None
        sys.modules['efficientnet_pytorch'] = stub
    hub_load = torch.hub.load

    def local_hub(repo, name, *a, **kw):
        if repo == 'facebookresearch/dinov2' and args.dinov2_root:
            return hub_load(args.dinov2_root, name, source='local', pretrained=False)
        return hub_load(repo, name, *a, **kw)

    torch.hub.load = local_hub
    from model.citywalker_feat import CityWalkerFeat

    with open(args.config or os.path.join(args.citywalker_root, 'config', 'citywalk_2000hr.yaml')) as f:
        cfg = NS.build(yaml.safe_load(f))
    model = CityWalkerFeat(cfg)
    ck = torch.load(args.ckpt, map_location='cpu', weights_only=False)
    sd = ck.get('state_dict', ck)
    sd = {k[len('model.'):]: v for k, v in sd.items() if k.startswith('model.')} or sd
    missing, unexpected = model.load_state_dict(sd, strict=False)
    if missing:
        raise RuntimeError(f'missing weights: {missing[:8]} ...')
    if unexpected:
        print(f'[citywalker] ignoring {len(unexpected)} unexpected keys, e.g. {unexpected[:3]}')
    return model.eval().to(device), cfg


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--citywalker_root', default='/root/CityWalker')
    ap.add_argument('--ckpt', default='/root/CityWalker/checkpoints/CityWalker_2000hr.ckpt')
    ap.add_argument('--config', default='')
    ap.add_argument('--dinov2_root', default='/root/dinov2')
    ap.add_argument('--host', default='127.0.0.1')
    ap.add_argument('--port', type=int, default=5802)
    ap.add_argument('--device', default='cuda')
    args = ap.parse_args()

    device = torch.device(args.device)
    t0 = time.time()
    model, cfg = load_model(args, device)
    info = {'model': 'CityWalker_2000hr', 'context': CONTEXT, 'device': str(device),
            'load_s': round(time.time() - t0, 1), 'requests': 0}
    print(f'[citywalker] loaded in {info["load_s"]} s on {device}')
    lock = threading.Lock()
    app = Flask(__name__)

    @app.get('/health')
    def health():
        return jsonify(ok=True, **info)

    @app.post('/citywalker')
    def citywalker():
        try:
            meta = json.loads(request.form.get('json') or '{}')
            frames = []
            for i in range(CONTEXT):
                f = request.files.get(f'image{i}')
                if f is None:
                    return jsonify(error=f'missing image{i}'), 400
                img = cv2.imdecode(np.frombuffer(f.read(), np.uint8), cv2.IMREAD_COLOR)
                if img is None:
                    return jsonify(error=f'bad image{i}'), 400
                frames.append(prep_frame(img))
            hist = np.asarray(meta['hist'], np.float32).reshape(CONTEXT, 2)
            target = np.asarray(meta['target'], np.float32).reshape(1, 2)
            scale = max(1e-2, float(meta.get('step_scale', 0.5)))
        except (KeyError, ValueError, TypeError) as e:
            return jsonify(error=f'bad request: {e}'), 400
        cord = np.concatenate([body_to_model(hist), body_to_model(target)], 0) / scale
        obs = torch.stack(frames)[None].to(device)
        cord = torch.from_numpy(cord)[None].to(device)
        t = time.time()
        with lock, torch.inference_mode():
            wp, arrive, _, _ = model(obs, cord, None)
            if device.type == 'cuda':
                torch.cuda.synchronize()
        info['requests'] += 1
        wp = model_to_body(wp[0].float().cpu().numpy() * scale)
        return jsonify(waypoints=np.round(wp, 3).tolist(),
                       arrive_prob=round(float(torch.sigmoid(arrive[0, 0])), 3),
                       infer_ms=round((time.time() - t) * 1000, 1))

    app.run(host=args.host, port=args.port, threaded=True)


if __name__ == '__main__':
    main()
