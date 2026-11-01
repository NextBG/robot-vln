#!/usr/bin/env python3
"""HTTP inference server for InternVLA-N1 (DualVLN) on a GPU machine.

Based on InternNav scripts/realworld/http_internvla_server.py, with these changes:
  * the instruction comes from the request JSON (the upstream one is hard-coded)
  * camera intrinsics come from the CLI (--intrinsic fx fy cx cy)
  * depth is optional; by default (--depth_fill mono) the sparse LiDAR-projected depth is densified with
    Depth-Anything-V2 metric depth, scale-aligned to the LiDAR pixels (pure monocular if none is sent)

Run inside the InternNav repo/conda env:
  python server/internvla_n1_server.py --model_path checkpoints/InternVLA-N1-DualVLN \
      --intrinsic 457 457 320 240 --port 5801

Request (multipart/form-data, POST /eval_dual):
  image: JPEG (RGB)
  depth: optional 16-bit PNG, value = metres * 10000
  json : {"reset": bool, "idx": int, "instruction": str}
Response JSON: {"discrete_action": [...]} or {"trajectory": [[x, y], ...], "pixel_goal": [u, v]}
"""
import argparse
import io
import json
import sys
import time
from pathlib import Path

import numpy as np
from flask import Flask, jsonify, request
from PIL import Image

app = Flask(__name__)
STATE = {"agent": None, "args": None, "intrinsic": None, "instruction": None, "mono": None}


def load_mono_depth(args):
    """Depth-Anything-V2 metric (Hypersim, ViT-S): the same checkpoint InternVLA-N1 already downloads."""
    import torch
    from internnav.model.encoder.depth_anything.depth_anything_v2.dpt import DepthAnythingV2

    m = DepthAnythingV2(encoder="vits", features=64, out_channels=[48, 96, 192, 384], max_depth=20.0)
    m.load_state_dict(torch.load(args.mono_depth_ckpt, map_location="cpu"))
    return m.to(args.device).eval()


def fill_depth(rgb, depth, max_range=20.0, min_points=50):
    """Densify sparse LiDAR depth: scale-align monocular metric depth to the LiDAR pixels, fill the holes.

    Returns (dense depth in metres, info dict). LiDAR pixels are kept as measured.
    """
    mono = STATE["mono"].infer_image(np.ascontiguousarray(rgb[:, :, ::-1]), 518).astype(np.float32)
    valid = (depth > 0.1) & (depth < max_range) & (mono > 0.05)
    n = int(valid.sum())
    scale = float(np.median(depth[valid] / mono[valid])) if n >= min_points else 1.0
    scale = float(np.clip(scale, 0.3, 3.0))
    dense = np.where(valid, depth, mono * scale).astype(np.float32)
    return dense, {"lidar_px": n, "lidar_frac": round(n / depth.size, 4), "mono_scale": round(scale, 3)}


def build_intrinsic(fx, fy, cx, cy):
    k = np.eye(4, dtype=np.float32)
    k[0, 0], k[1, 1], k[0, 2], k[1, 2] = fx, fy, cx, cy
    return k


def load_agent(args):
    sys.path.insert(0, str(Path(args.internnav_root).resolve()))
    from internnav.agent.internvla_n1_agent_realworld import InternVLAN1AsyncAgent  # noqa: E402

    agent = InternVLAN1AsyncAgent(args)
    agent.reset()  # upstream __init__ sets save_dir but only reset() creates it
    h, w = args.height, args.width
    import torch

    with torch.no_grad():
        agent.step(np.zeros((h, w, 3), np.uint8), np.zeros((h, w), np.float32), np.eye(4),
                   "warm up", intrinsic=STATE["intrinsic"])
    agent.reset()
    return agent


@app.route("/health", methods=["GET"])
def health():
    return jsonify({"ok": STATE["agent"] is not None})


@app.route("/eval_dual", methods=["POST"])
def eval_dual():
    import torch

    # upstream step_s1 (trajectory diffusion) runs without no_grad and OOMs a 24 GB GPU after ~3 steps
    with torch.no_grad():
        return _eval_dual()


def _eval_dual():
    t0 = time.time()
    meta = json.loads(request.form.get("json", "{}"))
    rgb = np.asarray(Image.open(io.BytesIO(request.files["image"].read())).convert("RGB"))
    if "depth" in request.files:
        depth = np.asarray(Image.open(io.BytesIO(request.files["depth"].read())), dtype=np.float32) / 10000.0
    else:
        depth = np.zeros(rgb.shape[:2], np.float32)
    depth_info = {}
    if STATE["mono"] is not None:
        t1 = time.time()
        depth, depth_info = fill_depth(rgb, depth)
        depth_info["fill_s"] = round(time.time() - t1, 3)

    agent = STATE["agent"]
    instruction = meta.get("instruction") or STATE["instruction"]
    if meta.get("reset") or instruction != STATE["instruction"]:
        agent.reset()
        STATE["instruction"] = instruction

    pose = np.eye(4)  # the realworld agent works in the body frame
    out = agent.step(rgb, depth, pose, instruction, intrinsic=STATE["intrinsic"])
    dual_sys_output, _ = out if isinstance(out, tuple) else (out, None)

    # action 5 = look down: upstream re-steps with look_down=True
    if getattr(dual_sys_output, "output_action", None) is not None and \
            list(dual_sys_output.output_action) == [5]:
        out = agent.step(rgb, depth, pose, instruction, intrinsic=STATE["intrinsic"], look_down=True)
        dual_sys_output, _ = out if isinstance(out, tuple) else (out, None)

    resp = {}
    if getattr(dual_sys_output, "output_action", None) is not None:
        resp["discrete_action"] = [int(a) for a in dual_sys_output.output_action]
    else:
        traj = getattr(dual_sys_output, "output_trajectory", None)
        if traj is not None:
            resp["trajectory"] = np.asarray(traj).tolist()
        pg = getattr(dual_sys_output, "output_pixel", None)
        if pg is not None:
            resp["pixel_goal"] = np.asarray(pg).tolist()
    if depth_info:
        resp["depth"] = depth_info
    resp["latency_s"] = round(time.time() - t0, 3)
    return jsonify(resp)


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--internnav_root", default=".")
    p.add_argument("--model_path", default="checkpoints/InternVLA-N1-DualVLN")
    p.add_argument("--device", default="cuda:0")
    p.add_argument("--resize_w", type=int, default=384)
    p.add_argument("--resize_h", type=int, default=384)
    p.add_argument("--num_history", type=int, default=8)
    p.add_argument("--plan_step_gap", type=int, default=4)
    p.add_argument("--width", type=int, default=640)
    p.add_argument("--height", type=int, default=480)
    p.add_argument("--intrinsic", type=float, nargs=4, metavar=("FX", "FY", "CX", "CY"),
                   default=[457.0, 457.0, 320.0, 240.0])
    p.add_argument("--instruction", default="Walk forward and stop.")
    p.add_argument("--depth_fill", choices=["mono", "none"], default="mono",
                   help="mono: fill holes in the sparse LiDAR depth with scale-aligned Depth-Anything-V2 metric "
                        "depth (pure monocular if no depth is sent); none: pass depth through (zeros = no data)")
    p.add_argument("--mono_depth_ckpt", default="checkpoints/depth_anything_v2_metric_hypersim_vits.pth")
    p.add_argument("--host", default="0.0.0.0")
    p.add_argument("--port", type=int, default=5801)
    args = p.parse_args()

    STATE["intrinsic"] = build_intrinsic(*args.intrinsic)
    STATE["instruction"] = args.instruction
    STATE["args"] = args
    STATE["agent"] = load_agent(args)
    if args.depth_fill == "mono":
        STATE["mono"] = load_mono_depth(args)
        fill_depth(np.zeros((args.height, args.width, 3), np.uint8), np.zeros((args.height, args.width), np.float32))
    app.run(host=args.host, port=args.port, threaded=False)


if __name__ == "__main__":
    main()
