#!/usr/bin/env python3
"""HTTP inference server for InternVLA-N1 (DualVLN) on a GPU machine.

Based on InternNav scripts/realworld/http_internvla_server.py, with these changes:
  * the instruction comes from the request JSON (the upstream one is hard-coded)
  * camera intrinsics come from the CLI (--intrinsic fx fy cx cy)
  * depth is optional (zeros when the robot sends none)

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
STATE = {"agent": None, "args": None, "intrinsic": None, "instruction": None}


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
    agent.step(np.zeros((h, w, 3), np.uint8), np.zeros((h, w), np.float32), np.eye(4),
               "warm up", intrinsic=STATE["intrinsic"])
    agent.reset()
    return agent


@app.route("/health", methods=["GET"])
def health():
    return jsonify({"ok": STATE["agent"] is not None})


@app.route("/eval_dual", methods=["POST"])
def eval_dual():
    t0 = time.time()
    meta = json.loads(request.form.get("json", "{}"))
    rgb = np.asarray(Image.open(io.BytesIO(request.files["image"].read())).convert("RGB"))
    if "depth" in request.files:
        depth = np.asarray(Image.open(io.BytesIO(request.files["depth"].read())), dtype=np.float32) / 10000.0
    else:
        depth = np.zeros(rgb.shape[:2], np.float32)

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
    p.add_argument("--host", default="0.0.0.0")
    p.add_argument("--port", type=int, default=5801)
    args = p.parse_args()

    STATE["intrinsic"] = build_intrinsic(*args.intrinsic)
    STATE["instruction"] = args.instruction
    STATE["args"] = args
    STATE["agent"] = load_agent(args)
    app.run(host=args.host, port=args.port, threaded=False)


if __name__ == "__main__":
    main()
