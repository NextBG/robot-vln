#!/usr/bin/env python3
"""Send one RGB(+depth) frame to the VLN server and print the reply (no robot / ROS needed).

  python server/test_client.py --url http://127.0.0.1:5801/eval_dual --image some.jpg \
      --instruction "Walk to the door and stop."
Without --image a random image is sent (only checks the pipeline, not the output quality).
"""
import argparse
import io
import json
import time

import numpy as np
import requests
from PIL import Image


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--url", default="http://127.0.0.1:5801/eval_dual")
    ap.add_argument("--image")
    ap.add_argument("--instruction", default="Walk forward and stop at the door.")
    ap.add_argument("--steps", type=int, default=3)
    a = ap.parse_args()

    img = Image.open(a.image).convert("RGB").resize((640, 480)) if a.image else \
        Image.fromarray(np.random.randint(0, 255, (480, 640, 3), np.uint8))
    jpg = io.BytesIO(); img.save(jpg, format="JPEG", quality=90)
    depth = Image.fromarray(np.full((480, 640), 20000, np.uint16))  # 2 m
    png = io.BytesIO(); depth.save(png, format="PNG")

    for i in range(a.steps):
        files = {"image": ("rgb.jpg", jpg.getvalue(), "image/jpeg"),
                 "depth": ("depth.png", png.getvalue(), "image/png")}
        data = {"json": json.dumps({"reset": i == 0, "idx": i, "instruction": a.instruction})}
        t = time.time()
        r = requests.post(a.url, files=files, data=data, timeout=60)
        print(f"step {i}: {time.time() - t:.2f}s HTTP {r.status_code} {r.text[:300]}")


if __name__ == "__main__":
    main()
