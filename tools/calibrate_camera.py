#!/usr/bin/env python3
"""Interactive chessboard calibration for the USB webcam (no ROS needed).

Run on the robot with plain python3 (uses ~/.local opencv), with usb_cam NOT running:

  python3 tools/calibrate_camera.py --cols 9 --rows 6 --square 0.025

--cols/--rows are INNER corners (a board with 10x7 squares has 9x6 inner corners),
--square is the square edge in metres.

Frames are captured automatically when the board is held still in a new pose. The
console prints which pose is still missing. A live view is shown on the robot screen
(if DISPLAY is available) and at http://<robot>:8090/ (MJPEG). When coverage is good
enough it calibrates, drops outlier views, and writes a ROS camera_info YAML. Ctrl-C
calibrates early with whatever has been collected.
"""
import argparse
import os
import signal
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2
import numpy as np

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DEFAULT_OUT = os.path.join(REPO, 'ros2_ws/src/vln_robot_bringup/config/camera_info.yaml')

# coverage targets: board centre in a 3x3 grid, near/far, tilted left/right/up/down
POS_NAMES = [['左上', '上中', '右上'], ['左中', '正中', '右中'], ['左下', '下中', '右下']]


class Preview:
    """Latest annotated frame, served as MJPEG."""

    def __init__(self):
        self.jpg = None
        self.lock = threading.Lock()

    def set(self, img):
        ok, buf = cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, 70])
        if ok:
            with self.lock:
                self.jpg = buf.tobytes()

    def get(self):
        with self.lock:
            return self.jpg


def serve(preview, port):
    class H(BaseHTTPRequestHandler):
        def log_message(self, *a):
            pass

        def do_GET(self):
            if self.path.startswith('/snap'):
                jpg = preview.get() or b''
                self.send_response(200)
                self.send_header('Content-Type', 'image/jpeg')
                self.end_headers()
                self.wfile.write(jpg)
                return
            self.send_response(200)
            self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=f')
            self.end_headers()
            try:
                while True:
                    jpg = preview.get()
                    if jpg:
                        self.wfile.write(b'--f\r\nContent-Type: image/jpeg\r\n\r\n' + jpg + b'\r\n')
                    time.sleep(0.1)
            except (BrokenPipeError, ConnectionResetError):
                pass

    srv = ThreadingHTTPServer(('0.0.0.0', port), H)
    threading.Thread(target=srv.serve_forever, daemon=True).start()


def board_params(corners, cols, rows, w, h):
    """(cx, cy, size, tilt_x, tilt_y) of a detected board, all roughly in [0,1] / [-1,1]."""
    c = corners.reshape(rows, cols, 2)
    tl, tr, br, bl = c[0, 0], c[0, -1], c[-1, -1], c[-1, 0]
    quad = np.array([tl, tr, br, bl], np.float32)
    area = cv2.contourArea(quad)
    cx, cy = c.reshape(-1, 2).mean(0)
    size = np.sqrt(area / (w * h))
    # perspective: ratio of opposite edges
    top, bot = np.linalg.norm(tr - tl), np.linalg.norm(br - bl)
    left, right = np.linalg.norm(bl - tl), np.linalg.norm(br - tr)
    tilt_y = (top - bot) / max(top, bot)      # board leaning forward/back
    tilt_x = (left - right) / max(left, right)  # board turned left/right
    return np.array([cx / w, cy / h, size, tilt_x, tilt_y])


class Coverage:
    def __init__(self):
        self.pos = np.zeros((3, 3), int)
        self.near = self.far = 0
        self.tilt = {'左转': 0, '右转': 0, '上仰': 0, '下俯': 0}

    def add(self, p):
        self.pos[min(int(p[1] * 3), 2), min(int(p[0] * 3), 2)] += 1
        if p[2] > 0.45:
            self.near += 1
        if p[2] < 0.28:
            self.far += 1
        if p[3] > 0.12:
            self.tilt['左转'] += 1
        if p[3] < -0.12:
            self.tilt['右转'] += 1
        if p[4] > 0.12:
            self.tilt['上仰'] += 1
        if p[4] < -0.12:
            self.tilt['下俯'] += 1

    def missing(self):
        m = [f'把棋盘移到画面{POS_NAMES[r][c]}' for r in range(3) for c in range(3) if self.pos[r, c] == 0]
        if self.near < 3:
            m.append('棋盘靠近相机（占满大半个画面）')
        if self.far < 3:
            m.append('棋盘远离相机（约占画面 1/4）')
        m += [f'棋盘{k}约 30°' for k, v in self.tilt.items() if v < 2]
        return m


def write_yaml(path, w, h, K, D, P, name, rms, n):
    f = lambda a: ', '.join(f'{x:.6f}' for x in np.asarray(a).ravel())  # noqa: E731
    txt = f"""# Calibrated {time.strftime('%Y-%m-%d %H:%M')} with tools/calibrate_camera.py
# {n} views, reprojection RMS {rms:.3f} px. VLN server: --intrinsic {K[0,0]:.1f} {K[1,1]:.1f} {K[0,2]:.1f} {K[1,2]:.1f}
image_width: {w}
image_height: {h}
camera_name: {name}
camera_matrix:
  rows: 3
  cols: 3
  data: [{f(K)}]
distortion_model: plumb_bob
distortion_coefficients:
  rows: 1
  cols: 5
  data: [{f(D[:5])}]
rectification_matrix:
  rows: 3
  cols: 3
  data: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
projection_matrix:
  rows: 3
  cols: 4
  data: [{f(P)}]
"""
    with open(path, 'w') as fh:
        fh.write(txt)


def calibrate(objp, img_pts, size):
    obj = [objp] * len(img_pts)
    rms, K, D, rv, tv = cv2.calibrateCamera(obj, img_pts, size, None, None)
    errs = []
    for i in range(len(img_pts)):
        proj, _ = cv2.projectPoints(objp, rv[i], tv[i], K, D)
        errs.append(np.sqrt(np.mean(np.sum((proj - img_pts[i]) ** 2, -1))))
    return rms, K, D, np.array(errs)


def detect_pattern(cap, preview, show):
    """Find the inner-corner count by trying common sizes (largest first) on live frames."""
    sizes = sorted([(c, r) for c in range(4, 14) for r in range(3, 11) if c >= r], key=lambda s: -s[0] * s[1])
    print('[calib] 自动识别棋盘规格：请把整块棋盘举到画面中央，保持不动', flush=True)
    t0, last = time.time(), 0.0
    while True:
        ok, frame = cap.read()
        if not ok:
            continue
        preview.set(frame)
        if show:
            cv2.imshow('calib', frame)
            if cv2.waitKey(1) & 0xFF in (ord('q'), 27):
                return None
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        for s in sizes:
            if cv2.findChessboardCorners(gray, s, cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_FAST_CHECK)[0]:
                # the FAST_CHECK hit can be a sub-board; confirm the full detection
                if cv2.findChessboardCorners(gray, s, cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE)[0]:
                    print(f'[calib] 识别到棋盘：{s[0]}x{s[1]} 内角点', flush=True)
                    return s
        if time.time() - last > 3:
            last = time.time()
            print(f'[calib] 还没看到棋盘 ({time.time() - t0:.0f}s, 平均亮度 {gray.mean():.0f}/255)', flush=True)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--device', default='/dev/vln_cam')
    ap.add_argument('--width', type=int, default=640)
    ap.add_argument('--height', type=int, default=480)
    ap.add_argument('--cols', type=int, default=0, help='inner corners per row (0 = auto-detect)')
    ap.add_argument('--rows', type=int, default=0, help='inner corners per column (0 = auto-detect)')
    ap.add_argument('--square', type=float, default=0.025,
                    help='square size in metres (does not affect intrinsics, only extrinsic scale)')
    ap.add_argument('--min-samples', type=int, default=25)
    ap.add_argument('--max-samples', type=int, default=45)
    ap.add_argument('--out', default=DEFAULT_OUT)
    ap.add_argument('--save-dir', default=os.path.expanduser('~/calib_' + time.strftime('%Y%m%d_%H%M%S')))
    ap.add_argument('--port', type=int, default=8090)
    ap.add_argument('--no-window', action='store_true')
    a = ap.parse_args()

    cap = cv2.VideoCapture(a.device, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, a.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, a.height)
    ok, frame = cap.read()
    if not ok:
        sys.exit(f'cannot read {a.device} (is usb_cam still running?)')
    h, w = frame.shape[:2]
    if (w, h) != (a.width, a.height):
        sys.exit(f'camera gave {w}x{h}, wanted {a.width}x{a.height}')
    os.makedirs(a.save_dir, exist_ok=True)

    preview = Preview()
    serve(preview, a.port)
    show = not a.no_window and bool(os.environ.get('DISPLAY'))
    if show:
        cv2.namedWindow('calib', cv2.WINDOW_NORMAL)
        cv2.setWindowProperty('calib', cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)

    pattern = (a.cols, a.rows)
    if not a.cols or not a.rows:
        pattern = detect_pattern(cap, preview, show)
        if pattern is None:
            sys.exit('[calib] stopped before a board was found')
        a.cols, a.rows = pattern
    objp = np.zeros((a.cols * a.rows, 3), np.float32)
    objp[:, :2] = np.mgrid[0:a.cols, 0:a.rows].T.reshape(-1, 2) * a.square
    crit = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.01)

    img_pts, params, cov = [], [], Coverage()
    prev, still_since = None, None
    stop = {'v': False}
    signal.signal(signal.SIGINT, lambda *_: stop.__setitem__('v', True))
    last_print = 0.0
    print(f'[calib] {w}x{h}, board {a.cols}x{a.rows} inner corners, {a.square*1000:.0f} mm squares', flush=True)
    print(f'[calib] live view: http://<robot-ip>:{a.port}/  samples -> {a.save_dir}', flush=True)

    while not stop['v']:
        ok, frame = cap.read()
        if not ok:
            continue
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        found, corners = cv2.findChessboardCorners(
            gray, pattern, cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE | cv2.CALIB_CB_FAST_CHECK)
        vis = frame.copy()
        status = '未检测到棋盘'
        now = time.time()
        if found:
            corners = cv2.cornerSubPix(gray, corners, (5, 5), (-1, -1), crit)
            cv2.drawChessboardCorners(vis, pattern, corners, found)
            moving = prev is not None and np.abs(corners - prev).mean() > 1.5
            prev = corners
            if moving or still_since is None:
                still_since = now
            p = board_params(corners, a.cols, a.rows, w, h)
            novel = not params or np.min(np.abs(np.array(params) - p).sum(1)) > 0.12
            if moving:
                status = '保持不动...'
            elif not novel:
                status = '这个姿态已采过，换一个'
            elif now - still_since > 0.4:
                img_pts.append(corners.copy())
                params.append(p)
                cov.add(p)
                cv2.imwrite(os.path.join(a.save_dir, f'{len(img_pts):02d}.png'), frame)
                status = f'已采集第 {len(img_pts)} 张'
                print(f'[calib] +sample {len(img_pts)}  centre=({p[0]:.2f},{p[1]:.2f}) '
                      f'size={p[2]:.2f} tilt=({p[3]:+.2f},{p[4]:+.2f})', flush=True)
                still_since = None
                vis[:] = 255 - vis  # flash
        else:
            prev, still_since = None, None

        miss = cov.missing()
        done = len(img_pts) >= a.max_samples or (len(img_pts) >= a.min_samples and not miss)
        for r in range(3):
            for c in range(3):
                col = (0, 200, 0) if cov.pos[r, c] else (0, 0, 255)
                cv2.rectangle(vis, (c * w // 3, r * h // 3), ((c + 1) * w // 3 - 1, (r + 1) * h // 3 - 1), col, 1)
        cv2.putText(vis, f'{len(img_pts)}/{a.min_samples}', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)
        preview.set(vis)
        if show:
            cv2.imshow('calib', vis)
            if cv2.waitKey(1) & 0xFF in (ord('q'), 27):
                break
        if now - last_print > 3:
            last_print = now
            print(f'[calib] {len(img_pts)} 张 | {status} | 还缺: {"；".join(miss[:3]) or "无"}', flush=True)
        if done:
            break

    cap.release()
    if show:
        cv2.destroyAllWindows()
    if len(img_pts) < 10:
        sys.exit(f'[calib] only {len(img_pts)} samples, need >= 10')

    print(f'[calib] calibrating with {len(img_pts)} views...', flush=True)
    rms, K, D, errs = calibrate(objp, img_pts, (w, h))
    keep = errs < max(2.0 * np.median(errs), 0.5)
    if (~keep).any() and keep.sum() >= 10:
        print(f'[calib] dropping {int((~keep).sum())} outlier views: {np.where(~keep)[0] + 1}', flush=True)
        img_pts = [p for p, k in zip(img_pts, keep) if k]
        rms, K, D, errs = calibrate(objp, img_pts, (w, h))
    D = D.ravel()
    newK, _ = cv2.getOptimalNewCameraMatrix(K, D, (w, h), 0)
    P = np.hstack([newK, np.zeros((3, 1))])
    hfov = np.degrees(2 * np.arctan(w / (2 * K[0, 0])))
    vfov = np.degrees(2 * np.arctan(h / (2 * K[1, 1])))

    print(f'[calib] RMS {rms:.3f} px (good < 0.5, ok < 1.0)')
    print(f'[calib] fx={K[0,0]:.2f} fy={K[1,1]:.2f} cx={K[0,2]:.2f} cy={K[1,2]:.2f}  HFOV {hfov:.1f} VFOV {vfov:.1f} deg')
    print(f'[calib] dist k1={D[0]:+.4f} k2={D[1]:+.4f} p1={D[2]:+.5f} p2={D[3]:+.5f} k3={D[4]:+.4f}')
    print(f'[calib] VLN server: --intrinsic {K[0,0]:.1f} {K[1,1]:.1f} {K[0,2]:.1f} {K[1,2]:.1f}')
    write_yaml(a.out, w, h, K, D, P, 'vln_cam', rms, len(img_pts))
    print(f'[calib] wrote {a.out}')

    # side-by-side raw / undistorted of the last sample for a visual check
    raw = cv2.imread(os.path.join(a.save_dir, sorted(os.listdir(a.save_dir))[-1]))
    und = cv2.undistort(raw, K, D, None, newK)
    cv2.imwrite(os.path.join(a.save_dir, 'check_raw_vs_undistorted.jpg'), np.hstack([raw, und]))
    print(f'[calib] check image: {a.save_dir}/check_raw_vs_undistorted.jpg', flush=True)


if __name__ == '__main__':
    main()
