#!/usr/bin/env python3
"""RoboMaster S1 low-level wheel test via the RoboMaster SDK (jeguzzi fork), without ROS.

Connect the M11 to the S1 intelligent controller's micro-USB port (RNDIS, robot = 192.168.42.2).
LIFT THE ROBOT so the wheels spin freely before running this.

  python3 tools/s1_wheel_test.py --rpm 60 --duration 2
"""
import argparse
import time

from robomaster import robot


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--conn", default="rndis", choices=["rndis", "ap", "sta"])
    ap.add_argument("--rpm", type=int, default=60, help="wheel speed, |rpm| <= 1000")
    ap.add_argument("--duration", type=float, default=2.0)
    a = ap.parse_args()

    ep = robot.Robot()
    ep.initialize(conn_type=a.conn)
    print("SN:", ep.get_sn(), "version:", ep.get_version())
    chassis = ep.chassis

    def on_esc(info):
        speed, angle, timestamp, state = info
        print(f"esc speed(rpm)={speed} angle={angle} state={state}")

    chassis.sub_esc(freq=5, callback=on_esc)
    try:
        # w1 front-right, w2 front-left, w3 rear-left, w4 rear-right (SDK convention)
        tests = [
            ("all forward", (a.rpm, a.rpm, a.rpm, a.rpm)),
            ("all backward", (-a.rpm, -a.rpm, -a.rpm, -a.rpm)),
            ("front-right only", (a.rpm, 0, 0, 0)),
            ("front-left only", (0, a.rpm, 0, 0)),
            ("rear-left only", (0, 0, a.rpm, 0)),
            ("rear-right only", (0, 0, 0, a.rpm)),
        ]
        for name, (w1, w2, w3, w4) in tests:
            print(f"--> {name}: w1..w4 = {w1} {w2} {w3} {w4}")
            chassis.drive_wheels(w1=w1, w2=w2, w3=w3, w4=w4, timeout=a.duration + 0.5)
            time.sleep(a.duration)
            chassis.drive_wheels(0, 0, 0, 0)
            time.sleep(0.5)
        print("--> drive_speed: x=0.2 m/s, then rotate z=30 deg/s")
        chassis.drive_speed(x=0.2, y=0, z=0, timeout=a.duration)
        time.sleep(a.duration)
        chassis.drive_speed(x=0, y=0, z=30, timeout=a.duration)
        time.sleep(a.duration)
    finally:
        chassis.drive_wheels(0, 0, 0, 0)
        chassis.unsub_esc()
        ep.close()


if __name__ == "__main__":
    main()
