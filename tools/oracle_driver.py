#!/usr/bin/env python3
"""A test driver for the fixed-layout events: the house driver (centreline pursuit from the
simulator's true position) on an open programme, braking to a stop at its end. For checking
the harness, never an entry. Runs inside the racing container, where feb_tools is installed:

    ./feb practice --track skidpad --ws tools --headless \\
        --stack "python3 /home/autodrive_devkit/src/stack/oracle_driver.py --track skidpad"

The track folder must be under tools/_tracks/<name>/ (copied there for the run).
"""
import argparse
import math
import sys

import numpy as np
import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Float32

from feb_tools.house_driver import MAX_STEER, SLIP_PEAK, STEER_DELAY, WHEELBASE, HouseDriver

STACK = "/home/autodrive_devkit/src/stack"


class OracleDriver(HouseDriver):
    def __init__(self):
        super().__init__()
        self.done = False
        self.i_last = 0
        self.pub_drive = self.create_publisher(AckermannDriveStamped, "/drive", 10)
        n = len(self.path)
        tail = max(2, int(1.0 / self.ds))
        self.kappa[:tail] = 0.0          # an open path: no wrap-around curvature at either end
        self.kappa[-tail:] = 0.0
        self.v_ref[-int(4.0 / self.ds):] = self.p["min_speed"]
        self.brake_from = n - int(8.0 / self.ds)       # the path runs 2 m past the stop zone: brake 6 m before its end

    def nearest(self, pos):
        """Progress-aware: the programme crosses itself, so the nearest point is searched only a
        little behind and some way ahead of where the car last was."""
        lo, hi = max(0, self.i_last - int(1.0 / self.ds)), min(len(self.path), self.i_last + int(6.0 / self.ds))
        i = lo + int(np.argmin(np.linalg.norm(self.path[lo:hi] - pos, axis=1)))
        self.i_last = max(self.i_last, i)
        return self.i_last

    def step(self):
        if self.position is None or self.yaw is None:
            return
        i_here = self.nearest(self.position)
        if self.done or i_here >= self.brake_from:
            self.done = True
            self.drive(0.0, 0.0)                               # speed 0 on /drive: the adapter brakes
            return
        # the house driver's step, with the windowed index in place of a global nearest search
        v = max(self.speed, 0.3)
        yaw = self.yaw + self.yaw_rate * STEER_DELAY
        pos = self.position + v * STEER_DELAY * np.array([math.cos(self.yaw), math.sin(self.yaw)])
        n = len(self.path)
        i0 = min(n - 1, i_here + int(np.argmin(np.linalg.norm(self.path[i_here:min(n, i_here + int(2.0 / self.ds))] - pos, axis=1))))
        ld = 0.5 + 0.3 * max(v, 1.0) + v * self.dt
        j = min(n - 1, i0 + max(1, int(ld / self.ds)))
        dx, dy = self.path[j] - pos
        eta = math.atan2(dy, dx) - yaw
        eta = math.atan2(math.sin(eta), math.cos(eta))
        kappa_fb = float(np.clip(2.0 * math.sin(eta) / max(math.hypot(dx, dy), 0.3), -0.8, 0.8))
        kappa_ff = 0.6 * float(self.kappa[min(n - 1, i0 + max(1, int(0.15 * v / self.ds)))])
        delta = float(np.clip(math.atan((kappa_fb + kappa_ff) * WHEELBASE), -SLIP_PEAK, SLIP_PEAK))
        self.steer += float(np.clip(delta - self.steer, -3.2 * self.dt, 3.2 * self.dt))
        window = [min(n - 1, i0 + k) for k in range(max(2, int(0.3 * v / self.ds)))]
        target = float(self.v_ref[window].min())
        self.v_target += (target - self.v_target) * min(1.0, self.dt / 0.3)
        self.drive(self.v_target, self.steer)

    def drive(self, speed, steer):
        # through the house layer like any driver: /drive -> supervisor -> sim adapter, which owns
        # the throttle. Publishing throttle_command directly would fight the adapter's zeros.
        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.drive.speed = float(speed)
        msg.drive.steering_angle = float(steer)
        self.pub_drive.publish(msg)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--track", required=True)
    ap.add_argument("--max-speed", type=float, default=3.0)
    args, _ = ap.parse_known_args()
    rclpy.init(args=["--ros-args", "-p", f"track:={STACK}/_tracks/{args.track}", "-p", f"max_speed:={args.max_speed}"])
    node = OracleDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.drive(0.0, 0.0)
        node.destroy_node()


if __name__ == "__main__":
    main()
