"""Scan-matching odometry: the car's motion between scans from the whole lidar scan, not from the wheels.

Each scan is aligned (point-to-point ICP, trimmed) against a local map made of the last keyframe scans in the
world frame, starting from the dead-reckoned guess (wheel distance along the IMU heading). Walls, planters,
benches, kerbs and every cone take part, so the alignment pins the pose to a few centimetres wherever there
is structure in range; where there is none (a bare field) the match is rejected and the guess stands, so it
is never worse than the wheels. The result replaces the motion edge the graph gets between keyframes: the
cone association then starts from a pose that is already right, instead of one that is a few decimetres off.

Pure numpy + scipy, a few milliseconds a scan at 10 Hz.
"""
import math

import numpy as np
from scipy.spatial import cKDTree


def _rot(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s], [s, c]])


class ScanOdometry:
    def __init__(self, max_range=12.0, min_range=0.3, step=2, keyframe_dist=0.3, keyframe_turn=0.15, submap=30,
                 iterations=10, match_dist=0.5, final_dist=0.2, trim=0.8, min_matched=0.5, max_shift=0.35, max_turn=0.1):
        self.p = dict(max_range=max_range, min_range=min_range, step=step, keyframe_dist=keyframe_dist, keyframe_turn=keyframe_turn,
                      submap=submap, iterations=iterations, match_dist=match_dist, final_dist=final_dist, trim=trim,
                      min_matched=min_matched, max_shift=max_shift, max_turn=max_turn)
        self.pose = np.zeros(2)          # world position
        self.yaw = None                  # world heading (starts at the first IMU heading)
        self.keyframes = []              # (pose, yaw, world points)
        self.tree = None
        self.cloud = np.zeros((0, 2))
        self.last_imu = None
        self.matched = 0.0               # share of points matched on the last scan
        self.ok = False                  # whether the last scan's match was accepted
        self.rejected = 0

    def points(self, msg):
        rg = np.asarray(msg.ranges, dtype=float)
        ang = msg.angle_min + np.arange(len(rg)) * msg.angle_increment
        ok = np.isfinite(rg) & (rg > self.p["min_range"]) & (rg < self.p["max_range"])
        idx = np.flatnonzero(ok)[:: self.p["step"]]
        return np.c_[rg[idx] * np.cos(ang[idx]), rg[idx] * np.sin(ang[idx])]

    def _keyframe(self, P):
        W = P @ _rot(self.yaw).T + self.pose
        self.keyframes.append((self.pose.copy(), self.yaw, W))
        self.keyframes = self.keyframes[-self.p["submap"]:]
        self.cloud = np.vstack([k[2] for k in self.keyframes])
        self.tree = cKDTree(self.cloud)

    def update(self, msg, imu_yaw, ds):
        """One scan: `imu_yaw` the IMU heading now, `ds` the wheel distance since the last scan.
        Returns (dx_world, yaw): the motion since the last call in the world frame and the heading."""
        P = self.points(msg)
        if self.yaw is None:
            self.yaw, self.last_imu = imu_yaw, imu_yaw
            self._keyframe(P)
            self.ok = True
            return np.zeros(2), self.yaw
        # the guess: turn as the IMU turned, move as the wheels moved, along the heading
        dyaw = math.atan2(math.sin(imu_yaw - self.last_imu), math.cos(imu_yaw - self.last_imu))
        self.last_imu = imu_yaw
        yaw = self.yaw + dyaw
        pose = self.pose + ds * np.array([math.cos(yaw), math.sin(yaw)])
        guess = (pose.copy(), yaw)
        if len(P) < 50 or self.tree is None:
            self.ok = False
            self.pose, self.yaw = pose, yaw
            return pose - guess[0] + ds * np.array([math.cos(yaw), math.sin(yaw)]), yaw
        matched = 0.0
        for it in range(self.p["iterations"]):
            W = P @ _rot(yaw).T + pose
            radius = self.p["match_dist"] if it < 3 else self.p["final_dist"]
            d, j = self.tree.query(W, distance_upper_bound=radius)
            ok = np.isfinite(d)
            matched = ok.mean()
            if ok.sum() < 40:
                break
            # trimmed: the best share of the matches, so people and the odd stray return do not pull
            keep = np.flatnonzero(ok)
            if self.p["trim"] < 1.0:
                keep = keep[np.argsort(d[keep])[: max(40, int(self.p["trim"] * len(keep)))]]
            A, B = W[keep], self.cloud[j[keep]]
            ca, cb = A.mean(0), B.mean(0)
            H = (A - ca).T @ (B - cb)
            U, _, Vt = np.linalg.svd(H)
            R = Vt.T @ U.T
            if np.linalg.det(R) < 0:
                Vt[1] *= -1
                R = Vt.T @ U.T
            da = math.atan2(R[1, 0], R[0, 0])
            t = cb - R @ ca
            pose = R @ pose + t
            yaw = yaw + da
            if np.linalg.norm(t) < 0.002 and abs(da) < 2e-4:
                break
        shift = np.linalg.norm(pose - guess[0])
        turn = abs(math.atan2(math.sin(yaw - guess[1]), math.cos(yaw - guess[1])))
        self.matched = matched
        if matched < self.p["min_matched"] or shift > self.p["max_shift"] or turn > self.p["max_turn"]:
            # no structure to match, or the match ran away: the dead-reckoned guess stands
            self.ok = False
            self.rejected += 1
            pose, yaw = guess
        else:
            self.ok = True
        dx = pose - self.pose
        self.pose, self.yaw = pose, yaw
        last = self.keyframes[-1]
        if np.linalg.norm(self.pose - last[0]) > self.p["keyframe_dist"] or abs(self.yaw - last[1]) > self.p["keyframe_turn"]:
            self._keyframe(P)
        return dx, self.yaw
