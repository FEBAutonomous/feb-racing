"""feb_localizer: a memory driver whose memory is only as good as the Localizer you write.

Lap one (MAPPING): drives reactively off the raw scan, the same law as feb_reactive_competitor,
while stamping every scan into an occupancy grid at the Localizer's pose and recording the path
it drove. The lap closes when the pose says the car is back at the start.
Lap two on (RACING): follows the recorded path by pure pursuit on the Localizer's pose. With the
dead reckoning the Localizer ships with, the pose has drifted by then and the car follows a path
that is no longer where the track is. Fix the Localizer and this lap works.

`fallback: false` (the default, for the simulator) lets that failure happen: the fit of the scan to
the map is published on /feb/fit but not acted on. `fallback: true` (the car file) hands control
back to the reactive law whenever the fit drops under `fit_threshold`, so a wrong pose costs a
slower lap, not a bumper.

`oracle: true` (simulator only) replaces the Localizer's pose with the simulator's true position,
so you can see what lap two looks like with a perfect localiser before writing one.

In: /scan, /odom.  Out: /drive, and for Foxglove /feb/pose, /feb/path, /feb/map, /feb/fit, /feb/mode.
"""
import math
import os
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from sensor_msgs.msg import Imu, LaserScan
from std_msgs.msg import Float32, String

from .grid import Grid
from .localizer import Localizer

QOS = QoSProfile(reliability=QoSReliabilityPolicy.RELIABLE, durability=QoSDurabilityPolicy.VOLATILE,
                 history=QoSHistoryPolicy.KEEP_LAST, depth=10)
SCAN_QOS = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT, durability=QoSDurabilityPolicy.VOLATILE,
                      history=QoSHistoryPolicy.KEEP_LAST, depth=1)
WHEELBASE, HALF_WIDTH, MAX_STEER, MAX_STEER_RATE = 0.324, 0.135, 0.5236, 3.2
MAPPING, RACING = "MAPPING", "RACING"


class Driver(Node):
    def __init__(self):
        super().__init__("localizer_driver")
        defaults = dict(max_speed=2.5, race_speed=3.0, min_speed=0.8, lookahead=1.2, fallback=False, fit_threshold=0.45,
                        min_lap_m=15.0, close_radius_m=1.5, grid_res=0.05, grid_size_m=60.0, path_step=0.3,
                        range_cap=8.0, margin=0.3, fov=1.4, disparity=0.3, lat_accel=3.5, brake_decel=3.0,
                        accel_limit=3.5, decel_limit=5.5, brake_margin=0.45, steer_tau=0.12, steer_delay=0.25, oracle=False)
        for k, v in defaults.items():
            self.declare_parameter(k, v)
        self.p = {k: self.get_parameter(k).value for k in defaults}

        self.loc = Localizer()
        self.grid = Grid(self.p["grid_res"], self.p["grid_size_m"])
        self.mode = MAPPING
        self.path = []                 # (x, y) every path_step m on the mapping lap
        self.dist = 0.0
        self.speed = 0.0
        self.yaw_rate = 0.0
        self.steer = 0.0
        self.steer_slow = 0.0
        self.speed_cmd = 0.0
        self.last_odom_t = None
        self.last_scan_t = None
        self.last_map_pub = 0.0
        self.path_i = 0
        self.fit = 0.0
        self.oracle_pose = None

        self.pub_drive = self.create_publisher(AckermannDriveStamped, "/drive", QOS)
        self.pub_pose = self.create_publisher(PoseStamped, "/feb/pose", 10)
        self.pub_path = self.create_publisher(Path, "/feb/path", 10)
        self.pub_map = self.create_publisher(OccupancyGrid, "/feb/map", 1)
        self.pub_fit = self.create_publisher(Float32, "/feb/fit", 10)
        self.pub_mode = self.create_publisher(String, "/feb/mode", 10)
        self.create_subscription(LaserScan, "/scan", self.on_scan, SCAN_QOS)
        self.create_subscription(Odometry, "/odom", self.on_odom, QOS)
        if self.p["oracle"]:
            self.create_subscription(Point, "/autodrive/roboracer_1/ips", self.on_oracle_pos, 10)
            self.create_subscription(Imu, "/autodrive/roboracer_1/imu", self.on_oracle_imu, 10)
            self.oracle_origin = None
            self.oracle_yaw = None
        self.get_logger().info("localizer driver up: %s, fallback %s%s" % (
            type(self.loc).__name__, "on" if self.p["fallback"] else "off", ", ORACLE pose" if self.p["oracle"] else ""))

    # ---------------------------------------------------------------- inputs

    def on_odom(self, msg):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.speed = msg.twist.twist.linear.x
        self.yaw_rate = msg.twist.twist.angular.z
        if self.last_odom_t is not None:
            dt = min(max(t - self.last_odom_t, 0.0), 0.2)
            self.loc.predict(self.speed, self.yaw_rate, dt)
            self.dist += abs(self.speed) * dt
        self.last_odom_t = t

    def on_oracle_imu(self, msg):
        q = msg.orientation
        self.oracle_yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))

    def on_oracle_pos(self, msg):
        if self.oracle_yaw is None:
            return
        if self.oracle_origin is None:
            self.oracle_origin = (msg.x, msg.y, self.oracle_yaw)
        ox, oy, oyaw = self.oracle_origin
        dx, dy = msg.x - ox, msg.y - oy
        c, s = math.cos(-oyaw), math.sin(-oyaw)
        self.loc.x, self.loc.y = dx * c - dy * s, dx * s + dy * c
        self.loc.yaw = self.oracle_yaw - oyaw

    # ---------------------------------------------------------------- the lap

    def on_scan(self, msg):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        dt = min(t - self.last_scan_t, 0.5) if self.last_scan_t else 0.0
        self.last_scan_t = t
        if dt <= 0.0:
            return
        ranges = np.asarray(msg.ranges, dtype=float)
        ranges[~np.isfinite(ranges)] = msg.range_max
        angles = msg.angle_min + np.arange(len(ranges)) * msg.angle_increment
        good = (ranges > msg.range_min) & (ranges < min(msg.range_max, self.p["range_cap"]))
        points = np.stack([ranges[good] * np.cos(angles[good]), ranges[good] * np.sin(angles[good])], axis=1)

        if self.mode == MAPPING:
            self.loc.update_mapping(points, self.grid)
            pose = (self.loc.x, self.loc.y, self.loc.yaw)
            self.fit = self.grid.fit(points, pose) if self.path else 1.0
            self.grid.insert(points, pose)
            if not self.path or math.hypot(pose[0] - self.path[-1][0], pose[1] - self.path[-1][1]) >= self.p["path_step"]:
                self.path.append((pose[0], pose[1]))
            if self.dist > self.p["min_lap_m"] and math.hypot(pose[0], pose[1]) < self.p["close_radius_m"] and math.cos(pose[2]) > 0.3:
                self.mode = RACING
                self.path_i = 0
                self.get_logger().info("lap closed after %.1f m with %d path points: RACING on the recorded path" % (self.dist, len(self.path)))
            self.reactive(ranges, angles, dt)
        else:
            self.loc.update_racing(points, self.grid)
            pose = (self.loc.x, self.loc.y, self.loc.yaw)
            self.fit = self.grid.fit(points, pose)
            if self.p["fallback"] and self.fit < self.p["fit_threshold"]:
                self.reactive(ranges, angles, dt)
            else:
                self.pursue(pose, dt)
        self.debug(msg.header, pose)

    # ---------------------------------------------------------------- lap two: the recorded path

    def pursue(self, pose, dt):
        x, y, yaw = pose
        n = len(self.path)
        # nearest path point, searched forward from the last one so the lap wraps cleanly
        best, bd = self.path_i, math.inf
        for k in range(n):
            i = (self.path_i + k) % n
            d = math.hypot(self.path[i][0] - x, self.path[i][1] - y)
            if d < bd:
                best, bd = i, d
            if k > 40 and d > bd + 2.0:
                break
        self.path_i = best
        ld = self.p["lookahead"]
        j, acc = best, 0.0
        while acc < ld:
            nj = (j + 1) % n
            acc += math.hypot(self.path[nj][0] - self.path[j][0], self.path[nj][1] - self.path[j][1])
            j = nj
            if j == best:
                break
        tx, ty = self.path[j]
        dx, dy = tx - x, ty - y
        alpha = math.atan2(dy, dx) - yaw
        alpha = math.atan2(math.sin(alpha), math.cos(alpha))
        alpha -= self.speed * math.tan(self.steer) / WHEELBASE * self.p["steer_delay"]
        dist = max(math.hypot(dx, dy), 0.3)
        wanted = float(np.clip(math.atan2(2.0 * WHEELBASE * math.sin(alpha), dist), -MAX_STEER, MAX_STEER))
        self.slew_steer(wanted, dt)
        curvature = abs(2.0 * math.sin(alpha) / dist)
        v_bend = math.sqrt(self.p["lat_accel"] / curvature) if curvature > 1e-3 else self.p["race_speed"]
        self.slew_speed(max(self.p["min_speed"], min(self.p["race_speed"], v_bend)), dt)
        self.publish(self.speed_cmd, self.steer)

    # ---------------------------------------------------------------- lap one: reactive (feb_reactive_competitor's law, compact)

    def reactive(self, ranges, angles, dt):
        r = np.minimum(ranges, self.p["range_cap"])
        ext = self.extend(r, angles[1] - angles[0])
        fov = np.abs(angles) <= self.p["fov"]
        cand = np.where(fov, ext, 0.0)
        best = float(cand.max())
        if best <= 0.0:
            self.publish(0.0, 0.0)
            return
        i = int(np.argmax(cand))
        lo = hi = i
        floor = 0.6 * best
        while lo > 0 and cand[lo - 1] >= floor:
            lo -= 1
        while hi < len(cand) - 1 and cand[hi + 1] >= floor:
            hi += 1
        bearing = 0.5 * angles[i] + 0.5 * 0.5 * (angles[lo] + angles[hi])
        reach = float(min(ext[i], cand[lo:hi + 1].min()))
        ld = min(max(0.8 + 0.3 * self.speed, 0.8), 1.8, max(reach, 0.3))
        alpha = bearing - self.speed * math.tan(self.steer) / WHEELBASE * (self.p["steer_delay"] + dt)
        wanted = float(np.clip(math.atan2(2.0 * WHEELBASE * math.sin(alpha), ld), -MAX_STEER, MAX_STEER))
        self.slew_steer(wanted, dt)
        x, y = ranges * np.cos(angles), ranges * np.sin(angles)
        corridor = (np.abs(y) < HALF_WIDTH + 0.1) & (x > 0.0)
        ahead = float(np.percentile(x[corridor], 3.0)) if corridor.any() else self.p["range_cap"]
        a, lag = self.p["brake_decel"], self.p["steer_delay"]
        room = max(0.0, min(ahead, reach) - self.p["brake_margin"])
        v_brake = -a * lag + math.sqrt(a * a * lag * lag + 2.0 * a * room)
        k_target = abs(2.0 * math.sin(bearing) / max(ld, 0.3))
        v_bend = math.sqrt(self.p["lat_accel"] / k_target) if k_target > 1e-3 else self.p["max_speed"]
        goal = max(self.p["min_speed"], min(self.p["max_speed"], v_bend, v_brake))
        if ahead < self.p["brake_margin"]:
            goal = 0.0
        self.slew_speed(goal, dt)
        self.publish(self.speed_cmd, self.steer)

    def extend(self, r, inc):
        out = r.copy()
        d = np.diff(r)
        jumps = np.flatnonzero(np.abs(d) > self.p["disparity"])
        width = HALF_WIDTH + self.p["margin"]
        for j in sorted(jumps, key=lambda j: min(r[j], r[j + 1])):
            rn = min(r[j], r[j + 1])
            k = int(math.ceil(math.atan2(width, max(rn, 0.05)) / inc))
            lo, hi = (j + 1, min(len(out), j + 1 + k)) if d[j] > 0 else (max(0, j + 1 - k), j + 1)
            out[lo:hi] = np.minimum(out[lo:hi], rn)
        return out

    # ---------------------------------------------------------------- outputs

    def slew_steer(self, wanted, dt):
        step = (wanted - self.steer) * (1.0 - math.exp(-dt / self.p["steer_tau"]))
        self.steer += float(np.clip(step, -MAX_STEER_RATE * dt, MAX_STEER_RATE * dt))

    def slew_speed(self, goal, dt):
        self.speed_cmd += float(np.clip(goal - self.speed_cmd, -self.p["decel_limit"] * dt, self.p["accel_limit"] * dt))

    def publish(self, speed, steer):
        cmd = AckermannDriveStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = "base_link"
        cmd.drive.speed, cmd.drive.steering_angle = float(speed), float(steer)
        self.pub_drive.publish(cmd)

    def debug(self, header, pose):
        ps = PoseStamped(header=header)
        ps.header.frame_id = "map"
        ps.pose.position.x, ps.pose.position.y = pose[0], pose[1]
        ps.pose.orientation.z, ps.pose.orientation.w = math.sin(pose[2] / 2), math.cos(pose[2] / 2)
        self.pub_pose.publish(ps)
        self.pub_fit.publish(Float32(data=float(self.fit)))
        self.pub_mode.publish(String(data="%s fit %.2f" % (self.mode, self.fit)))
        now = time.monotonic()
        if now - self.last_map_pub > 2.0:
            self.last_map_pub = now
            path = Path(header=ps.header)
            for x, y in self.path:
                q = PoseStamped(header=ps.header)
                q.pose.position.x, q.pose.position.y = x, y
                q.pose.orientation.w = 1.0
                path.poses.append(q)
            self.pub_path.publish(path)
            g = OccupancyGrid(header=ps.header)
            g.info.resolution = self.grid.res
            g.info.width = g.info.height = self.grid.size
            g.info.origin.position.x = g.info.origin.position.y = -self.grid.half * self.grid.res
            g.info.origin.orientation.w = 1.0
            g.data = (self.grid.occupied().astype(np.int8) * 100).ravel().tolist()
            self.pub_map.publish(g)


def main():
    rclpy.init()
    node = Driver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publish(0.0, 0.0)
        node.destroy_node()


if __name__ == "__main__":
    main()
