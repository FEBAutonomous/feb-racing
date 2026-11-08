#!/usr/bin/env python3
"""FEB Auto dash relay: runs inside the box, subscribes to whatever the stack publishes and prints one
JSON line per tick (10 Hz) with the latest values and message rates. The host side (feb) reads these
lines over `docker exec -i` and serves them to browsers; nothing here opens a port.

Topic types are resolved at runtime, so /feb/map works both as a PoseArray (the cone racer) and as
an OccupancyGrid (the walls racer). Unknown types are listed with their rate only.

Each line: {"t": seconds, "nodes": [...], "topics": {name: {"type": short, "hz": float, "data": ...}}}
Heavy topics (scan, grids, images) are sent at most a few times per second, the rest at every tick.
"""
import base64, json, math, sys, time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

TICK = 0.1
SLOW = {"sensor_msgs/msg/LaserScan": 0.2, "nav_msgs/msg/OccupancyGrid": 0.5, "sensor_msgs/msg/Image": 0.5,
        "sensor_msgs/msg/CompressedImage": 0.5, "nav_msgs/msg/Path": 0.25, "geometry_msgs/msg/PoseArray": 0.2}
SKIP = ("/rosout", "/parameter_events", "/tf", "/tf_static", "/autodrive/", "/feb/cone_order", "/feb/order_cones")
QOS = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE,
                 history=HistoryPolicy.KEEP_LAST)   # best effort matches reliable and best-effort publishers alike


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def r2(v): return round(float(v), 2)
def r3(v): return round(float(v), 3)


def thin(seq, n):
    """at most n items, evenly spaced, the last one kept"""
    if len(seq) <= n:
        return list(seq)
    step = len(seq) / float(n)
    out = [seq[int(i * step)] for i in range(n)]
    out[-1] = seq[-1]
    return out


# ---- serialisers: message -> small JSON-able dict (or None to skip)

def ser_scan(m):
    r = [None if (math.isinf(v) or math.isnan(v)) else r2(v) for v in m.ranges]
    n = len(r)
    k = max(1, n // 360)
    r = r[::k]
    return {"frame": m.header.frame_id, "amin": r3(m.angle_min), "ainc": r3(m.angle_increment * k),
            "rmax": r2(m.range_max), "r": r, "n": n}


def ser_odom(m):
    p, q = m.pose.pose.position, m.pose.pose.orientation
    return {"frame": m.header.frame_id, "x": r3(p.x), "y": r3(p.y), "yaw": r3(yaw_of(q)),
            "v": r3(m.twist.twist.linear.x), "w": r3(m.twist.twist.angular.z)}


def ser_pose_stamped(m):
    p, q = m.pose.position, m.pose.orientation
    return {"frame": m.header.frame_id, "x": r3(p.x), "y": r3(p.y), "yaw": r3(yaw_of(q))}


def ser_pose_array(m):
    # the racers put the cone colour code in orientation.w (0 unknown, 1 blue, 2 yellow, 3 orange)
    pts = [[r2(p.position.x), r2(p.position.y), int(round(p.orientation.w))] for p in m.poses]
    return {"frame": m.header.frame_id, "pts": pts}


def ser_path(m):
    pts = [[r2(p.pose.position.x), r2(p.pose.position.y), r2(p.pose.position.z)] for p in m.poses]
    return {"frame": m.header.frame_id, "pts": thin(pts, 400), "n": len(pts)}


def ser_ackermann(m):
    return {"speed": r3(m.drive.speed), "steer": r3(m.drive.steering_angle)}


def ser_string(m): return m.data
def ser_bool(m): return bool(m.data)
def ser_float(m): return r3(m.data)
def ser_int(m): return int(m.data)


def ser_battery(m):
    return {"v": r2(m.voltage), "pct": None if math.isnan(m.percentage) else r2(m.percentage * 100.0), "i": r2(m.current)}


def ser_imu(m):
    return {"wz": r3(m.angular_velocity.z), "ax": r2(m.linear_acceleration.x), "ay": r2(m.linear_acceleration.y)}


def ser_joy(m):
    return {"axes": [r2(a) for a in m.axes], "buttons": list(m.buttons)}


def ser_grid(m):
    import numpy as np
    try:
        import cv2
    except ImportError:
        return {"w": m.info.width, "h": m.info.height, "res": r3(m.info.resolution)}
    w, h = m.info.width, m.info.height
    if w == 0 or h == 0:
        return None
    a = np.array(m.data, dtype=np.int16).reshape(h, w)
    img = np.full((h, w), 110, np.uint8)        # unknown: mid grey
    img[a == 0] = 235                           # free: light
    img[a > 50] = 25                            # occupied: dark
    # row 0 is the grid's y = 0 row; flip so the image's top is +y (north up)
    img = img[::-1]
    scale = min(1.0, 320.0 / max(w, h))
    if scale < 1.0:
        img = cv2.resize(img, (max(1, int(w * scale)), max(1, int(h * scale))), interpolation=cv2.INTER_AREA)
    ok, buf = cv2.imencode(".png", img)
    o = m.info.origin.position
    return {"frame": m.header.frame_id, "w": w, "h": h, "res": r3(m.info.resolution), "ox": r3(o.x), "oy": r3(o.y),
            "png": base64.b64encode(buf.tobytes()).decode("ascii") if ok else None}


def ser_image(m):
    import numpy as np
    try:
        import cv2
    except ImportError:
        return {"w": m.width, "h": m.height, "enc": m.encoding}
    enc = m.encoding.lower()
    buf = np.frombuffer(bytes(m.data), np.uint8)
    try:
        if enc in ("rgb8", "bgr8"):
            img = buf.reshape(m.height, m.step // 3, 3)[:, : m.width]
            if enc == "rgb8":
                img = img[:, :, ::-1]
        elif enc in ("rgba8", "bgra8"):
            img = buf.reshape(m.height, m.step // 4, 4)[:, : m.width, :3]
            if enc == "rgba8":
                img = img[:, :, ::-1]
        elif enc in ("mono8", "8uc1"):
            img = buf.reshape(m.height, m.step)[:, : m.width]
        else:
            return {"w": m.width, "h": m.height, "enc": m.encoding}
    except ValueError:
        return {"w": m.width, "h": m.height, "enc": m.encoding}
    scale = 320.0 / max(1, m.width)
    if scale < 1.0:
        img = cv2.resize(img, (320, max(1, int(m.height * scale))), interpolation=cv2.INTER_AREA)
    ok, jpg = cv2.imencode(".jpg", np.ascontiguousarray(img), [cv2.IMWRITE_JPEG_QUALITY, 60])
    return {"w": m.width, "h": m.height, "enc": m.encoding, "jpg": base64.b64encode(jpg.tobytes()).decode("ascii") if ok else None}


def ser_compressed(m):
    return {"enc": m.format, "jpg": base64.b64encode(bytes(m.data)).decode("ascii") if "jpeg" in m.format.lower() or "jpg" in m.format.lower() else None}


SERIALISERS = {
    "sensor_msgs/msg/LaserScan": ser_scan,
    "nav_msgs/msg/Odometry": ser_odom,
    "geometry_msgs/msg/PoseStamped": ser_pose_stamped,
    "geometry_msgs/msg/PoseArray": ser_pose_array,
    "nav_msgs/msg/Path": ser_path,
    "nav_msgs/msg/OccupancyGrid": ser_grid,
    "ackermann_msgs/msg/AckermannDriveStamped": ser_ackermann,
    "std_msgs/msg/String": ser_string,
    "std_msgs/msg/Bool": ser_bool,
    "std_msgs/msg/Float32": ser_float,
    "std_msgs/msg/Float64": ser_float,
    "std_msgs/msg/Int32": ser_int,
    "sensor_msgs/msg/BatteryState": ser_battery,
    "sensor_msgs/msg/Imu": ser_imu,
    "sensor_msgs/msg/Joy": ser_joy,
    "sensor_msgs/msg/Image": ser_image,
    "sensor_msgs/msg/CompressedImage": ser_compressed,
}


def msg_class(type_name):
    pkg, _, name = type_name.split("/")
    mod = __import__(pkg + ".msg", fromlist=[name])
    return getattr(mod, name)


class Relay(Node):
    def __init__(self):
        super().__init__("feb_dash_relay")
        self.subs = {}          # topic -> subscription
        self.types = {}         # topic -> type name
        self.latest = {}        # topic -> (stamp, serialised) for the next tick
        self.sent_at = {}       # topic -> time of the last send (slow topics)
        self.counts = {}        # topic -> [count in the current window, window start, hz]
        self.last = {}          # topic -> last message, serialised lazily for slow topics
        self.create_timer(2.0, self.discover)
        self.create_timer(TICK, self.tick)
        self.discover()

    def discover(self):
        for name, types in self.get_topic_names_and_types():
            if name in self.subs or any(name.startswith(s) for s in SKIP):
                continue
            t = types[0]
            self.types[name] = t
            self.counts.setdefault(name, [0, time.monotonic(), 0.0])
            if t not in SERIALISERS:
                self.subs[name] = None      # listed, not read
                continue
            try:
                cls = msg_class(t)
            except (ImportError, AttributeError, ValueError):
                self.subs[name] = None
                continue
            self.subs[name] = self.create_subscription(cls, name, lambda m, n=name: self.on_msg(n, m), QOS)

    def on_msg(self, name, m):
        c = self.counts[name]
        c[0] += 1
        self.last[name] = m

    def tick(self):
        now = time.monotonic()
        out = {}
        for name, t in self.types.items():
            c = self.counts[name]
            if now - c[1] >= 1.0:
                c[2] = round(c[0] / (now - c[1]), 1)
                c[0], c[1] = 0, now
            entry = {"type": t.split("/")[-1], "hz": c[2] if self.subs.get(name) is not None else None}
            m = self.last.pop(name, None)
            if m is not None:
                period = SLOW.get(t, 0.0)
                if now - self.sent_at.get(name, 0.0) >= period:
                    try:
                        data = SERIALISERS[t](m)
                    except Exception as e:      # a serialiser must never take the relay down
                        data = {"error": str(e)[:80]}
                    if data is not None:
                        entry["data"] = data
                        self.sent_at[name] = now
                else:
                    self.last[name] = m          # keep the newest for the next slow slot
            out[name] = entry
        line = {"t": round(time.time(), 2), "topics": out}
        if int(now * 10) % 20 == 0:   # every 2 s
            line["nodes"] = sorted(n for n in self.get_node_names() if n != "feb_dash_relay" and not n.startswith("_"))
        sys.stdout.write(json.dumps(line, separators=(",", ":")) + "\n")
        sys.stdout.flush()


def watch_stdin():
    """The host closes our stdin when it stops (or dies); then this process has no reader and must go."""
    try:
        while sys.stdin.readline():
            pass
    except (OSError, ValueError):
        pass
    import os
    os._exit(0)


def main():
    rclpy.init(args=sys.argv)
    node = Relay()
    import threading
    threading.Thread(target=watch_stdin, daemon=True).start()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, BrokenPipeError):
        pass
    finally:
        try:
            node.destroy_node()
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
