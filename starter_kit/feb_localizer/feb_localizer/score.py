"""Score a Localizer offline against a recorded lap with ground truth, no simulator needed:

    ros2 run feb_localizer score <bag folder>            # inside ./feb shell

Replays the bag's /odom through predict() and its /scan through update_mapping() for the first
lap (building the grid as the driver would) and update_racing() for the second, and at every
scan compares the Localizer's pose with the simulator's true position, moved into the car's
start frame. Prints the median and end-of-lap error per phase and writes <bag>_score.png with
the true path (grey) and the estimate (mapping lap blue, racing lap orange).
"""
import math
import pathlib
import sys

import numpy as np
from rclpy.serialization import deserialize_message
from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
from rosidl_runtime_py.utilities import get_message

from .grid import Grid
from .localizer import Localizer

RANGE_CAP = 8.0


def read(bag):
    r = SequentialReader()
    r.open(StorageOptions(uri=str(bag), storage_id="sqlite3"), ConverterOptions("", ""))
    types = {t.name: t.type for t in r.get_all_topics_and_types()}
    msgs = []
    while r.has_next():
        topic, data, ts = r.read_next()
        if topic in ("/scan", "/odom", "/autodrive/roboracer_1/ips", "/autodrive/roboracer_1/imu", "/autodrive/roboracer_1/lap_count"):
            msgs.append((ts * 1e-9, topic, deserialize_message(data, get_message(types[topic]))))
    return msgs


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def main():
    if len(sys.argv) < 2:
        sys.exit(__doc__)
    bag = pathlib.Path(sys.argv[1])
    msgs = read(bag)
    loc, grid = Localizer(), Grid()
    truth_yaw, origin, lap0 = None, None, None
    last_odom = None
    err = {"mapping": [], "racing": []}
    track = {"truth": [], "mapping": [], "racing": []}
    # the bag may start mid-lap: wait for the next start-line crossing, then score one full
    # mapping lap and one full racing lap from there, the car's start frame set at that crossing
    phase = "waiting"
    for t, topic, m in msgs:
        if topic.endswith("/imu"):
            truth_yaw = yaw_of(m.orientation)
        elif topic.endswith("/lap_count"):
            if lap0 is None:
                lap0 = m.data
            lap = m.data - lap0
            if lap >= 3:
                break
            if lap == 1 and phase == "waiting":
                phase, last_odom = "mapping", None
            elif lap == 2 and phase == "mapping":
                phase = "racing"
        elif phase == "waiting":
            continue
        elif topic.endswith("/ips"):
            if truth_yaw is None:
                continue
            if origin is None:
                origin = (m.x, m.y, truth_yaw)
            ox, oy, oyaw = origin
            dx, dy = m.x - ox, m.y - oy
            c, s = math.cos(-oyaw), math.sin(-oyaw)
            truth = (dx * c - dy * s, dx * s + dy * c)
            track["truth"].append(truth)
            est = (loc.x, loc.y)
            err[phase].append(math.hypot(est[0] - truth[0], est[1] - truth[1]))
            track[phase].append(est)
        elif topic == "/odom":
            if last_odom is not None:
                loc.predict(m.twist.twist.linear.x, m.twist.twist.angular.z, min(max(t - last_odom, 0.0), 0.2))
            last_odom = t
        elif topic == "/scan":
            rg = np.asarray(m.ranges, float)
            rg[~np.isfinite(rg)] = m.range_max
            ang = m.angle_min + np.arange(len(rg)) * m.angle_increment
            good = (rg > m.range_min) & (rg < min(m.range_max, RANGE_CAP))
            pts = np.stack([rg[good] * np.cos(ang[good]), rg[good] * np.sin(ang[good])], axis=1)
            if phase == "mapping":
                loc.update_mapping(pts, grid)
                grid.insert(pts, (loc.x, loc.y, loc.yaw))
            else:
                loc.update_racing(pts, grid)
    for name in ("mapping", "racing"):
        e = err[name]
        if e:
            print("%-8s lap  median error %.2f m   end of lap %.2f m   (%d samples)" % (name, float(np.median(e)), e[-1], len(e)))
        else:
            print("%-8s lap  no samples (the bag has no lap of that kind)" % name)
    try:
        from PIL import Image, ImageDraw
        pts = np.array(track["truth"] + track["mapping"] + track["racing"])
        lo, hi = pts.min(0), pts.max(0)
        span = max(hi - lo) + 2.0
        S = 700 / span
        img = Image.new("RGB", (720, 720), "white")
        d = ImageDraw.Draw(img)
        def px(p): return (10 + (p[0] - lo[0] + 1.0) * S, 710 - (p[1] - lo[1] + 1.0) * S)
        for name, colour, w in (("truth", (170, 170, 170), 5), ("mapping", (40, 100, 240), 2), ("racing", (240, 130, 30), 2)):
            seg = track[name]
            for a, b in zip(seg, seg[1:]):
                d.line([px(a), px(b)], fill=colour, width=w)
        d.text((12, 8), "grey: truth   blue: mapping lap   orange: racing lap", fill=(0, 0, 0))
        out = bag.with_name(bag.name + "_score.png")
        img.save(out)
        print("picture:", out)
    except ImportError:
        pass


if __name__ == "__main__":
    main()
