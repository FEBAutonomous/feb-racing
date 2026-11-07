#!/usr/bin/env python3
"""Race telemetry probe. Copied into the racing container and run there, because DDS
between containers on the host network discovers but does not deliver.

    python3 probe.py --laps 11 --timeout 300 [--audit 20]

Prints one JSON object per line: {"event": "start"|"lap"|"collision"|"audit"|"end", ...}.
Lap times come from the simulator's own lap timer (last_lap_time on each lap_count
increment). The audit lists which nodes subscribe to the restricted ground-truth topics.
"""
import argparse
import json
import math
import subprocess
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from geometry_msgs.msg import Point
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Float32, Int32

NS = "/autodrive/roboracer_1/"
RESTRICTED = ["ips", "odom", "lap_count", "collision_count", "lap_time", "best_lap_time", "last_lap_time"]
OWN_NODES = ("/autodrive_bridge", "/feb_probe", "/rosbag2_recorder")
QOS = QoSProfile(reliability=QoSReliabilityPolicy.RELIABLE, durability=QoSDurabilityPolicy.VOLATILE,
                 history=QoSHistoryPolicy.KEEP_LAST, depth=10)


def emit(**kw):
    print(json.dumps(kw), flush=True)


def audit():
    """Nodes other than ours subscribed to restricted topics: {topic: [node, ...]}."""
    found = {}
    for topic in RESTRICTED + ["/tf"]:
        name = topic if topic.startswith("/") else NS + topic
        out = subprocess.run(["ros2", "topic", "info", "-v", name], capture_output=True, text=True, timeout=20).stdout
        nodes = []
        for block in out.split("Subscription count")[1:] if "Subscription count" in out else []:
            nodes += [line.split(":", 1)[1].strip() for line in block.splitlines() if line.strip().startswith("Node name:")]
        nodes = [n for n in nodes if "/" + n.lstrip("/") not in OWN_NODES]
        if nodes:
            found[name] = nodes
    return found


def settle_lap_time(node, state, before, wait=2.0):
    """last_lap_time arrives on its own topic and may lag lap_count by a message; wait for it
    to change from the raw value seen at the previous lap. The simulator reports +inf until the
    first lap, which JSON cannot carry: use None."""
    deadline = time.time() + wait
    while time.time() < deadline and (state.get("last_lap_time") in (before, None) or not math.isfinite(state["last_lap_time"])):
        rclpy.spin_once(node, timeout_sec=0.05)
    t = state.get("last_lap_time")
    return t if t is not None and math.isfinite(t) else None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--laps", type=int, default=11, help="stop after this many laps (incl. warm-up)")
    ap.add_argument("--timeout", type=float, default=300.0)
    ap.add_argument("--audit", type=float, default=20.0, help="seconds after start to run the topic audit")
    ap.add_argument("--timing", default="", help="JSON: lines to time from the car's true position (the FS fixed-layout events)")
    args = ap.parse_args()
    timing = json.loads(args.timing) if args.timing else None

    rclpy.init()
    node = Node("feb_probe")
    state = {}
    for name, T in (("lap_count", Int32), ("collision_count", Int32), ("last_lap_time", Float32)):
        node.create_subscription(T, NS + name, (lambda k: lambda m: state.__setitem__(k, m.data))(name), QOS)
    # simulated time from the bridge's /clock: the run's real-time factor is sim elapsed / wall elapsed
    clock = {}
    def on_clock(m):
        sim = m.clock.sec + m.clock.nanosec * 1e-9
        clock.setdefault("first", (sim, time.time()))
        clock["last"] = (sim, time.time())
    node.create_subscription(Clock, "/clock", on_clock, 10)

    # the line timer: every true-position sample is checked against each timing line; a crossing
    # is interpolated between the two samples around it, in simulated time. Also a stop detector
    # for the events that end with the car at rest.
    lt = {"last": None, "xs": [], "crossed": [], "done_at": None, "rest_since": None, "stopped": False}
    def sim_now():
        return clock["last"][0] if "last" in clock else None
    def on_ips(m):
        if timing is None:
            return
        ts = sim_now()
        if ts is None:
            return
        p = (m.x, m.y, ts)
        last = lt["last"]
        lt["last"] = p
        lt["xs"].append(m.x)
        if last is None:
            emit(event="position", x=round(m.x, 3), y=round(m.y, 3))
            return
        for name, (a, b) in timing["lines"].items():
            # segment last->p against the line a->b
            dx, dy = p[0] - last[0], p[1] - last[1]
            ex, ey = b[0] - a[0], b[1] - a[1]
            den = dx * ey - dy * ex
            if abs(den) < 1e-12:
                continue
            u = ((a[0] - last[0]) * ey - (a[1] - last[1]) * ex) / den     # along the car's step
            v = ((a[0] - last[0]) * dy - (a[1] - last[1]) * dx) / den     # along the line
            if 0.0 <= u <= 1.0 and 0.0 <= v <= 1.0:
                t_cross = last[2] + u * (p[2] - last[2])
                direction = 1 if (dx * -ey + dy * ex) > 0 else -1          # +1: crossing from the line's right-hand side to its left
                emit(event="line", name=name, t=round(time.time() - t0, 2), t_sim=round(t_cross, 4), dir=direction,
                     side=round(sum(lt["xs"]) / len(lt["xs"]), 3), x=round(m.x, 3), y=round(m.y, 3))
                lt["crossed"].append(name)
                lt["xs"] = []
                if name in ("finish", "exit"):
                    lt["done_at"] = ts
        # at rest after the finish: the programme's end
        if lt["done_at"] is not None and not lt["stopped"]:
            dt = p[2] - last[2]
            speed = math.hypot(p[0] - last[0], p[1] - last[1]) / dt if dt > 0 else 0.0
            if speed < 0.1:
                lt["rest_since"] = lt["rest_since"] if lt["rest_since"] is not None else ts
                if ts - lt["rest_since"] >= 1.0:
                    lt["stopped"] = True
                    emit(event="stopped", t=round(time.time() - t0, 2), t_sim=round(ts, 4), x=round(m.x, 3), y=round(m.y, 3))
            else:
                lt["rest_since"] = None
    node.create_subscription(Point, NS + "ips", on_ips, QOS)

    t0 = time.time()
    base = None
    seen_lap = seen_col = None
    audited = False
    lap_times = []
    raw_lap_time = None
    while time.time() - t0 < args.timeout:
        rclpy.spin_once(node, timeout_sec=0.1)
        if "lap_count" not in state or "collision_count" not in state:
            continue
        now = round(time.time() - t0, 2)
        if base is None:
            base = (state["lap_count"], state["collision_count"])
            seen_lap, seen_col = base
            emit(event="start", t=now, lap_count=base[0], collision_count=base[1])
        if state["lap_count"] != seen_lap:
            seen_lap = state["lap_count"]
            raw_lap_time = settle_lap_time(node, state, raw_lap_time)
            lap_time = round(raw_lap_time, 4) if raw_lap_time is not None else None
            lap_times.append(lap_time)
            emit(event="lap", t=now, lap=seen_lap - base[0], lap_time=lap_time)
            if seen_lap - base[0] >= args.laps and timing is None:
                break
        if state["collision_count"] != seen_col:
            seen_col = state["collision_count"]
            emit(event="collision", t=now, lap=seen_lap - base[0], collisions=seen_col - base[1])
        if not audited and now >= args.audit:
            audited = True
            emit(event="audit", t=now, restricted=audit())
        if timing is not None and lt["stopped"]:
            break
        if timing is not None and lt["done_at"] is not None and sim_now() is not None and sim_now() - lt["done_at"] > 60.0:
            break                                   # finished but never stopped: the result is a DNF either way
    if base is None:
        emit(event="end", t=round(time.time() - t0, 2), error="no telemetry")
    else:
        rtf = None
        if "last" in clock and clock["last"][1] - clock["first"][1] > 1.0:
            rtf = round((clock["last"][0] - clock["first"][0]) / (clock["last"][1] - clock["first"][1]), 3)
        emit(event="end", t=round(time.time() - t0, 2), laps=seen_lap - base[0],
             collisions=seen_col - base[1], lap_times=lap_times, real_time_factor=rtf)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
