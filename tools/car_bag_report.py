#!/usr/bin/env python3
"""A health read of a bag recorded on the car: pad dropouts and button flicker, the supervisor's
state changes, IMU units and bias, heading drift at rest, the turn total against the kinematic
bicycle, and what the racer commanded against what the car did.

Run inside the box (rosbag2 and the message packages are there):
    ./feb shell -c "python3 ~/FEB/feb-racing/tools/car_bag_report.py ~/FEB/bags/<bag>"
"""
import math
import sys

import numpy as np
from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Imu, Joy
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from ackermann_msgs.msg import AckermannDriveStamped

WHEELBASE = 0.33     # m, the Traxxas chassis


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def main():
    if len(sys.argv) < 2:
        sys.exit(__doc__)
    r = SequentialReader()
    r.open(StorageOptions(uri=sys.argv[1], storage_id="sqlite3"), ConverterOptions("", ""))
    imu, imu_contract, odom, cmd, drive, joy, states, status = [], 0, [], [], [], [], [], []
    while r.has_next():
        topic, data, t = r.read_next()
        t /= 1e9
        if topic == "/sensors/imu/raw":
            m = deserialize_message(data, Imu)
            imu.append((t, yaw_of(m.orientation), m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z,
                        m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z))
        elif topic == "/imu":
            imu_contract += 1
        elif topic == "/odom":
            odom.append((t, deserialize_message(data, Odometry).twist.twist.linear.x))
        elif topic == "/ackermann_cmd":
            m = deserialize_message(data, AckermannDriveStamped)
            cmd.append((t, m.drive.speed, m.drive.steering_angle))
        elif topic == "/drive":
            m = deserialize_message(data, AckermannDriveStamped)
            drive.append((t, m.drive.speed, m.drive.steering_angle))
        elif topic == "/joy":
            joy.append((t, tuple(deserialize_message(data, Joy).buttons)))
        elif topic == "/feb/state":
            states.append((t, deserialize_message(data, String).data))
        elif topic == "/feb/status":
            status.append((t, deserialize_message(data, String).data))
    if not imu or not odom:
        sys.exit("the bag needs /sensors/imu/raw and /odom")
    imu = np.array(imu)
    odom = np.array(odom)
    t0 = imu[0, 0]
    T = lambda t: t - t0
    print("bag: %.0f s" % (imu[-1, 0] - t0))

    # the pad
    if joy:
        jt = np.array([t for t, _ in joy])
        gaps = np.diff(jt)
        print("\npad: %.0f Hz, longest gap %.2f s (the supervisor releases at 0.5 s), %d gaps over 0.5 s"
              % (len(jt) / (jt[-1] - jt[0]), gaps.max(), int((gaps > 0.5).sum())))
        auto = [(a, b) for (a, s), (b, _) in zip(states, states[1:] + [(imu[-1, 0], "")]) if s.startswith("AUTONOMOUS")]
        if auto:
            n = max(len(b) for _, b in joy)
            for i in range(n):
                drops = 0
                for a, b in auto:
                    seq = [bt[i] for t, bt in joy if a <= t <= b and len(bt) > i]
                    drops += sum(1 for p, q in zip(seq, seq[1:]) if p and not q)
                if drops:
                    print("  button %d read unpressed %d times inside AUTONOMOUS (%.0f s held in total)" % (i, drops, sum(b - a for a, b in auto)))
    if states:
        print("\nsupervisor:")
        last = None
        for t, s in states:
            if s != last:
                print("  %6.1f  %s" % (T(t), s))
                last = s

    # the IMU: units, bias, drift
    still = imu[imu[:, 0] < t0 + 10]
    g = np.linalg.norm(still[:, 5:8].mean(0))
    unit = "g (the VESC driver's raw units: the relay must convert)" if 0.8 < g < 1.2 else "m/s^2" if 8 < g < 11.5 else "unknown (|a| = %.2f)" % g
    print("\nimu: %d raw at %.0f Hz, %d on /imu (%.1fx: 1.0 = one relay). Acceleration at rest %.2f -> %s"
          % (len(imu), len(imu) / (imu[-1, 0] - t0), imu_contract, imu_contract / len(imu), g, unit))
    print("  gyro at rest x %+.3f y %+.3f z %+.3f (bias)" % tuple(still[:, 2:5].mean(0)))
    yaw = np.unwrap(still[:, 1])
    print("  orientation yaw drift at rest: %+.2f deg/s" % (math.degrees(yaw[-1] - yaw[0]) / max(still[-1, 0] - still[0, 0], 1e-3)))
    if cmd:
        cmd = np.array(cmd)
        ts = imu[:, 0]
        v = np.interp(ts, odom[:, 0], odom[:, 1])
        steer = np.interp(ts, cmd[:, 0], cmd[:, 2])
        expect = v * np.tan(steer) / WHEELBASE
        moving = np.abs(v) > 0.3
        if moving.sum() > 100:
            for i, name in ((2, "x"), (3, "y"), (4, "z")):
                gg = imu[moving, i] - still[:, i].mean()
                c = np.corrcoef(gg, expect[moving])[0, 1]
                k = np.polyfit(expect[moving], gg, 1)[0]
                print("  gyro %s vs kinematic yaw rate: correlation %+.2f, slope %.2f%s" % (name, c, k, "  <- the yaw axis; a slope near 57 means degrees per second" if abs(c) > 0.6 else ""))
            dt = np.diff(ts, prepend=ts[0])
            yv = np.unwrap(imu[:, 1])
            print("  turn over the bag: orientation %+.0f deg, kinematic bicycle %+.0f deg" % (math.degrees(yv[-1] - yv[0]), math.degrees(np.sum(expect * dt))))

    # speeds
    vs = odom[:, 1]
    print("\nodom: max %.2f m/s, mean while moving %.2f" % (vs.max(), vs[vs > 0.05].mean() if (vs > 0.05).any() else 0.0))
    if drive:
        d = np.array(drive)
        print("racer /drive: %d commands, speed max %.2f m/s, steer %.2f..%.2f rad" % (len(d), d[:, 1].max(), d[:, 2].min(), d[:, 2].max()))
    if status:
        phases = []
        for t, s in status:
            ph = s.split("|")[0].strip()
            if not phases or phases[-1][1] != ph:
                phases.append((T(t), ph))
        print("racer phases:")
        for t, ph in phases[:12]:
            print("  %6.1f  %s" % (t, ph))


if __name__ == "__main__":
    main()
