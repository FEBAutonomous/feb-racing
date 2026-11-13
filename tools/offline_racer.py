"""Deterministic replay: the racer's callbacks fed straight from the car bag in bag order (no ROS timing, no dropped
frames), from <t0> seconds in, until the map closes. Writes the closed map as JSON for score_gt.py.
    python3 offline_racer.py <car bag> <t0> <out map json> [key=value ...]     (extra racer parameters)
Runs in the box with the starter_kit install sourced."""
import sys, math, json, time
import rclpy
from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import LaserScan, Image, Imu
from nav_msgs.msg import Odometry
from std_msgs.msg import String
CFG = "/home/feb/FEB/feb-racing/starter_kit/feb_cone_racer/config"
bag, t0, out = sys.argv[1], float(sys.argv[2]), sys.argv[3]
extra = dict(a.split("=", 1) for a in sys.argv[4:])
# the same stack of files and detector overrides as race.launch.py detector:=cnn on the car
args = ["--ros-args", "--params-file", CFG + "/racer.yaml", "--params-file", CFG + "/racer.car.yaml", "--params-file", CFG + "/venues/uab.yaml",
        "-p", "use_sim_time:=false", "-p", "detector:=cnn", "-p", "side_guess_range:=0.0", "-p", "side_override_x:=0.0", "-p", "max_range:=5.0",
        "-p", "keyframe_dist:=0.2", "-p", "lidar_cone_range:=3.5"]
for k, v in extra.items():
    args += ["-p", "%s:=%s" % (k, v)]
rclpy.init(args=args)
from feb_cone_racer.racer import Racer
racer = Racer()
r = SequentialReader(); r.open(StorageOptions(uri=bag, storage_id="sqlite3"), ConverterOptions("", ""))
first = None; n = 0; k = math.pi / 180.0; closed = False; t_start = time.time()
while r.has_next():
    topic, data, ts = r.read_next(); ts /= 1e9
    if first is None: first = ts
    if ts - first < t0: continue
    if topic == "/scan":
        racer.on_scan(deserialize_message(data, LaserScan)); n += 1
    elif topic == "/camera/color/image_raw":
        racer.on_image(deserialize_message(data, Image))
    elif topic == "/sensors/imu/raw":
        m = deserialize_message(data, Imu)
        w, a = m.angular_velocity, m.linear_acceleration
        w.x, w.y, w.z = w.x * k, w.y * k, w.z * k; a.x, a.y, a.z = a.x * 9.80665, a.y * 9.80665, a.z * 9.80665
        racer.on_imu(m)
    elif topic == "/odom":
        racer.on_odom(deserialize_message(data, Odometry))
    elif topic == "/feb/state":
        racer.on_state(deserialize_message(data, String))
    if racer.slam.frozen and not closed:
        closed = True
        break
msg = racer.map_message()
json.dump({"closed": closed, "scans": n, "landmarks": [[p.position.x, p.position.y, int(round(p.orientation.w))] for p in msg.poses]}, open(out, "w"))
print("offline replay: %d scans, map %s, %d landmarks, %.0f s" % (n, "closed" if closed else "NOT closed", len(msg.poses), time.time() - t_start))
racer.destroy_node(); rclpy.shutdown()
