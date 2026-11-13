"""Everything cone-like the lidar saw on the mapping lap, placed through the racer's own solved keyframe poses
(freeze_dump JSON: path = solved positions per keyframe, kf = (scan time, heading)), grouped at 0.3 m. The result
is the layout to score a map against, before any of the mapper's filtering: candidates with sighting counts, their
distance to the path, and a numbered picture.   python3 lidar_gt2.py <car bag> <freeze json> <out prefix>"""
import sys, math, json, numpy as np, cv2
from scipy.spatial import cKDTree
from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import LaserScan
bag, dump, out = sys.argv[1], json.load(open(sys.argv[2])), sys.argv[3]
path = np.array(dump["path"]); kf = np.array(dump["kf"])            # path[0] is the origin pose, path[k] the k-th keyframe
kt, kyaw = kf[:, 0], np.unwrap(kf[:, 1])  # (time, heading[, dx, dy]); kpos = path[1:1 + len(kt)]
t_lo, t_hi = kt[0] - 0.5, kt[-1] + 0.5
r = SequentialReader(); r.open(StorageOptions(uri=bag, storage_id="sqlite3"), ConverterOptions("", ""))
cands = []; nscan = 0
while r.has_next():
    topic, data, ts = r.read_next()
    if topic != "/scan": continue
    m = deserialize_message(data, LaserScan); t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
    if not (t_lo <= t <= t_hi): continue
    nscan += 1
    px, py = np.interp(t, kt, kpos[:, 0]), np.interp(t, kt, kpos[:, 1]); yaw = np.interp(t, kt, kyaw)
    rg = np.asarray(m.ranges, float); ang = m.angle_min + np.arange(len(rg)) * m.angle_increment
    ok = np.isfinite(rg) & (rg > 0.15) & (rg < 6.0); xs, ys = rg * np.cos(ang), rg * np.sin(ang)
    allok = np.isfinite(rg) & (rg > 0.15) & (rg < 8.0); ax, ay = xs[allok], ys[allok]
    groups, cur = [], []
    for i in np.flatnonzero(ok):
        if cur and math.hypot(xs[i] - xs[cur[-1]], ys[i] - ys[cur[-1]]) > 0.12: groups.append(cur); cur = []
        cur.append(i)
    if cur: groups.append(cur)
    c, s = math.cos(yaw), math.sin(yaw)
    for g in groups:
        gx, gy = xs[g], ys[g]
        if math.hypot(gx.max() - gx.min(), gy.max() - gy.min()) > 0.35 or len(g) < 3: continue
        cx, cy = gx.mean(), gy.mean()
        if int(np.sum((ax - cx) ** 2 + (ay - cy) ** 2 < 0.45 ** 2)) - len(g) > 2: continue
        cands.append((px + c * cx - s * cy, py + s * cx + c * cy, math.hypot(cx, cy), t))
cands = np.array(cands); print("scans", nscan, "candidates", len(cands))
# grouping: densest first, within RAD, and never two candidates from the same scan (two clusters in one scan are two
# objects, however close: a gate pair, cones touching at a drain)
RAD = 0.35
tree = cKDTree(cands[:, :2]); used = np.zeros(len(cands), bool); groups = []
order = np.argsort(-np.array([len(tree.query_ball_point(q[:2], RAD)) for q in cands]))
for i in order:
    if used[i]: continue
    nb = sorted((j for j in tree.query_ball_point(cands[i, :2], RAD) if not used[j]), key=lambda j: np.linalg.norm(cands[j, :2] - cands[i, :2]))
    seen_scans, keep = set(), []
    for j in nb:
        if cands[j, 3] in seen_scans: continue
        seen_scans.add(cands[j, 3]); keep.append(j)
    nb = keep; used[nb] = True; pts = cands[nb]
    groups.append({"x": float(pts[:, 0].mean()), "y": float(pts[:, 1].mean()), "n": len(nb), "rmin": float(pts[:, 2].min()), "spread": float(np.linalg.norm(pts[:, :2] - pts[:, :2].mean(0), axis=1).mean())})
# distance to the path and side
P = path[1:]; seg = np.roll(P, -1, axis=0) - P
for g in groups:
    q = np.array([g["x"], g["y"]]); v = q - P; tt = np.clip(np.einsum("ij,ij->i", v, seg) / (np.einsum("ij,ij->i", seg, seg) + 1e-9), 0, 1)
    proj = P + tt[:, None] * seg; d = np.linalg.norm(proj - q, axis=1); k = int(np.argmin(d))
    g["dpath"] = float(d[k]); g["left"] = bool(seg[k, 0] * (q[1] - P[k, 1]) - seg[k, 1] * (q[0] - P[k, 0]) > 0)
groups = [g for g in groups if g["n"] >= 5]; groups.sort(key=lambda g: -g["n"])
json.dump({"groups": groups, "path": P.tolist()}, open(out + ".json", "w"))
G = np.array([[g["x"], g["y"]] for g in groups]); allp = np.vstack([P, G]); lo = allp.min(0) - 1; hi = allp.max(0) + 1; sc = 1000 / max(hi - lo)
Wd, Hd = int((hi[0] - lo[0]) * sc) + 1, int((hi[1] - lo[1]) * sc) + 1; img = np.full((Hd, Wd, 3), 255, np.uint8)
X = lambda q: (int((q[0] - lo[0]) * sc), int(Hd - 1 - (q[1] - lo[1]) * sc))
for a, b in zip(P, P[1:]): cv2.line(img, X(a), X(b), (190, 190, 190), 1)
for i, g in enumerate(groups):
    strong = g["n"] >= 15 and g["dpath"] < 3.5
    cv2.circle(img, X((g["x"], g["y"])), 7 if strong else 4, (30, 30, 200) if strong else (170, 170, 170), -1)
    cv2.putText(img, str(i), (X((g["x"], g["y"]))[0] + 8, X((g["x"], g["y"]))[1] + 4), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 0), 1)
cv2.circle(img, X(P[0]), 6, (0, 0, 0), 2); cv2.imwrite(out + ".png", img)
print("groups >= 5 sightings: %d; >= 15 and within 3.5 m of the path: %d; wrote %s.png/.json" % (len(groups), sum(g["n"] >= 15 and g["dpath"] < 3.5 for g in groups), out))
