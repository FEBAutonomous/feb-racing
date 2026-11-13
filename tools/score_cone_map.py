"""Per-cone score of a replay's final map against the lidar candidate layout (lidar_gt2.py JSON). Ground truth =
candidates with at least MIN_N sightings within MAX_D of the path; a landmark within 0.5 m of a truth cone is that cone
(one-to-one, nearest first). Colour truth: the side of the path (inner ring yellow, outer blue), orange anywhere counts
as right on the outer ring's start gate. usage: score_gt.py <replay bag> <gt json> [min_n] [max_d]"""
import sys, json, numpy as np
from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
from rclpy.serialization import deserialize_message
from geometry_msgs.msg import PoseArray
bag, gt = sys.argv[1], json.load(open(sys.argv[2])); MIN_N = int(sys.argv[3]) if len(sys.argv) > 3 else 10; MAX_D = float(sys.argv[4]) if len(sys.argv) > 4 else 3.0
if bag.endswith(".json"):            # a map written by offline_racer.py
    M = [tuple(l) for l in json.load(open(bag))["landmarks"]]
else:
    r = SequentialReader(); r.open(StorageOptions(uri=bag, storage_id="sqlite3"), ConverterOptions("", ""))
    last = None
    while r.has_next():
        t, d, _ = r.read_next()
        if t == "/feb/map": last = d
    M = [(p.position.x, p.position.y, int(round(p.orientation.w))) for p in deserialize_message(last, PoseArray).poses]
G = [g for g in gt["groups"] if g["n"] >= MIN_N and g["dpath"] <= MAX_D]
# which hand is the inner ring: the side with the smaller mean distance to the path among the truth cones
left = np.array([g["left"] for g in G]); dp = np.array([g["dpath"] for g in G])
inner_left = dp[left].mean() < dp[~left].mean() if left.any() and (~left).any() else True
truth_col = [2 if (g["left"] == inner_left) else 1 for g in G]
L = np.array([[x, y] for x, y, c in M]); C = [c for x, y, c in M]; P = np.array([[g["x"], g["y"]] for g in G])
# the map and the truth may come from different runs (frames share the start pose but not the drift): a rigid fit of
# the map onto the truth, nearest neighbours within 0.8 m, a few rounds
def refine(L):
    for _ in range(10):
        d = np.linalg.norm(L[:, None] - P[None], axis=2); j = d.argmin(1); ok = d[np.arange(len(L)), j] < 0.8
        if ok.sum() < 6: break
        A, B = L[ok], P[j[ok]]; ca, cb = A.mean(0), B.mean(0)
        H = (A - ca).T @ (B - cb); U, _, Vt = np.linalg.svd(H); R = Vt.T @ U.T
        if np.linalg.det(R) < 0: Vt[1] *= -1; R = Vt.T @ U.T
        L = (L - ca) @ R.T + cb
    d = np.linalg.norm(L[:, None] - P[None], axis=2)
    return L, int((d.min(1) < 0.5).sum())
# the map's frame may differ from the truth's (another lap, another start heading): coarse search over rotations
# about the centroid, then the usual refinement; the alignment with the most cones within 0.5 m wins
best = None
for deg in range(0, 360, 10):
    a = np.radians(deg); R0 = np.array([[np.cos(a), -np.sin(a)], [np.sin(a), np.cos(a)]])
    L0 = (L - L.mean(0)) @ R0.T + P.mean(0)
    cand, n = refine(L0)
    if best is None or n > best[1]: best = (cand, n)
L = best[0]
pairs = sorted(((np.linalg.norm(L[i] - P[j]), i, j) for i in range(len(L)) for j in range(len(P))))
used_l, used_g, match = set(), set(), {}
for d, i, j in pairs:
    if d > 0.5: break
    if i in used_l or j in used_g: continue
    used_l.add(i); used_g.add(j); match[j] = i
found = len(match); missing = [j for j in range(len(P)) if j not in match]; extra = [i for i in range(len(L)) if i not in used_l]
right = sum(1 for j, i in match.items() if C[i] == truth_col[j] or C[i] == 3); wrong = [(j, i) for j, i in match.items() if C[i] != truth_col[j] and C[i] != 3]
n_in = sum(1 for c in truth_col if c == 2); n_out = len(G) - n_in
if len(sys.argv) > 5:                   # overlay picture: truth as rings (inner ring thin, outer thick), the map as filled dots
    import cv2
    path = np.array(gt["path"]); allp = np.vstack([P, L, path]); lo = allp.min(0) - 1; hi = allp.max(0) + 1; sc = 1000 / max(hi - lo)
    Wd, Hd = int((hi[0] - lo[0]) * sc) + 1, int((hi[1] - lo[1]) * sc) + 1; img = np.full((Hd, Wd, 3), 255, np.uint8)
    X = lambda q: (int((q[0] - lo[0]) * sc), int(Hd - 1 - (q[1] - lo[1]) * sc))
    for a, b in zip(path, path[1:]): cv2.line(img, X(a), X(b), (200, 200, 200), 1)
    COL = {0: (150, 150, 150), 1: (255, 120, 40), 2: (0, 200, 255), 3: (40, 100, 255)}
    for j, g in enumerate(G):
        cv2.circle(img, X(P[j]), 11, (0, 0, 0) if j in match else (0, 0, 230), 2 if truth_col[j] == 1 else 1)
        cv2.putText(img, str(j), (X(P[j])[0] + 12, X(P[j])[1] - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (90, 90, 90), 1)
    for i, (q, c) in enumerate(zip(L, C)):
        cv2.circle(img, X(q), 6, COL[c], -1)
        if i in extra: cv2.putText(img, "x%d" % i, (X(q)[0] + 7, X(q)[1] + 12), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0, 0, 200), 1)
    cv2.imwrite(sys.argv[5], img)
print("%s vs lidar truth (%d cones: %d inner, %d outer): found %d, missing %d, extra %d | colour right %d of %d found, wrong %d | missing truth ids %s | wrong (truth id, map id) %s | extras map ids %s"
      % (bag.rstrip("/").split("/")[-1], len(G), n_in, n_out, found, len(missing), len(extra), right, found, len(wrong), missing, wrong, extra))
