"""Camera calibration for the cone racer, from a bag: the numbers in racer.yaml that are about
this car's camera, fitted from what the lidar and the camera saw together.

    ros2 run feb_cone_racer calibrate_camera <bag> [--track track.json] [--params racer.yaml]

What it fits, from lidar cones and camera blobs that clearly belong together (one blob near
the cone's expected column, no other cone on that bearing):
  camera geometry   camera_hfov_deg, camera_ahead, camera_lateral, camera_col_bias (where a
                    point in front of the lidar lands in the image), camera_lag_s (how far the
                    image lags the scan, from the error against yaw rate)
  image band        band_top, band_bottom, band_floor (the rows cones live in)
  brightness gate   blob_min_value (a hue blob darker than this is a speck, not a cone)
With --track (the true cone positions, and the bag's IPS ground truth as on the simulator)
it also fits the HSV bounds per colour from the pixels of cones of known colour, and reports
the colour accuracy by range. On the real car there is no truth: the geometry, band and
brightness fits still work, the HSV bounds are reported as they stand.

The output is a yaml snippet to paste into config/racer.yaml, with the fit quality beside
each number so a bad bag (too few cones, the car standing still) is obvious.
"""
import argparse
import json
import math
import sys

import cv2
import numpy as np

from .perception import BLUE, ORANGE, YELLOW, Perception
from .racer import DEFAULTS


def load_params(path):
    p = dict(DEFAULTS)
    if path:
        import yaml
        doc = yaml.safe_load(open(path))
        node = next(iter(doc.values()))
        p.update(node.get("ros__parameters", node))
    return p


def read_bag(path, per, truth, image_hsv):
    """One row per lidar cone that the camera could see: car-frame position, the blob that was
    nearest its column (if any), the truth colour (if known), yaw rate, and the image's HSV."""
    from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
    from rclpy.serialization import deserialize_message
    from geometry_msgs.msg import Point
    from sensor_msgs.msg import Image, Imu, LaserScan
    r = SequentialReader()
    r.open(StorageOptions(uri=path, storage_id="sqlite3"), ConverterOptions("", ""))
    yaw = ips = None
    yaw_rate = 0.0
    hsv = None
    rows = []
    scans = 0
    # the simulator's bags name the topics /autodrive/.../lidar, /front_camera, /imu; the car's are the
    # contract's /scan, /camera/color/image_raw, /imu
    is_scan = lambda t: t.endswith("/lidar") or t == "/scan"
    is_image = lambda t: t.endswith("/front_camera") or t == "/camera/color/image_raw"
    while r.has_next():
        topic, data, _ = r.read_next()
        if topic == "/imu" or topic.endswith("/imu") and not topic.startswith("/sensors"):
            m = deserialize_message(data, Imu)
            q = m.orientation
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
            yaw_rate = m.angular_velocity.z
        elif topic.endswith("/ips"):
            m = deserialize_message(data, Point)
            ips = np.array([m.x, m.y])
        elif is_image(topic):
            m = deserialize_message(data, Image)
            per.on_image(m)
            if image_hsv:
                img = np.frombuffer(m.data, dtype=np.uint8).reshape(m.height, m.width, 3)
                hsv = cv2.cvtColor(np.ascontiguousarray(img[:, :, ::-1] if m.encoding == "rgb8" else img), cv2.COLOR_BGR2HSV)
        elif is_scan(topic) and yaw is not None and per.blobs and (truth is None or ips is not None):
            scans += 1
            clusters = per.clusters(deserialize_message(data, LaserScan))
            cols = [per.pixel_column(x, y, yaw_rate) for x, y in clusters]
            for i, (x, y) in enumerate(clusters):
                if math.isnan(cols[i]):
                    continue
                lonely = all(j == i or math.isnan(cols[j]) or abs(cols[j] - cols[i]) > 0.065 * per.image_width for j in range(len(clusters)))
                near = [b for b in per.blobs if abs(b[1] - cols[i]) < 0.06 * per.image_width]
                blob = min(near, key=lambda b: abs(b[1] - cols[i])) if near else None
                tc = None
                if truth is not None:
                    c, s = math.cos(yaw), math.sin(yaw)
                    w = ips + 0.32 * np.array([c, s]) + np.array([c * x - s * y, s * x + c * y])
                    d = np.linalg.norm(truth[0] - w, axis=1)
                    k = int(np.argmin(d))
                    tc = int(truth[1][k]) if d[k] < 0.4 else None
                rows.append(dict(x=x, y=y, col=cols[i], lonely=lonely, blob=blob, truth=tc, yaw_rate=yaw_rate, hsv=hsv,
                                 blobs=[(b[0], b[1]) for b in per.blobs]))
    # the VESC's driver reports the gyro in degrees per second; the racer's relay converts, an old bag may not have
    rates = np.array([abs(r["yaw_rate"]) for r in rows]) if rows else np.zeros(1)
    if np.percentile(rates, 95) > 8.0:
        print("note: yaw rates look like degrees per second (95th percentile %.0f); converting to rad/s" % np.percentile(rates, 95))
        for r in rows:
            r["yaw_rate"] = math.radians(r["yaw_rate"])
    return rows, scans


def fit_geometry(rows, W, p, truth=False):
    """Column model u = W/2 - fx * (y - dy) / (x - x0) + bias, then the lag from the error's
    slope against yaw rate. Uses cones with one clear blob and nothing else on the bearing."""
    # pairing a lidar cone with its blob needs the geometry, and the geometry comes from the pairs: so the
    # pairs are made from the current model with a wide tolerance (a cone spacing), the model refitted, and
    # the tolerance tightened, three rounds. Cones with no other cone on their bearing only.
    cand = [r for r in rows if r["lonely"] and r["blobs"] and 1.0 < math.hypot(r["x"], r["y"]) < 6.0 and abs(r["yaw_rate"]) < 1.5]
    if len(cand) < 200:
        return None, "only %d lidar cones with a clear bearing and an image (need a few hundred: drive past more cones)" % len(cand)
    fx0 = (W / 2) / math.tan(math.radians(p["camera_hfov_deg"]) / 2)
    model = dict(fx=fx0, x0=p["camera_ahead"], dy=p["camera_lateral"], bias=p["camera_col_bias"], lag=p["camera_lag_s"])
    def predict(m, x, y, yr):
        return W / 2 - m["fx"] * (y - m["dy"]) / max(x - m["x0"], 0.2) + m["bias"] - m["fx"] * m["lag"] * yr
    A = None
    for tol in (0.10, 0.06, 0.04):
        S = []
        for r in cand:
            u0 = predict(model, r["x"], r["y"], r["yaw_rate"])
            near = [b for b in r["blobs"] if abs(b[1] - u0) < tol * W and (r["truth"] == b[0] if truth else True)]
            if len(near) == 1:
                S.append((r["x"], r["y"], near[0][1], r["yaw_rate"]))
        if len(S) < 150:
            return None, "only %d cone-blob pairs within %.0f%% of the width (the camera geometry may be far off; check the mount)" % (len(S), tol * 100)
        A = np.array(S)
        keep = np.ones(len(A), dtype=bool)
        model, sd = _solve(A, keep, W, model)
    x0, fx, dy, bias, lag = model["x0"], model["fx"], model["dy"], model["bias"], model["lag"]
    hfov = 2 * math.degrees(math.atan((W / 2) / fx))
    if not 35 < hfov < 95 or abs(dy) > 0.4 or abs(bias) > 120:
        return None, "the fit ran away (hfov %.0f, lateral %.2f, bias %.0f): the pairs are not trustworthy, record a slower lap with the cones well apart" % (hfov, dy, bias)
    return dict(camera_hfov_deg=round(hfov, 1), camera_ahead=round(x0, 2), camera_lateral=round(dy, 3),
                camera_col_bias=round(bias, 1), camera_lag_s=round(lag, 3)), \
        "%d pairs, residual %.1f px (hfov %.1f, ahead %.2f, lateral %.3f, bias %.1f px, lag %.3f s)" % (len(A), sd, hfov, x0, dy, bias, lag)


def _solve(A, keep, W, start):
    """One fit of the column model to the pairs in A: x0 scanned, the rest linear least squares, the
    pairs that do not fit dropped and the fit repeated. Returns (model, residual sd)."""
    best_model, sd = dict(start), 0.0
    # u - W/2 = -fx * (y - dy) / (x - x0) + bias - fx * lag * yaw_rate: linear in fx, fx*dy,
    # bias and fx*lag once x0 is fixed, so x0 is scanned and the rest solved by least squares;
    # then the pairs that do not fit (a wrong blob) are dropped and it is done again
    for _ in range(3):
        x, y, u, yr = A[keep].T
        best = None
        for x0 in np.arange(-1.0, 0.61, 0.02):
            cx = x - x0
            m = cx > 0.2
            if m.sum() < 100:
                continue
            M = np.c_[y[m] / cx[m], 1.0 / cx[m], np.ones(int(m.sum())), yr[m]]
            coef, *_ = np.linalg.lstsq(M, u[m] - W / 2, rcond=None)
            err = u[m] - W / 2 - M @ coef
            if best is None or err.std() < best[0]:
                best = (err.std(), x0, coef)
        sd, x0, coef = best
        fx, bias = -coef[0], coef[2]
        dy = coef[1] / fx if fx else 0.0
        lag = -coef[3] / fx if fx else 0.0
        cx = A[:, 0] - x0
        pred = W / 2 - fx * (A[:, 1] - dy) / np.maximum(cx, 0.2) + bias - fx * lag * A[:, 3]
        e_all = A[:, 2] - pred
        keep = (cx > 0.2) & (np.abs(e_all - np.median(e_all[keep])) < 3.0 * max(sd, 1.0))
        best_model = dict(fx=fx, x0=x0, dy=dy, bias=bias, lag=lag)
    return best_model, sd


def fit_band(rows, H):
    tops = np.array([r["blob"][2] for r in rows if r["blob"] is not None and r["lonely"]])
    bottoms = np.array([r["blob"][3] for r in rows if r["blob"] is not None and r["lonely"]])
    if len(tops) < 100:
        return None, "too few blobs"
    return dict(band_top=round(max(np.percentile(tops, 1) / H - 0.03, 0.0), 2),
                band_bottom=round(min(np.percentile(bottoms, 99) / H + 0.03, 1.0), 2),
                band_floor=round(np.percentile(bottoms, 1) / H, 2)), "%d blobs; cone tops at rows %.0f-%.0f, bottoms %.0f-%.0f of %d" % (
        len(tops), np.percentile(tops, 1), np.percentile(tops, 99), np.percentile(bottoms, 1), np.percentile(bottoms, 99), H)


def blob_value(r):
    """Mean HSV value of the blob's pixels (its column +-3, its rows)."""
    b, hsv = r["blob"], r["hsv"]
    u0, u1 = int(max(b[1] - 3, 0)), int(min(b[1] + 4, hsv.shape[1]))
    v0, v1 = int(b[2]), int(max(b[3], b[2] + 1))
    return float(np.mean(hsv[v0:v1, u0:u1, 2]))


def fit_brightness(rows, truth):
    M = [r for r in rows if r["blob"] is not None and r["hsv"] is not None and 2.0 < math.hypot(r["x"], r["y"]) < 4.5]
    if len(M) < 100:
        return None, "too few blobs"
    vals = np.array([blob_value(r) for r in M])
    if not truth:
        return dict(blob_min_value=int(np.percentile(vals, 5))), "%d blobs, value 5%%/50%%/95%%: %.0f / %.0f / %.0f (no truth: the 5th percentile keeps nearly every blob; raise it if far cones get the wrong colour)" % (
            len(M), *np.percentile(vals, [5, 50, 95]))
    right = np.array([blob_value(r) for r in M if r["truth"] is not None and r["truth"] == r["blob"][0]])
    wrong = np.array([blob_value(r) for r in M if r["truth"] is not None and r["truth"] not in (None, r["blob"][0]) and r["truth"] != ORANGE])
    if len(wrong) < 20:
        return dict(blob_min_value=int(np.percentile(right, 3))), "%d right blobs, too few wrong ones to weigh against" % len(right)
    best = max(range(5, 120, 5), key=lambda t: np.mean(right >= t) - np.mean(wrong >= t))
    return dict(blob_min_value=best), "right blobs %d (value median %.0f), wrong %d (median %.0f); at %d it keeps %.0f%% of right and %.0f%% of wrong" % (
        len(right), np.median(right), len(wrong), np.median(wrong), best, 100 * np.mean(right >= best), 100 * np.mean(wrong >= best))


def fit_hsv(rows):
    """HSV bounds per colour from the pixels of blobs on cones of known colour."""
    out, notes = {}, []
    for name, c in (("hsv_blue", BLUE), ("hsv_yellow", YELLOW), ("hsv_orange", ORANGE)):
        px = []
        for r in rows:
            b = r["blob"]
            if b is None or r["hsv"] is None or r["truth"] != c or b[0] != c or not r["lonely"]:
                continue
            u0, u1 = int(max(b[1] - 2, 0)), int(min(b[1] + 3, r["hsv"].shape[1]))
            win = r["hsv"][int(b[2]):int(max(b[3], b[2] + 1)), u0:u1].reshape(-1, 3)
            px.append(win[win[:, 1] > 80])           # the cone's pixels are the saturated ones; the rest is ground
        if not px:
            notes.append("%s: no cones of this colour seen" % name)
            continue
        P = np.vstack(px)
        # the hue range: the 90% of pixels around the commonest hue, widened a little; the
        # saturation and value floors from the darkest, palest cone pixels
        hist = np.bincount(P[:, 0], minlength=180)
        mode = int(np.argmax(hist))
        lo_h, hi_h = mode, mode
        while hist[lo_h:hi_h + 1].sum() < 0.9 * len(P) and (lo_h > 0 or hi_h < 179):
            if lo_h > 0 and (hi_h >= 179 or hist[lo_h - 1] >= hist[hi_h + 1]):
                lo_h -= 1
            else:
                hi_h += 1
        lo, hi = np.percentile(P, 2, axis=0), np.percentile(P, 98, axis=0)
        lo[0], hi[0] = max(lo_h - 5, 0), min(hi_h + 5, 179)
        out[name] = [int(lo[0]), int(lo[1]), int(max(lo[2] - 5, 0)), int(hi[0]), 255, 255]
        notes.append("%s: %d pixels, hue %d-%d, saturation from %d, value from %d" % (name, len(P), lo[0], hi[0], lo[1], lo[2]))
    return out, "; ".join(notes)


def colour_report(rows):
    bands = [(0, 2), (2, 4), (4, 6.5)]
    lines = []
    for a, b in bands:
        R = [r for r in rows if r["blob"] is not None and r["truth"] not in (None, ORANGE) and a <= math.hypot(r["x"], r["y"]) < b]
        if R:
            wrong = sum(1 for r in R if r["blob"][0] != r["truth"])
            lines.append("  %.0f-%.1f m: %d readings, %.1f%% wrong" % (a, b, len(R), 100.0 * wrong / len(R)))
    return "\n".join(lines)


def main(argv=None):
    ap = argparse.ArgumentParser(description="fit the racer's camera parameters from a bag")
    ap.add_argument("bag")
    ap.add_argument("--track", help="track.json with the true cones (simulator bags with IPS): also fits the HSV bounds and reports colour accuracy")
    ap.add_argument("--params", help="racer.yaml to start from (default: the built-in defaults)")
    args = ap.parse_args(argv)
    p = load_params(args.params)
    per = Perception(p)
    truth = None
    if args.track:
        t = json.load(open(args.track))
        truth = (np.array([[c["x"], c["y"]] for c in t["cones"]]),
                 np.array([{"blue": BLUE, "yellow": YELLOW}.get(c["color"], ORANGE) for c in t["cones"]]))
    rows, scans = read_bag(args.bag, per, truth, image_hsv=True)
    if not rows:
        sys.exit("nothing usable in the bag: it needs the scan, the camera image and /imu (and ips with --track)")
    W = per.image_width
    H = int(round(max(r["blob"][3] for r in rows if r["blob"] is not None) / max(p["band_bottom"], 0.01))) if any(r["blob"] for r in rows) else 0
    H = rows[0]["hsv"].shape[0] if rows[0]["hsv"] is not None else H
    print("bag: %d scans with an image, %d lidar cones in the camera's view, image %dx%d" % (scans, len(rows), W, H))
    out = {}
    for name, (fit, note) in (("camera geometry", fit_geometry(rows, W, p, truth is not None)), ("image band", fit_band(rows, H)),
                              ("brightness gate", fit_brightness(rows, truth is not None))):
        print("%s: %s" % (name, note))
        if fit:
            out.update(fit)
    if truth is not None:
        fit, note = fit_hsv(rows)
        print("hsv bounds: %s" % note)
        out.update(fit)
        print("colour accuracy with the current parameters, camera readings against the true cones:\n" + colour_report(rows))
    print("\n# paste into config/racer.yaml (ros__parameters)")
    for k, v in out.items():
        print("    %s: %s" % (k, v))


if __name__ == "__main__":
    main()
