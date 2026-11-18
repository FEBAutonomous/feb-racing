"""Cone perception: lidar clusters + camera colour, fused by bearing, colour carried between scans.

Same method as feb_cone_driver (the simple driver), packaged as a class so the racer can call
it per scan. Frames: car frame, x forward, y left, origin at the lidar. Images are decoded with
numpy (no cv_bridge) so the package has no dependency beyond numpy and OpenCV.
"""
import math

import cv2
import numpy as np

BLUE, YELLOW, ORANGE = 1, 2, 3          # orange: the big start-line cones
UNKNOWN = 0                              # a lidar cone the camera has not coloured (beside or behind the car)


class Perception:
    def __init__(self, p):
        self.p = p
        self.blobs = []                 # [(colour, u_centre_px, top_row_px, bottom_row_px, area, dim)] from the latest image
        self.image_width = None
        self.prev = []                  # last scan's cones: (x, y, colour, age, colour votes)
        self.prev_yaw = None

    # ------------------------------------------------------------ camera

    def on_image(self, msg):
        if msg.encoding not in ("rgb8", "bgr8"):
            return
        img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
        if msg.encoding == "rgb8":
            img = img[:, :, ::-1]
        self.image_width = msg.width
        h, w = img.shape[:2]
        hsv = cv2.cvtColor(np.ascontiguousarray(img), cv2.COLOR_BGR2HSV)
        # HSV bounds per colour (h_lo, s_lo, v_lo, h_hi, s_hi, v_hi), calibrated on this camera:
        # the cones are dark in this scene, so the value floor is low and colour comes from hue
        # and saturation; the sky is a desaturated blue, so blue needs high saturation
        bounds = {BLUE: self.p["hsv_blue"], YELLOW: self.p["hsv_yellow"], ORANGE: self.p["hsv_orange"]}
        top, bottom, floor = self.p["band_top"], self.p["band_bottom"], self.p["band_floor"]
        blobs = []
        for colour, b in bounds.items():
            mask = cv2.inRange(hsv, tuple(int(v) for v in b[:3]), tuple(int(v) for v in b[3:]))
            mask[: int(top * h), :] = 0              # sky
            mask[int(bottom * h):, :] = 0            # the car's own body and the floor
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for c in contours:
                area = cv2.contourArea(c)
                if not self.p["min_blob_px"] <= area <= 0.02 * w * h:
                    continue
                x, y, bw, bh = cv2.boundingRect(c)
                if y + bh < floor * h:                   # entirely above the horizon: sky
                    continue
                # a cone is brighter than the ground: a handful of dark pixels with the right hue
                # (a speck on the ground, a streak at the horizon) is not one, and matched to a
                # lidar cone it hands that cone the wrong colour
                inside = mask[y:y + bh, x:x + bw] > 0
                dim = float(np.mean(hsv[y:y + bh, x:x + bw, 2][inside])) < self.p["blob_min_value"]
                blobs.append([colour, x + bw / 2.0, float(y), float(y + bh), area, dim, float(x), float(x + bw)])
        blobs = self.merge_stripes(blobs)
        # the start cones are big: a small orange-hued blob is a shaded yellow cone
        self.blobs = [(YELLOW if c == ORANGE and area < self.p["orange_min_px"] else c, u, top_, bot, area, dim) for c, u, top_, bot, area, dim, _, _ in blobs]

    @staticmethod
    def merge_stripes(blobs):
        """A striped cone is two or three blobs of its colour stacked in the same columns (the white
        and black bands cut it). One cone must be one blob, or the one-to-one matching hands the
        pieces of one cone to two lidar cones and the colours shuffle along the row. Blobs of a
        colour whose column spans overlap, and that are no further apart vertically than the taller
        of the two, are merged into one (union box, areas summed)."""
        out = []
        for colour in sorted({b[0] for b in blobs}):
            group = sorted((b for b in blobs if b[0] == colour), key=lambda b: b[1])
            merged = []
            for b in group:
                for m in merged:
                    overlap = min(b[7], m[7]) - max(b[6], m[6])
                    width = min(b[7] - b[6], m[7] - m[6])
                    gap = max(b[2], m[2]) - min(b[3], m[3])
                    tall = max(b[3] - b[2], m[3] - m[2])
                    if overlap > 0.4 * width and gap < 1.2 * tall:
                        total = b[4] + m[4]
                        m[6], m[7] = min(b[6], m[6]), max(b[7], m[7])
                        m[1] = (m[6] + m[7]) / 2.0
                        m[2], m[3] = min(b[2], m[2]), max(b[3], m[3])
                        m[5] = bool(m[5] and b[5])          # dim only if every piece is dim
                        m[4] = total
                        break
                else:
                    merged.append(list(b))
            out.extend(merged)
        return out

    # ------------------------------------------------------------ lidar

    def clusters(self, msg):
        """Small tight clusters within max_range: cone candidates as (x, y) in the car frame."""
        ranges = np.asarray(msg.ranges, dtype=float)
        angles = msg.angle_min + np.arange(len(ranges)) * msg.angle_increment
        self.last_scan = (ranges, angles)          # the beams, for the camera-first observations (see assign)
        ok = np.isfinite(ranges) & (ranges > 0.3) & (ranges < self.p["max_range"])
        xs, ys = ranges * np.cos(angles), ranges * np.sin(angles)
        groups, cur = [], []
        for i in np.flatnonzero(ok):
            if cur and math.hypot(xs[i] - xs[cur[-1]], ys[i] - ys[cur[-1]]) > self.p["cluster_gap"]:
                groups.append(cur)
                cur = []
            cur.append(i)
        if cur:
            groups.append(cur)
        out, iso, small, widths = [], [], [], []
        for g in groups:
            if len(g) > self.p["cone_max_points"]:
                continue
            px, py = xs[g], ys[g]
            w = math.hypot(px.max() - px.min(), py.max() - py.min())
            if w > self.p["cone_max_width"]:
                continue
            out.append((float(px.mean()), float(py.mean())))
            widths.append(w)
            small.extend(g)
        # the cluster's width at the lidar's height: a small cone shows 0.05-0.1 m, a big (orange, start gate) cone
        # 0.25-0.4 m; see big_cone_width
        self.cluster_width = widths
        # isolation: a cone stands clear of structure (a bench, a planter, a wall with a post); another cone-sized
        # cluster next to it is allowed (cones touching at a drain, the big gate cones in pairs). Structure = every
        # return within reach that is not part of a cone-sized cluster, including the ones beyond max_range
        radius = float(self.p.get("cone_isolation", 0.45))
        struct = np.isfinite(ranges) & (ranges > 0.1) & (ranges < self.p["max_range"] + 1.0)
        struct[small] = False
        ax, ay = xs[struct], ys[struct]
        for cx, cy in out:
            iso.append(int(np.sum((ax - cx) ** 2 + (ay - cy) ** 2 < radius * radius)) <= 2)
        self.cluster_isolated = iso
        return out

    # ------------------------------------------------------------ fusion

    def pixel_column(self, x, y, yaw_rate=0.0):
        """Image column where a point (x forward, y left, lidar frame) lands; NaN if behind the
        camera. Camera pose relative to the lidar, focal length, column bias and the image's lag
        behind the scan (the car keeps turning between the two) are calibrated parameters."""
        w = self.image_width
        fx = (w / 2.0) / math.tan(math.radians(self.p["camera_hfov_deg"]) / 2.0)
        cx = x - self.p["camera_ahead"]
        cy = y - self.p["camera_lateral"]
        if cx < 0.2:
            return float("nan")
        bearing = math.atan2(cy, cx) + yaw_rate * (self.p["camera_lag_s"] + getattr(self, "extra_lag", 0.0))   # where it was when the image was taken
        return w / 2.0 - fx * math.tan(bearing) + self.p["camera_col_bias"]

    # ---- the floor-row range: on a flat floor the image row where a cone meets the ground tells its range,
    # whatever the cone's size. bottom = b0 + b1 / range, fitted from every box the lidar matched (self-calibrating:
    # nothing about the camera's pitch or height is assumed), usable after floor_fit_min pairs.
    def _floor_fit(self):
        f = self.__dict__.get("floor_fit")
        if f is None:
            # seeded with a prior (floor_row_b0/b1, this camera's mount, worth floor_prior_n pairs) so the gate works
            # from the first frames; the lidar-matched boxes then take over
            b0, b1, n0 = float(self.p.get("floor_row_b0", 0.0)), float(self.p.get("floor_row_b1", 0.0)), float(self.p.get("floor_prior_n", 0.0))
            f = {"n": 0, "sx": 0.0, "sy": 0.0, "sxx": 0.0, "sxy": 0.0}
            if n0 > 0 and b1 > 0:
                for x in np.linspace(1.0 / 6.0, 1.0 / 1.2, int(n0)):       # the prior as points at 1.2..6 m
                    f["n"] += 1; f["sx"] += x; f["sy"] += b0 + b1 * x; f["sxx"] += x * x; f["sxy"] += x * (b0 + b1 * x)
            self.floor_fit = f
        return f

    def floor_update(self, rng, bottom):
        f = self._floor_fit()
        x = 1.0 / max(rng, 0.3)
        f["n"] += 1; f["sx"] += x; f["sy"] += bottom; f["sxx"] += x * x; f["sxy"] += x * bottom
        if f["n"] > 2000:                          # a sliding memory: halve the sums so a new venue can take over
            for k in ("n", "sx", "sy", "sxx", "sxy"):
                f[k] *= 0.5

    def floor_range(self, bottom):
        """Range implied by a box's bottom row, or None before the model is fitted (or for a row the model
        cannot place: at or above the horizon, or cut by the frame's bottom edge)."""
        f = self._floor_fit()
        if f["n"] < int(self.p.get("floor_fit_min", 30)) or not self.image_height or bottom >= self.image_height - 3:
            return None
        d = f["n"] * f["sxx"] - f["sx"] ** 2
        if abs(d) < 1e-9:
            return None
        b1 = (f["n"] * f["sxy"] - f["sx"] * f["sy"]) / d
        b0 = (f["sy"] - b1 * f["sx"]) / f["n"]
        if b1 <= 1.0 or bottom - b0 <= 1.0:
            return None
        return b1 / (bottom - b0)

    def assign(self, clusters, yaw_rate=0.0):
        blobs = self.blobs      # one snapshot: the detector thread replaces the list while this runs
        """One-to-one matching of lidar cones to camera blobs by image column, closest pairs
        first, so two cones cannot claim the same blob. Returns per cluster (colour or None,
        ambiguous) where ambiguous means another cone sits within a few pixels of its column."""
        n = len(clusters)
        cols = [self.pixel_column(x, y, yaw_rate) if self.image_width else float("nan") for x, y in clusters]
        colour, dim, taken = [None] * n, [False] * n, set()
        conf = [1.0] * n                                  # a detector's confidence, when the blobs carry one
        self._conf = conf
        self._big = [False] * n                           # the box says big cone (see below)
        self._bottom = [None] * n                         # the matched box's bottom row (the floor-row range, see fuse)
        self._shift = {}                                  # cluster index -> where its big-cone observation really stands
        if blobs and self.image_width:
            tol = self.p["match_px_frac"] * self.image_width
            fx = (self.image_width / 2.0) / math.tan(math.radians(self.p["camera_hfov_deg"]) / 2.0)
            # Joint assignment of boxes to lidar cones (the AMZ / MIT Driverless recipe: lidar gives the range, the
            # camera the colour, and the two are tied by where a cone of that size at that range must appear in the
            # image). Each pair costs its column error plus how far the box height is from a cone's height at the
            # cluster's range; the cheapest one-to-one assignment over all pairs wins (Hungarian), so a big cone's
            # tall box goes to the cluster at the range that fits it, not to the small cone standing in front of it
            # on the same bearing. A box that fits no cluster, or a cone with no box, stays unmatched.
            m = len(blobs)
            cost = np.full((n, m), 1e6)
            big = np.zeros((n, m), dtype=bool)
            for i in range(n):
                if math.isnan(cols[i]):
                    continue
                rng = max(0.3, math.hypot(clusters[i][0] - self.p["camera_ahead"], clusters[i][1] - self.p["camera_lateral"]))
                for k, b in enumerate(blobs):
                    dc = abs(b[1] - cols[i])
                    if dc >= tol:
                        continue
                    if len(b) > 6:                           # a detector box: its height must fit a cone at this range
                        h = b[3] - b[2]
                        r_small, r_big = h / (fx * 0.22 / rng), h / (fx * 0.33 / rng)
                        is_big = abs(math.log(r_big)) < abs(math.log(r_small))
                        ratio = r_big if is_big else r_small
                        if not 0.5 <= ratio <= 2.0:
                            continue
                        # ...and its bottom row must put it at this range too (size-free): the big cone's box in
                        # front cannot land on the small cone standing behind it on the same bearing
                        rf = self.floor_range(b[3])
                        # (a big cone's box is held to a tighter band: a cluster that merges the big cone with the
                        # small cone behind it along a wall sits a few decimetres too far; unmatched, the box becomes
                        # a camera-first cone at the nearest lidar return on its bearing, which is the big cone itself)
                        # (a big cone's box is never handed to a cluster clearly farther than its floor row says: that
                        # is the small cone behind it along the wall, and the lidar often has no return on a big cone
                        # at all, its black base swallows the beam; a nearer cluster is allowed, the image lags)
                        if rf is not None and not (0.75 if is_big else 0.7) <= rf / rng <= 1.4:
                            continue
                        if rf is None and self.image_height and b[3] >= self.image_height - 3:
                            # the box runs off the bottom of the frame: the cone is nearer than the nearest row the
                            # floor model can place, so a cluster much farther than that is not this cone
                            near = self.floor_range(self.image_height - 4)
                            if near is not None and rng > 1.4 * near:
                                continue
                        cost[i, k] = (dc / tol) ** 2 + 4.0 * math.log(ratio) ** 2
                        big[i, k] = is_big and r_small >= 1.35
                    else:
                        cost[i, k] = (dc / tol) ** 2
            from scipy.optimize import linear_sum_assignment
            rows, colsk = linear_sum_assignment(cost)
            matched_boxes = set()
            for i, k in zip(rows, colsk):
                if cost[i, k] >= 1e6:
                    continue
                b = blobs[k]
                matched_boxes.add(k)
                if len(b) > 6 and self.image_height and b[3] < self.image_height - 3:
                    self.floor_update(max(0.3, math.hypot(clusters[i][0] - self.p["camera_ahead"], clusters[i][1] - self.p["camera_lateral"])), b[3])
                colour[i], dim[i] = b[0], b[5]
                if len(b) > 6:
                    conf[i] = float(b[6])
                    self._bottom[i] = float(b[3])
                    # size before class: a box the height of a big cone at this range is a big cone, and big cones
                    # are orange whatever the class says; a small-cone-sized box the class calls orange is a small
                    # cone with the gate behind it: no colour
                    if big[i, k]:
                        colour[i], conf[i], self._big[i] = ORANGE, max(conf[i], 1.0), True
                        # a big cone's observation stands where the lidar beams on the box's bearing first return
                        # (the big cone itself), not at the centroid of a cluster that may run on into the small
                        # cone and the wall behind it: that centroid gave the cone behind the orange votes
                        if getattr(self, "last_scan", None) is not None:
                            rg, ang = self.last_scan
                            fx2 = (self.image_width / 2.0) / math.tan(math.radians(self.p["camera_hfov_deg"]) / 2.0)
                            bearing = math.atan((self.image_width / 2.0 + self.p["camera_col_bias"] - b[1]) / fx2) - yaw_rate * (self.p["camera_lag_s"] + getattr(self, "extra_lag", 0.0))
                            rng_i = max(0.3, math.hypot(clusters[i][0] - self.p["camera_ahead"], clusters[i][1] - self.p["camera_lateral"]))
                            cx0, cy0 = rng_i * math.cos(bearing) + self.p["camera_ahead"], rng_i * math.sin(bearing) + self.p["camera_lateral"]
                            lb = math.atan2(cy0, cx0)
                            win = np.abs(np.arctan2(np.sin(ang - lb), np.cos(ang - lb))) < math.radians(2.0)
                            beams = rg[win & np.isfinite(rg) & (rg > 0.3)]
                            if len(beams):
                                r_near = float(np.min(beams))
                                if r_near < 0.85 * math.hypot(clusters[i][0], clusters[i][1]):
                                    self._shift[i] = (r_near * math.cos(lb), r_near * math.sin(lb))
                    elif colour[i] == ORANGE:
                        colour[i] = None
        amb_px = 0.065 * self.image_width if self.image_width else 12.0
        ambiguous = [not math.isnan(cols[i]) and any(j != i and not math.isnan(cols[j]) and abs(cols[j] - cols[i]) < amb_px for j in range(n)) for i in range(n)]
        # camera-first: a sure box that fits no cluster is still a cone (one the lidar folded into a wall it stands
        # against, or one too small for a cluster). Its bearing comes from the box column, its range from the lidar
        # beams on that bearing (they hit the cone before the wall), checked against the floor-row range
        self.camera_only = []
        if blobs and self.image_width and self.p.get("camera_only_conf", 0.0) > 0.0 and getattr(self, "last_scan", None) is not None:
            fx = (self.image_width / 2.0) / math.tan(math.radians(self.p["camera_hfov_deg"]) / 2.0)
            rg, ang = self.last_scan
            for k, b in enumerate(blobs):
                if k in matched_boxes:
                    continue
                if len(b) <= 6 or float(b[6]) < self.p["camera_only_conf"] or b[5]:
                    continue
                rf = self.floor_range(b[3])
                if rf is None or rf > self.p["max_range"]:
                    continue
                bearing = math.atan((self.image_width / 2.0 + self.p["camera_col_bias"] - b[1]) / fx) - yaw_rate * (self.p["camera_lag_s"] + getattr(self, "extra_lag", 0.0))
                cx, cy = rf * math.cos(bearing) + self.p["camera_ahead"], rf * math.sin(bearing) + self.p["camera_lateral"]
                lb = math.atan2(cy, cx)
                win = np.abs(np.arctan2(np.sin(ang - lb), np.cos(ang - lb))) < math.radians(2.0)
                beams = rg[win & np.isfinite(rg) & (rg > 0.3)]
                h = b[3] - b[2]
                r_small0, r_big0 = h / (fx * 0.22 / rf), h / (fx * 0.33 / rf)
                big_box = abs(math.log(r_big0)) < abs(math.log(r_small0)) and r_small0 >= 1.35
                if len(beams) == 0 or float(np.min(beams)) > 1.3 * rf:
                    # no return on that bearing, or only the wall far behind: a big cone's black base swallows the
                    # beam. A big orange box stands at the range its floor row says; a small cone needs the lidar
                    if not (b[0] == ORANGE and big_box):
                        continue
                    r_lidar = rf
                else:
                    r_lidar = float(np.min(beams))        # the nearest return on that bearing: the cone, not the wall behind it
                    if not 0.7 <= r_lidar / rf <= 1.3:
                        continue
                r_small, r_big = h / (fx * 0.22 / r_lidar), h / (fx * 0.33 / r_lidar)
                col = b[0]
                if col == ORANGE:
                    if not (abs(math.log(r_big)) < abs(math.log(r_small)) and r_small >= 1.35):
                        continue                           # a small orange box is the gate behind a small cone
                reach = float(np.clip(1.3 - 0.15 * r_lidar, 0.35, 1.0))
                self.camera_only.append((r_lidar * math.cos(lb), r_lidar * math.sin(lb), col, 0.6 * reach * float(b[6])))
        return colour, ambiguous, dim

    def fuse(self, clusters, speed, yaw, dt, yaw_rate=0.0):
        """Colour each lidar cone: camera first, then the colour it had last scan (moved into
        this scan's frame), then a side guess for near cones. Returns [(x, y, colour, weight)]: weight
        1.0 when the camera matched this cone this scan and no other cone shares its bearing,
        0.3 when the colour was carried from an earlier scan, 0.0 for a side guess. Clusters
        that get no colour are kept in self.unknown: the lidar sees all round, the camera does
        not, and a cone is a cone for localisation."""
        dyaw = 0.0
        if yaw is not None and self.prev_yaw is not None:
            dyaw = math.atan2(math.sin(yaw - self.prev_yaw), math.cos(yaw - self.prev_yaw))
        self.prev_yaw = yaw
        ds = speed * dt
        c, s = math.cos(dyaw), math.sin(dyaw)
        moved = []
        for x, y, colour, age, votes in self.prev:
            x0, y0 = x - ds, y
            moved.append((c * x0 + s * y0, -s * x0 + c * y0, colour, age + 1, votes))
        cam_colour, ambiguous, dim = self.assign(clusters, yaw_rate)
        out, result = [], []
        for i, (x, y) in enumerate(clusters):
            # the camera's word counts for less the farther the cone: a far cone is a few pixels
            # wide and the blob of the cone behind it on the same bearing is easily taken for it
            # (on this camera one reading in five is wrong beyond 4 m, one in thirty within 2 m)
            reach = float(np.clip(1.3 - 0.15 * math.hypot(x, y), 0.35, 1.0))
            colour, age = cam_colour[i], 0
            weight = 0.0 if colour is None else (0.3 if ambiguous[i] else 1.0) * reach * self._conf[i]
            if colour is not None and getattr(self, "_big", None) and self._big[i]:
                weight = reach * self._conf[i]          # a box the size of a big cone is unmistakable, pair or not
            # the same cone last scan: its colour votes carry over, so one wrong reading of a
            # cone the camera has seen right several times does not change it
            votes = {}
            best, best_d = None, self.p["track_gate"]
            for px, py, pc, page, pv in moved:
                d = math.hypot(px - x, py - y)
                if d < best_d and page < self.p["track_memory"]:
                    best, best_d = (pc, page, pv), d
            if best is not None:
                votes = {c: 0.9 * v for c, v in best[2].items()}
            if colour is not None and not dim[i]:
                votes[colour] = votes.get(colour, 0.0) + weight
                colour = max(votes, key=votes.get)
            elif best is not None:
                colour, age = best[0], best[1]
                weight = 0.3
            elif colour is not None:
                # a dim blob (a dark far cone, or a speck on the ground): too doubtful to vote
                # for the map, but still a better guess for driving than the side of the car
                weight = 0.0
            if colour is None and 0.0 < x < self.p["side_guess_range"] and abs(y) < self.p["track_width"]:
                colour, age, weight = (BLUE if y > 0.0 else YELLOW), self.p["track_memory"] - 1, 0.0
            # a cone right beside the car is that side's boundary as long as the car is between
            # the lines: its side beats a colour remembered from when it was far ahead (the camera
            # cannot see it here). Such a cone drives the car but does not vote for the map.
            if colour is not None and x < self.p["side_override_x"] and abs(y) > self.p["side_override_y"] and abs(y) < self.p["track_width"]:
                side = BLUE if y > 0.0 else YELLOW
                if colour != side and colour != ORANGE:
                    colour, weight = side, 0.0
            out.append((x, y, colour, age, votes))
            if colour is not None:
                sx, sy = getattr(self, "_shift", {}).get(i, (x, y))
                result.append((sx, sy, colour, weight))
        self.prev = out
        self.unknown = [(x, y) for x, y, colour, _, _ in out if colour is None]
        iso = getattr(self, "cluster_isolated", None) or [False] * len(out)
        self.unknown_isolated = [iso[i] for i, (x, y, colour, _, _) in enumerate(out) if colour is None]   # see racer.on_scan
        wid = getattr(self, "cluster_width", None) or [0.0] * len(out)
        self.unknown_width = [wid[i] for i, (x, y, colour, _, _) in enumerate(out) if colour is None]
        # size is colour for the big cones: a cluster at least big_cone_width wide is a big cone and big cones are
        # orange (full vote); a narrow cluster is a small cone and can never be orange, so an orange box on it is the
        # bearing match taking the big cone behind it (its vote is dropped, the cone stays uncoloured this scan)
        big = float(self.p.get("big_cone_width", 0.0))
        if big > 0.0:
            fixed, k, stray = [], 0, []
            for i, (x, y, colour, _, _) in enumerate(out):
                if colour is None:
                    continue
                rx, ry, rc, rw = result[k]; k += 1
                if rc == ORANGE and wid[i] < big and not self._big[i]:
                    # an orange box on a narrow cluster: a small cone (or the sliver of a half-hidden big one) with the
                    # big cone on the same bearing; the box goes to the big cluster next to it if there is one
                    stray.append((rx, ry, rw, i))
                else:
                    fixed.append((rx, ry, rc, rw))
            fixed_idx = {id(r) for r in fixed}
            for rx, ry, rw, i in stray:
                b = math.atan2(ry, rx); given = False
                # the box's floor row says where the big cone stands: the wide cluster that takes the box must be there
                # (a box cut by the frame's bottom is nearer than the nearest row the floor model can place)
                bottom = self._bottom[i] if i < len(getattr(self, "_bottom", [])) else None
                rf = self.floor_range(bottom) if bottom is not None else None
                near = self.floor_range(self.image_height - 4) if (rf is None and bottom is not None and self.image_height and bottom >= self.image_height - 3) else None
                for j, (wx, wy) in enumerate(clusters):
                    if j == i or wid[j] < big or abs(math.atan2(math.sin(math.atan2(wy, wx) - b), math.cos(math.atan2(wy, wx) - b))) > 0.12 or math.hypot(wx - rx, wy - ry) > 1.0:
                        continue
                    rj = math.hypot(wx - self.p["camera_ahead"], wy - self.p["camera_lateral"])
                    if rf is not None and not 0.7 <= rf / max(rj, 0.3) <= 1.4:
                        continue
                    if near is not None and rj > 1.4 * near:
                        continue
                    for n, r in enumerate(fixed):
                        if r[0] == wx and r[1] == wy:
                            fixed[n] = (wx, wy, ORANGE, max(r[3], rw)); given = True; break
                    if not given:
                        # the big cluster had no colour (it is in the uncoloured list): colour it now
                        for n, (ux, uy) in enumerate(self.unknown):
                            if ux == wx and uy == wy:
                                del self.unknown[n]; del self.unknown_isolated[n]; del self.unknown_width[n]
                                fixed.append((wx, wy, ORANGE, rw)); given = True; break
                    if given:
                        break
                self.unknown.append((rx, ry)); self.unknown_isolated.append(iso[i]); self.unknown_width.append(wid[i])
            result = fixed
        for cx, cy, cc, cw in getattr(self, "camera_only", []):
            # not within half a metre of an observation the lidar already made: the same cone twice in one scan would
            # file as two landmarks (one observation per landmark per keyframe) that are then "seen together" for good
            if all(math.hypot(cx - rx, cy - ry) > 0.5 for rx, ry, _, _ in result):
                result.append((cx, cy, cc, cw))
        return result
