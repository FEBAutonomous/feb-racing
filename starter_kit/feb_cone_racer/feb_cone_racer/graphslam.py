"""GraphSLAM with cones as landmarks, ported from the team's FSAE stack (graphslam_global).

The graph is linear: heading comes from the IMU, so every pose is (x, y) and every landmark is
(x, y). Two kinds of edges, each a pair of rows in a sparse system A x = b:
  odometry   x_k - x_{k-1} = dx          (weight dx_weight)
  landmark   l_j - x_k     = z           (weight z_weight; z is the cone seen from pose k, in
                                          world orientation)
Solving the normal equations gives every pose and landmark at once. Data association follows
the FSAE recipe: align the current observations to the map with a few ICP iterations, then
match each cone to the nearest landmark; anything farther than new_landmark_dist becomes a new
landmark. Colour is a vote: every observation adds to its landmark's blue or yellow count and
the majority wins, so one wrong camera reading does not create a ghost of the other colour.
When the car comes back to cones it has seen, the same matching closes the loop.

After the first lap the map is frozen and the same ICP alignment localises the car against it.
"""
import numpy as np
import scipy.sparse as sps
import scipy.sparse.linalg as spla

from .perception import UNKNOWN

BLUE, YELLOW, ORANGE = 1, 2, 3


class GraphSLAM:
    def __init__(self, x0=(0.0, 0.0), dx_weight=2.0, z_weight=1.0, new_landmark_dist=0.6,
                 icp_gate=1.5, icp_iterations=5, icp_min_matches=5, icp_max_shift=1.0, icp_max_turn=0.26):
        self.dx_weight, self.z_weight = dx_weight, z_weight
        self.icp_min_matches, self.icp_max_shift, self.icp_max_turn = icp_min_matches, icp_max_shift, icp_max_turn
        self.icp_max_residual = 0.5
        self.icp_gate_frac = 0.45               # gate never more than this fraction of the local landmark spacing
        self.new_landmark_dist, self.icp_gate, self.icp_iterations = new_landmark_dist, icp_gate, icp_iterations
        self.rows = []               # (row, col, value) triplets of A
        self.b = []
        self.nvars, self.neqns = 0, 0
        self.x = []                  # variable index of each pose
        self.l = []                  # variable index of each landmark
        self.xhat = np.zeros((0, 2))
        self.lhat = np.zeros((0, 2))
        self.votes = np.zeros((0, 4))          # per landmark: [unused, blue, yellow, orange] weighted colour votes
        self.count = np.zeros(0, dtype=int)    # per landmark: observations
        self.lidar_seen = np.zeros(0, dtype=int)   # per landmark: sightings the camera did not colour (lidar-only, see _admit_ghosts)
        self.x.append(0)
        self.nvars = 2
        self._eq([(0, 1.0)], x0[0])
        self._eq([(1, 1.0)], x0[1])
        self.xhat = np.array([[x0[0], x0[1]]], dtype=float)
        self.frozen = False
        self.obs = []                          # every keyframe's sightings: (keyframe, z_rel, colour, weight), see refile()
        self.colour_override = None            # colours repaired by the track builder, once frozen
        self.last_matched = []
        # lidar-only cones (the camera never coloured them: beside or behind the car) are tracked outside the graph
        # until they have been seen at ghost_keyframes keyframes in the same place; then they become landmarks.
        # Clutter and drift duplicates fail that test, a cone the car hugs passes it (sightings: (keyframe, z_rel, world))
        self.ghosts = []                       # [{"pos", "n", "last", "sightings"}]
        self.ghost_keyframes = 3
        self.ghost_radius = 0.3
        self.merge_radius = 0.25               # at closure, landmarks of one colour closer than this are one cone (racer: merge_radius)

    @property
    def colour(self):
        if self.colour_override is not None:
            return self.colour_override
        if len(self.votes) == 0:
            return np.zeros(0, dtype=int)
        c = np.argmax(self.votes[:, 1:], axis=1) + 1
        c[self.votes[:, 1:].sum(axis=1) <= 0.0] = UNKNOWN      # never coloured by the camera: unknown, not "blue" (argmax of zeros)
        return c

    @property
    def seen(self):
        return self.count

    def _eq(self, terms, rhs):
        for col, val in terms:
            self.rows.append((self.neqns, col, val))
        self.b.append(rhs)
        self.neqns += 1

    # ------------------------------------------------------------ data association

    def icp(self, z, colour, rotate=True):
        """Rigid transform (t, R) that best lays the world-frame cones z over the map, by
        mutual nearest neighbours of the same colour; identity if there is nothing to match.
        With rotate=False only the translation is fitted: the heading is known from the IMU,
        and three cones in a row (a straight) cannot pin a rotation, the fit then spins."""
        t, R = np.zeros(2), np.eye(2)
        if len(z) < 2 or len(self.lhat) < 2:
            return t, R
        lcol = self.colour
        # where landmarks stand close together (two track sections side by side, a hairpin's
        # inner cones) the gate shrinks to a fraction of the local spacing, so a cone is never
        # associated across to its neighbour's section
        gate = np.full(len(self.lhat), self.icp_gate)
        if self.icp_gate_frac > 0 and len(self.lhat) >= 2:
            dd = np.linalg.norm(self.lhat[:, None, :] - self.lhat[None, :, :], axis=2)
            np.fill_diagonal(dd, np.inf)
            gate = np.minimum(gate, self.icp_gate_frac * dd.min(axis=1))
        for _ in range(self.icp_iterations):
            zz = z @ R.T + t
            X, Y = [], []
            for c in np.unique(colour):
                zi = np.flatnonzero(colour == c)
                li = np.arange(len(lcol)) if c == UNKNOWN else np.flatnonzero((lcol == c) | (lcol == UNKNOWN))   # an uncoloured landmark matches any colour
                if not len(zi) or not len(li):
                    continue
                d = np.linalg.norm(zz[zi][:, None, :] - self.lhat[li][None, :, :], axis=2)
                fwd, bwd = np.argmin(d, axis=1), np.argmin(d, axis=0)
                for i, j in enumerate(fwd):
                    if bwd[j] == i and d[i, j] < gate[li[j]]:
                        X.append(z[zi[i]])
                        Y.append(self.lhat[li[j]])
            if len(X) < self.icp_min_matches:
                return np.zeros(2), np.eye(2)
            X, Y = np.array(X), np.array(Y)
            xm, ym = X.mean(axis=0), Y.mean(axis=0)
            if not rotate:
                R, t = np.eye(2), ym - xm
                continue
            U, _, Vt = np.linalg.svd((X - xm).T @ (Y - ym))
            Rn = (U @ Vt).T
            if np.linalg.det(Rn) < 0:
                Vt[-1, :] *= -1
                Rn = (U @ Vt).T
            R, t = Rn, ym - Rn @ xm
        residual = np.median(np.linalg.norm(X @ R.T + t - Y, axis=1))
        if np.linalg.norm(t) > self.icp_max_shift or abs(np.arctan2(R[1, 0], R[0, 0])) > self.icp_max_turn or residual > self.icp_max_residual:
            return np.zeros(2), np.eye(2)
        return t, R

    def relocalise(self, z_rel, colour, min_matches=4, near=None, radius=3.0):
        """Find the car anywhere on the frozen map from the cones in view, heading known (the
        IMU heading is absolute, so only a translation is unknown). Every pairing of a seen cone
        with a landmark of its colour is a candidate translation; the one that lays the most
        seen cones onto landmarks wins, if it is clear and unique. Returns (x, y) or None."""
        z_rel, colour = np.asarray(z_rel, float).reshape(-1, 2), np.asarray(colour, int)
        n = len(z_rel)
        if n < min_matches or len(self.lhat) < min_matches:
            return None
        lcol = self.colour
        cands = []
        for i in range(n):
            li = np.arange(len(lcol)) if colour[i] == UNKNOWN else np.flatnonzero((lcol == colour[i]) | (lcol == UNKNOWN))
            if len(li):
                cands.append(self.lhat[li] - z_rel[i])
        if not cands:
            return None
        T = np.vstack(cands)                                                   # (C, 2)
        if near is not None:                                                   # only positions close to the guess
            T = T[np.linalg.norm(T - np.asarray(near, float), axis=1) < radius]
            if len(T) == 0:
                return None
        d = np.linalg.norm((z_rel[None, :, None, :] + T[:, None, None, :]) - self.lhat[None, None, :, :], axis=3)   # (C, n, L)
        hit = d.min(axis=2) < 0.4
        score = hit.sum(axis=1)
        best = int(np.argmax(score))
        if score[best] < max(min_matches, 0.8 * n):
            return None
        others = np.linalg.norm(T - T[best], axis=1) > 0.6
        if np.any(others) and score[others].max() >= score[best] - 2:
            return None                # another place fits nearly as well (cones every metre look alike one cone along)
        return T[best].copy()

    def snap(self, z, colour, centre, radius, gate):
        """Loop closure with a wide net: align the world-frame cones z to the landmarks within
        `radius` of `centre` (the start area) allowing matches up to `gate` apart. Used when the
        orange start cones come back into view after a lap, when odometry drift may exceed the
        normal association gate. Returns the shift (t) and rotation (R), identity if unsure."""
        near = np.linalg.norm(self.lhat - centre, axis=1) < radius
        if near.sum() < 4 or len(z) < 4:
            return np.zeros(2), np.eye(2)
        saved = (self.lhat, self.votes, self.count, self.colour_override, self.icp_gate, self.icp_max_shift, self.icp_min_matches)
        try:
            self.lhat, self.votes, self.count = self.lhat[near], self.votes[near], self.count[near]
            self.colour_override = None if saved[3] is None else saved[3][near]
            self.icp_gate, self.icp_max_shift, self.icp_min_matches = gate, gate, 4
            return self.icp(z, colour)
        finally:
            self.lhat, self.votes, self.count, self.colour_override, self.icp_gate, self.icp_max_shift, self.icp_min_matches = saved

    # ------------------------------------------------------------ building the graph

    def add(self, dx, z_rel, colour, weight=None, ghosts=None):
        """One keyframe: the car moved dx (world frame) since the last one and now sees cones
        at z_rel (world orientation, relative to the car) with colours `colour`; `weight` is how
        much each colour reading counts (1 camera-confirmed, less for a remembered colour).
        `ghosts` are lidar-only cone candidates (same frame as z_rel): matched to a landmark they
        count as a sighting of it, otherwise they feed the ghost tracks (see __init__)."""
        dx, z_rel, colour = np.asarray(dx, float), np.asarray(z_rel, float).reshape(-1, 2), np.asarray(colour, int)
        weight = np.ones(len(colour)) if weight is None else np.asarray(weight, float)
        ghosts = np.zeros((0, 2)) if ghosts is None else np.asarray(ghosts, float).reshape(-1, 2)
        prev = self.x[-1]
        kf = len(self.x)
        self.obs.append((kf, z_rel.copy(), colour.copy(), weight.copy()))   # keyframe index into xhat, for refile()
        self.x.append(self.nvars)
        self.nvars += 2
        w = self.dx_weight
        self._eq([(self.x[-1], w), (prev, -w)], dx[0] * w)
        self._eq([(self.x[-1] + 1, w), (prev + 1, -w)], dx[1] * w)
        guess = self.xhat[-1] + dx
        self.xhat = np.vstack([self.xhat, guess])
        if len(z_rel) == 0 and len(ghosts) == 0:
            return
        zw = z_rel + guess
        t, R = self.icp(zw, colour) if len(z_rel) else (np.zeros(2), np.eye(2))
        zw = zw @ R.T + t
        self.xhat[-1] = R @ guess + t
        if len(ghosts):
            z_rel, colour, weight, ghost_rel = self._admit_ghosts(kf, z_rel, colour, weight, ghosts, R, t)
            zw = np.vstack([zw, ghost_rel + self.xhat[-1]]) if len(ghost_rel) else zw
        self.last_matched = []                  # landmarks this keyframe was associated with (loop closure looks at it)
        lcol = self.colour if len(self.lhat) else np.zeros(0, dtype=int)
        for i in range(len(zw)):
            j = None
            if len(self.lhat):
                d = np.linalg.norm(self.lhat - zw[i], axis=1)
                # colour is part of the association: within the gate, the nearest landmark of this colour (or
                # one not yet coloured); a landmark of another colour standing just as close is a different cone
                # (cones a metre apart with a few decimetres of pose jitter otherwise swap votes)
                ok = d < self.new_landmark_dist
                if colour[i] != UNKNOWN:
                    ok &= (lcol == colour[i]) | (lcol == UNKNOWN)
                if ok.any():
                    k = int(np.flatnonzero(ok)[np.argmin(d[ok])])
                    j = k
                    self.last_matched.append(k)
            if j is None:
                j = len(self.lhat)
                self.lhat = np.vstack([self.lhat, zw[i]])
                lcol = np.append(lcol, UNKNOWN)            # the new landmark has no colour yet (keeps lcol as long as lhat)
                self.votes = np.vstack([self.votes, np.zeros(4)])
                self.count = np.append(self.count, 0)
                self.lidar_seen = np.append(self.lidar_seen, 0)
                self.l.append(self.nvars)
                self.nvars += 2
            self.votes[j, colour[i]] += weight[i]
            self.count[j] += 1
            if colour[i] == UNKNOWN:
                self.lidar_seen[j] += 1
            w = self.z_weight
            self._eq([(self.l[j], w), (self.x[-1], -w)], z_rel[i, 0] * w)
            self._eq([(self.l[j] + 1, w), (self.x[-1] + 1, -w)], z_rel[i, 1] * w)

    def _admit_ghosts(self, kf, z_rel, colour, weight, ghosts, R, t):
        """Lidar-only candidates of this keyframe. One within 0.6 of the gate of a landmark is a sighting
        of it (an uncoloured observation: a position edge and a count, no colour vote). The rest go to the
        ghost tracks; a track seen at ghost_keyframes keyframes is promoted: its earlier sightings are
        refiled as observations too, so refile() and freeze() treat the cone like any other."""
        gw = (ghosts + self.xhat[-1])
        extra_rel, extra_col, extra_w = [], [], []
        for i in range(len(gw)):
            if len(self.lhat):
                d = np.linalg.norm(self.lhat - gw[i], axis=1)
                if d.min() < 0.6 * self.new_landmark_dist:
                    extra_rel.append(ghosts[i]); extra_col.append(UNKNOWN); extra_w.append(0.0)
                    continue
            hit = None
            for g in self.ghosts:
                if np.linalg.norm(g["pos"] - gw[i]) < self.ghost_radius and g["last"] != kf:
                    hit = g
                    break
            if hit is None:
                self.ghosts.append({"pos": gw[i].copy(), "n": 1, "last": kf, "sightings": [(kf, ghosts[i].copy())]})
                continue
            n = hit["n"]
            hit["pos"] = (hit["pos"] * n + gw[i]) / (n + 1)
            hit["n"], hit["last"] = n + 1, kf
            hit["sightings"].append((kf, ghosts[i].copy()))
            if hit["n"] >= self.ghost_keyframes:
                # promoted: this sighting becomes an observation now, the earlier ones are filed to their keyframes
                extra_rel.append(ghosts[i]); extra_col.append(UNKNOWN); extra_w.append(0.0)
                for k0, z0 in hit["sightings"][:-1]:
                    for j, (k, zr, cc, ww) in enumerate(self.obs):
                        if k == k0:
                            self.obs[j] = (k, np.vstack([zr, z0]), np.append(cc, UNKNOWN), np.append(ww, 0.0))
                            break
                self.ghosts = [g for g in self.ghosts if g is not hit]
        # tracks not seen for a while are dropped (a passer-by, a speck)
        self.ghosts = [g for g in self.ghosts if kf - g["last"] <= 6]
        if not extra_rel:
            return z_rel, colour, weight, np.zeros((0, 2))
        extra_rel = np.array(extra_rel).reshape(-1, 2)
        # the current keyframe's observation record grows by the admitted ghosts
        k, zr, cc, ww = self.obs[-1]
        self.obs[-1] = (k, np.vstack([zr, extra_rel]), np.append(cc, extra_col), np.append(ww, extra_w))
        return (np.vstack([z_rel, extra_rel]), np.append(colour, extra_col).astype(int), np.append(weight, extra_w), extra_rel)

    def solve(self):
        r, c, v = zip(*self.rows)
        A = sps.csr_matrix((v, (r, c)), shape=(self.neqns, self.nvars))
        b = np.asarray(self.b)
        sol = spla.spsolve((A.T @ A).tocsc(), A.T @ b)
        for k, i in enumerate(self.x):
            self.xhat[k] = sol[i:i + 2]
        for k, i in enumerate(self.l):
            self.lhat[k] = sol[i:i + 2]

    # ------------------------------------------------------------ frozen map

    def refile(self, radius=None):
        """Rebuild the landmarks from every sighting, placed through the keyframe poses as the solved
        graph now has them. During the lap a sighting was filed against the landmarks as believed at
        that moment, with the pose still drifting; the loop closure then moved the poses but not the
        votes, so cones a metre apart had merged or swapped colours. Here each sighting goes to the
        nearest landmark of its colour (or an uncoloured one) within `radius`, else starts a new one;
        positions are the mean of their sightings, votes and counts are recounted."""
        if not self.obs:
            return
        # tight: cones can stand half a metre apart (a gate pair, three at a drain); drift duplicates are merged after
        radius = radius if radius is not None else float(np.clip(0.6 * self.new_landmark_dist, 0.2, 0.3))
        pts, votes, counts, cols, lseen, kfs = [], [], [], [], [], []
        for k, z_rel, colour, weight in self.obs:
            if k >= len(self.xhat):
                continue
            world = self.xhat[k] + z_rel
            for i in range(len(world)):
                c = int(colour[i])
                j = None
                if pts:
                    P = np.array(pts); d = np.linalg.norm(P - world[i], axis=1)
                    ok = d < radius
                    if c != UNKNOWN:
                        ok &= (np.array(cols) == c) | (np.array(cols) == UNKNOWN)
                    if ok.any():
                        j = int(np.flatnonzero(ok)[np.argmin(d[ok])])
                if j is None:
                    pts.append(world[i].copy()); votes.append(np.zeros(4)); counts.append(0); cols.append(UNKNOWN); lseen.append(0); kfs.append(set())
                    j = len(pts) - 1
                kfs[j].add(k)
                n = counts[j]
                pts[j] = (pts[j] * n + world[i]) / (n + 1)
                counts[j] = n + 1
                if c == UNKNOWN:
                    lseen[j] += 1
                votes[j][c] += float(weight[i])
                v = votes[j][1:]
                cols[j] = int(np.argmax(v)) + 1 if v.sum() > 0 else UNKNOWN
        self.lhat = np.array(pts).reshape(-1, 2)
        self.votes = np.array(votes).reshape(-1, 4)
        self.count = np.array(counts, dtype=int)
        self.lidar_seen = np.array(lseen, dtype=int)
        self.kfs = kfs                         # keyframes each landmark was seen at: the co-visibility test in freeze()

    def freeze(self, min_seen=2, refile=True, lidar_min_seen=4, camera_only_min_seen=0):
        """Stop mapping; refile every sighting through the solved poses (see refile), drop landmarks
        seen fewer than min_seen times and merge any two of the same colour closer than half the
        association gate (duplicates from drift)."""
        self.solve()
        if refile:
            before = len(self.lhat)
            self.refile()
            self.refiled = (before, len(self.lhat))
        # a landmark stays if seen at min_seen keyframes, or seen once but coloured by the camera with a clear
        # vote (a small far cone passes few keyframes; a lidar-only speck seen once does not qualify)
        keep = (self.seen >= min_seen) | (self.votes[:, 1:].max(axis=1) >= 0.6)
        # a cone the camera never coloured stays only with lidar_min_seen sightings (see _admit_ghosts)
        uncoloured = self.votes[:, 1:].sum(axis=1) <= 0.0
        keep &= ~uncoloured | (self.seen >= lidar_min_seen)
        # a lidar-only cone seen fewer times still stays when it was seen together with its nearest neighbour in
        # several keyframes: two things in view at once are two things (the second cone of a gate pair, seen only
        # at the start); a weak one never seen together with its neighbour is that neighbour, drifted
        kf_all = getattr(self, "kfs", None)
        if kf_all is not None and len(kf_all) == len(keep):
            for i in np.flatnonzero(uncoloured & ~keep & (self.seen >= 2)):
                d = np.linalg.norm(self.lhat - self.lhat[i], axis=1); d[i] = np.inf
                j = int(np.argmin(d))
                if d[j] < 0.7 and len(kf_all[i] & kf_all[j]) >= 2:
                    keep[i] = True
        # a cone the camera coloured a few times but the lidar never confirmed on its own (a passer-by's leg, a far
        # cluster that took a cone's colour by bearing) goes too, when the lidar-only admission is on
        if camera_only_min_seen > 0 and len(self.lidar_seen) == len(keep):
            keep &= uncoloured | (self.seen >= camera_only_min_seen) | (self.lidar_seen >= 2)
        # what the pruning dropped, for the audit tools
        self.dropped = [{"x": float(self.lhat[i, 0]), "y": float(self.lhat[i, 1]), "count": int(self.count[i]), "votes": [float(v) for v in self.votes[i, 1:]],
                         "lidar_seen": int(self.lidar_seen[i]) if i < len(self.lidar_seen) else -1} for i in np.flatnonzero(~keep)]
        lhat, votes, count = self.lhat[keep], self.votes[keep], self.count[keep]
        kfs = [k for k, ok in zip(getattr(self, "kfs", []), keep) if ok] if len(getattr(self, "kfs", [])) == len(keep) else None
        col = np.argmax(votes[:, 1:], axis=1) + 1
        col[votes[:, 1:].sum(axis=1) <= 0.0] = UNKNOWN
        # duplicates from drift: two landmarks of one colour much closer to each other than either is to the next cone
        # of that colour (0.4 of that local spacing, at least merge_radius, at most 1 m). An outer ring with cones 2.5 m
        # apart merges a 0.6 m double; three yellows touching at a drain do not. The big orange gate cones stand in
        # pairs half a metre apart: a fixed quarter metre for them
        merge_radius = self.merge_radius if refile else 0.5 * self.new_landmark_dist + 0.2
        merged = np.ones(len(lhat), dtype=bool)
        def spacing(i, j):
            same = np.flatnonzero(((col == col[i]) | (col == UNKNOWN) | (col[i] == UNKNOWN)) & merged)
            same = same[(same != i) & (same != j)]
            return np.linalg.norm(lhat[same] - lhat[i], axis=1).min() if len(same) else 2.0
        for i in range(len(lhat)):
            if not merged[i]:
                continue
            for j in range(i + 1, len(lhat)):
                if not merged[j]:
                    continue
                # one cone the camera called yellow on the way in and blue on the way out is filed twice with two
                # colours: near-coincident, never seen together -> one cone, the votes decide
                same_spot = kfs is not None and np.linalg.norm(lhat[i] - lhat[j]) < 0.3 and len(kfs[i] & kfs[j]) == 0
                if not (col[i] == col[j] or col[i] == UNKNOWN or col[j] == UNKNOWN or same_spot):
                    continue
                if ORANGE in (col[i], col[j]):
                    radius_ij = min(merge_radius, 0.25)
                else:
                    radius_ij = float(np.clip(0.4 * min(spacing(i, j), spacing(j, i)), merge_radius, 1.0))
                dij = np.linalg.norm(lhat[i] - lhat[j])
                self.merge_log = getattr(self, "merge_log", [0, 0])
                # co-visibility: two cones standing half a metre apart are seen together in scan after scan; one cone
                # filed twice (the early pass and the late pass of a drifting lap) never is
                never_together = kfs is not None and len(kfs[i] & kfs[j]) <= 1 and dij < 0.7
                if dij < radius_ij or never_together or same_spot:
                    self.merge_log[0 if dij < radius_ij else 1] += 1
                    if col[i] != col[j] and col[j] != UNKNOWN:          # the majority colour after pooling the votes
                        pooled = votes[i] + votes[j]
                        col[i] = int(np.argmax(pooled[1:])) + 1 if pooled[1:].sum() > 0 else UNKNOWN
                    wi, wj = count[i], count[j]
                    lhat[i] = (wi * lhat[i] + wj * lhat[j]) / (wi + wj + 1e-9)
                    votes[i] += votes[j]
                    count[i] += count[j]
                    if kfs is not None:
                        kfs[i] = kfs[i] | kfs[j]
                    merged[j] = False
        self.lhat, self.votes, self.count = lhat[merged], votes[merged], count[merged]
        self.lidar_seen = (self.lidar_seen[keep] if len(self.lidar_seen) == len(keep) else np.zeros(int(keep.sum()), dtype=int))[merged]
        if kfs is not None:
            self.kfs = [k for k, ok in zip(kfs, merged) if ok]
        self.freeze_counts = (int(len(keep)), int(keep.sum()), int(merged.sum()))     # before, seen >= min_seen, after merge
        self.merge_log = getattr(self, "merge_log", [0, 0])
        self.frozen = True

    def localise(self, pose_guess, z_rel, colour, wide=True, subset=None):
        """Correct a dead-reckoned position against the frozen map. Returns the corrected (x, y)
        and how many cones matched. With `wide`, a failed match is retried as a unique translation
        search within 3 m."""
        pose_guess = np.asarray(pose_guess, float)
        z_rel, colour = np.asarray(z_rel, float).reshape(-1, 2), np.asarray(colour, int)
        if len(z_rel) < 3:
            return pose_guess, 0
        if subset is not None:                  # only the landmarks along the stretch the car is on
            saved = (self.lhat, self.votes, self.count, self.colour_override)
            try:
                self.lhat, self.votes, self.count = self.lhat[subset], self.votes[subset], self.count[subset]
                self.colour_override = None if saved[3] is None else saved[3][subset]
                return self.localise(pose_guess, z_rel, colour, wide=wide)
            finally:
                self.lhat, self.votes, self.count, self.colour_override = saved
        zw = z_rel + pose_guess
        saved_min = self.icp_min_matches
        self.icp_min_matches = 3                 # a hairpin shows only a few cones; three are enough to hold position
        try:
            # against a frozen map the geometry alone places the car: cones stand a metre or
            # more apart and the gate is a fraction of that, so a cone's colour (wrong one
            # time in twenty at speed) would only lose matches, never add one
            t, R = self.icp(zw, np.zeros_like(colour), rotate=False)
        finally:
            self.icp_min_matches = saved_min
        if not np.any(t) and wide:
            # nothing within the usual gate: the car may have drifted more than that. A wider
            # ICP net would happily lock one cone along on a regular layout, so instead search
            # the translations within 3 m and accept only a clear, unique fit.
            found = self.relocalise(z_rel, colour, near=pose_guess, radius=3.0)
            if found is None:
                return pose_guess, 0
            t, R = found - pose_guess, np.eye(2)
        if not np.any(t):
            return pose_guess, 0
        zz = zw @ R.T + t
        d = np.linalg.norm(zz[:, None, :] - self.lhat[None, :, :], axis=2)
        matched = int(np.sum(d.min(axis=1) < 0.5))
        return R @ pose_guess + t, matched
