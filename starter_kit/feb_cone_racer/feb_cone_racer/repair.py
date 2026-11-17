"""Track repair at closure: make the closed map read as a track.

Each boundary (blue, yellow) is taken in the order the car drove past it, along the mapping
lap's own path: every cone gets an arc length s along the path and a lateral offset d from it.
In that frame a boundary is a chain with a steady spacing, and two things stand out at once:
a cone much closer to its neighbour than the chain's spacing (a duplicate from drift), and a
gap of two or three spacings (missing cones). Duplicates are merged, gaps are filled with cones
interpolated in (s, d), so a fill follows the bend of the track rather than cutting the corner.

Deliberately small and conservative: a fill needs a gap that is clearly a whole number of
spacings, is never more than `max_fill` cones, never lands within `min_clearance` of any cone,
and never happens where the chain's spacing itself is unknown (fewer than 4 cones). Orange cones
are left alone. Nothing here depends on the sensors: it is geometry on the finished map, and it
is one switch (racer parameter track_repair) to turn off.
"""
import numpy as np

from .perception import BLUE, ORANGE, UNKNOWN, YELLOW


def path_frame(path):
    """Closed path -> cumulative arc length per vertex and the segments."""
    P = np.asarray(path, float)
    seg = np.roll(P, -1, axis=0) - P
    seglen = np.linalg.norm(seg, axis=1) + 1e-9
    s0 = np.concatenate([[0.0], np.cumsum(seglen)[:-1]])
    return P, seg, seglen, s0


def to_sd(P, seg, seglen, s0, q):
    """(s, d) of point q: arc length of its foot on the path and signed lateral offset (left positive)."""
    v = q - P
    t = np.clip(np.einsum("ij,ij->i", v, seg) / seglen ** 2, 0.0, 1.0)
    foot = P + t[:, None] * seg
    dist = np.linalg.norm(foot - q, axis=1)
    k = int(np.argmin(dist))
    cross = seg[k, 0] * (q[1] - P[k, 1]) - seg[k, 1] * (q[0] - P[k, 0])
    return s0[k] + t[k] * seglen[k], float(np.sign(cross) * dist[k])


def from_sd(P, seg, seglen, s0, s, d):
    """Point at arc length s (wrapped) and lateral offset d."""
    total = s0[-1] + seglen[-1]
    s = s % total
    k = int(np.searchsorted(s0, s, side="right") - 1)
    t = (s - s0[k]) / seglen[k]
    foot = P[k] + t * seg[k]
    n = np.array([-seg[k, 1], seg[k, 0]]) / seglen[k]      # left normal
    return foot + d * n


def repair(lhat, colour, path, fill_ratio=1.7, max_fill=3, dup_ratio=0.35, min_clearance=0.3, max_offset=None, off_line=0.35):
    """Returns (points, colours, inferred_mask, report). `inferred_mask` marks the cones that were added.
    `path` is the closed mapping-lap path (the solved keyframe poses)."""
    lhat = np.asarray(lhat, float).reshape(-1, 2)
    colour = np.asarray(colour, int)
    report = {"removed": 0, "added": 0, "chains": {}}
    if len(path) < 8 or len(lhat) < 6:
        return lhat, colour, np.zeros(len(lhat), bool), report
    P, seg, seglen, s0 = path_frame(path)
    total = s0[-1] + seglen[-1]
    keep = np.ones(len(lhat), bool)
    added_pts, added_col = [], []
    for c in (BLUE, YELLOW):
        idx = np.flatnonzero(colour == c)
        if len(idx) < 4:
            continue
        sd = np.array([to_sd(P, seg, seglen, s0, lhat[i]) for i in idx])
        if max_offset is not None:                       # a cone far off the path is not this boundary
            far = np.abs(sd[:, 1]) > max_offset
            keep[idx[far]] = False
            report["removed"] += int(far.sum())
            idx, sd = idx[~far], sd[~far]
            if len(idx) < 4:
                continue
        order = np.argsort(sd[:, 0])
        idx, sd = idx[order], sd[order]
        # two cones of one colour at the same place along the lap but at different distances from the path are one
        # cone filed twice from two passes that disagreed (the drift of a pass that left the track): the one farther
        # from the chain's usual offset goes
        if len(idx) >= 6 and off_line > 0.0:
            usual = float(np.median(np.abs(sd[:, 1])))
            drop = np.zeros(len(idx), bool)
            for a in range(len(idx)):
                for b in range(a + 1, len(idx)):
                    if drop[a] or drop[b]:
                        continue
                    ds_ = min(abs(sd[b, 0] - sd[a, 0]), total - abs(sd[b, 0] - sd[a, 0]))
                    if ds_ < 0.3 and abs(sd[b, 1] - sd[a, 1]) > 0.3:
                        worse = a if abs(abs(sd[a, 1]) - usual) > abs(abs(sd[b, 1]) - usual) else b
                        drop[worse] = True
            if drop.any():
                keep[idx[drop]] = False
                report["removed"] += int(drop.sum())
                report["off_line"] = report.get("off_line", 0) + int(drop.sum())
                idx, sd = idx[~drop], sd[~drop]
                if len(idx) < 4:
                    continue
        # a cone that has strayed off its boundary line: its lateral offset from the path differs from its
        # neighbours' along the chain by more than off_line (a double that drifted onto the track, a sighting of the
        # other boundary filed with this colour). The chain's offset profile is the median of the four neighbours
        if len(idx) >= 6 and off_line > 0.0:
            n_c = len(idx)
            stray = np.zeros(n_c, bool)
            for a in range(n_c):
                around = [sd[(a + o) % n_c, 1] for o in (-2, -1, 1, 2)]
                # only a cone that has come IN towards the path (a double that drifted onto the track): a cone
                # standing farther out than its neighbours is left alone, the path may just have cut the corner
                if abs(sd[a, 1]) < abs(np.median(around)) - off_line:
                    stray[a] = True
            if stray.any():
                keep[idx[stray]] = False
                report["removed"] += int(stray.sum())
                report.setdefault("off_line", 0)
                report["off_line"] += int(stray.sum())
                idx, sd = idx[~stray], sd[~stray]
                if len(idx) < 4:
                    continue
        # spacing along the chain, wrapped round the loop
        nxt = np.roll(np.arange(len(idx)), -1)
        gap_s = (sd[nxt, 0] - sd[:, 0]) % total
        gap = np.linalg.norm(lhat[idx[nxt]] - lhat[idx], axis=1)
        spacing = float(np.median(gap))
        if spacing <= 0.0:
            continue
        # the spacing a gap is judged against is local (the median of the four steps around it): a track can have a
        # tight slalom and a long straight, and a cone is missing relative to its own neighbours
        n_g = len(gap)
        def local(a):
            # ...but never a reference below the whole chain's spacing: on an uneven boundary a few short steps in a
            # row would otherwise make a normal step look like a gap (false fills); the stricter of the two decides
            around = [gap[(a + o) % n_g] for o in (-2, -1, 1, 2) if n_g > 4] or [spacing]
            return max(float(np.median(around)), spacing)
        # duplicates: a step much shorter than the spacing merges the pair (the later one goes, the first moves to the mean)
        removed = 0
        for a in range(len(idx)):
            b = nxt[a]
            if keep[idx[a]] and keep[idx[b]] and gap[a] < dup_ratio * local(a) and a != b:
                lhat[idx[a]] = 0.5 * (lhat[idx[a]] + lhat[idx[b]])
                keep[idx[b]] = False
                removed += 1
        # fills: a gap of n spacings (n >= 2, clearly) gets n-1 cones, interpolated in (s, d)
        added = 0
        for a in range(len(idx)):
            b = nxt[a]
            if not (keep[idx[a]] and keep[idx[b]]):
                continue
            n = gap[a] / local(a)
            k = int(round(n)) - 1
            if n < fill_ratio or k < 1:
                continue
            k = min(k, max_fill)
            for m in range(1, k + 1):
                f = m / (k + 1.0)
                s = sd[a, 0] + f * gap_s[a]
                d = sd[a, 1] + f * (sd[b, 1] - sd[a, 1])
                q = from_sd(P, seg, seglen, s0, s, d)
                others = np.vstack([lhat[keep], np.array(added_pts).reshape(-1, 2)])
                if len(others) and np.linalg.norm(others - q, axis=1).min() < min_clearance:
                    continue
                added_pts.append(q); added_col.append(c); added += 1
        report["chains"][int(c)] = {"cones": int(keep[idx].sum()), "spacing": round(spacing, 2), "removed": removed, "added": added}
        report["removed"] += removed
        report["added"] += added
    pts = np.vstack([lhat[keep], np.array(added_pts).reshape(-1, 2)])
    cols = np.concatenate([colour[keep], np.array(added_col, int)])
    inferred = np.concatenate([np.zeros(int(keep.sum()), bool), np.ones(len(added_pts), bool)])
    report["kept"] = keep
    return pts, cols, inferred, report
