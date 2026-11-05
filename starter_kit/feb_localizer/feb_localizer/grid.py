"""The occupancy grid the driver builds on the mapping lap: every scan's endpoints are stamped into
it at the pose the Localizer gives. The grid is what update_mapping / update_racing match against.

    grid.res, grid.size      cell size (m) and width in cells; the car starts at the centre
    grid.counts              uint8 (size, size): how many scans hit each cell
    grid.occupied()          bool (size, size): cells hit at least `min_hits` times
    grid.insert(points, pose)   stamp a scan (n, 2 in the car frame) at pose (x, y, yaw)
    grid.fit(points, pose)      fraction of the scan's endpoints that land on an occupied cell (or
                                one of its 8 neighbours) when placed at pose: 1.0 is a perfect
                                match, under ~0.45 the pose is wrong
    grid.cells(points, pose)    (row, col) integer arrays for the scan placed at pose
"""
import math

import numpy as np


class Grid:
    def __init__(self, res=0.05, size_m=60.0, min_hits=2):
        self.res = res
        self.size = int(size_m / res)
        self.min_hits = min_hits
        self.counts = np.zeros((self.size, self.size), dtype=np.uint8)
        self.half = self.size // 2
        self._occ = None
        self.dirty = True

    def to_world(self, points, pose):
        x, y, yaw = pose
        c, s = math.cos(yaw), math.sin(yaw)
        wx = x + points[:, 0] * c - points[:, 1] * s
        wy = y + points[:, 0] * s + points[:, 1] * c
        return wx, wy

    def cells(self, points, pose):
        wx, wy = self.to_world(points, pose)
        col = np.floor(wx / self.res).astype(int) + self.half
        row = np.floor(wy / self.res).astype(int) + self.half
        ok = (row >= 0) & (row < self.size) & (col >= 0) & (col < self.size)
        return row[ok], col[ok]

    def insert(self, points, pose):
        row, col = self.cells(points, pose)
        np.add.at(self.counts, (row, col), 1)
        self.dirty = True

    def occupied(self):
        if self.dirty or self._occ is None:
            occ = self.counts >= self.min_hits
            # one cell of slack in every direction, so a scan a few cm off still counts as a hit
            dil = occ.copy()
            dil[1:, :] |= occ[:-1, :]; dil[:-1, :] |= occ[1:, :]
            dil[:, 1:] |= occ[:, :-1]; dil[:, :-1] |= occ[:, 1:]
            self._occ = dil
            self.dirty = False
        return self._occ

    def fit(self, points, pose):
        if len(points) == 0:
            return 0.0
        row, col = self.cells(points, pose)
        if len(row) == 0:
            return 0.0
        return float(self.occupied()[row, col].mean())
