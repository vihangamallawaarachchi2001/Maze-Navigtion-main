"""Occupancy-map loading and the planning lattice used by the DQN.

The lattice is derived from an occupancy map (normally the SLAM map) rather than
from the world geometry:
  * map image is read with explicit row inversion, origin and resolution
    (origin yaw must be 0; non-zero yaw is rejected);
  * unknown cells are treated as obstacles (no exploration mode);
  * lattice nodes are 0.5 m apart (corridor pitch of the maze) and their offset is
    found automatically by maximising robot clearance at node centres;
  * each node is then refined within +-`refine` m to the point of (saturated) maximum
    clearance, i.e. the corridor centre as actually observed in the SLAM map;
  * a node is valid if clearance >= robot radius + margin; an edge is valid only if
    every sampled point of the continuous straight segment satisfies the same
    clearance, so each grid move is a collision-free continuous motion.
"""
import hashlib
import math
import os

import numpy as np
import yaml
from PIL import Image

ACTIONS = [(1, 0), (-1, 0), (0, 1), (0, -1)]  # +x, -x, +y, -y (cardinal moves in map frame)
ACTION_NAMES = ["+x", "-x", "+y", "-y"]


class OccupancyMap:
    def __init__(self, yaml_path):
        with open(yaml_path) as f:
            meta = yaml.safe_load(f)
        img_path = meta["image"]
        if not os.path.isabs(img_path):
            img_path = os.path.join(os.path.dirname(os.path.abspath(yaml_path)), img_path)
        self.yaml_path = os.path.abspath(yaml_path)
        self.resolution = float(meta["resolution"])
        ox, oy, oyaw = (list(meta["origin"]) + [0.0])[:3]
        if abs(oyaw) > 1e-6:
            raise ValueError("map origin yaw != 0 is not supported")
        self.origin = (float(ox), float(oy))
        img = np.array(Image.open(img_path).convert("L"), dtype=np.float64)
        negate = int(meta.get("negate", 0))
        p = (img / 255.0) if negate else (255.0 - img) / 255.0  # occupancy probability
        occ_t, free_t = float(meta["occupied_thresh"]), float(meta["free_thresh"])
        self.height, self.width = img.shape
        # flip so that grid[row=iy][col=ix] has iy increasing with world y
        p = p[::-1, :]
        self.occupied = p > occ_t
        self.free = p < free_t
        self.unknown = ~(self.occupied | self.free)
        blocked = self.occupied | self.unknown
        iy, ix = np.nonzero(blocked)
        self._blocked_xy = np.stack([self.origin[0] + (ix + 0.5) * self.resolution,
                                     self.origin[1] + (iy + 0.5) * self.resolution], axis=1)
        self.bounds = (self.origin[0], self.origin[1],
                       self.origin[0] + self.width * self.resolution,
                       self.origin[1] + self.height * self.resolution)

    def world_to_cell(self, x, y):
        return (int(math.floor((x - self.origin[0]) / self.resolution)),
                int(math.floor((y - self.origin[1]) / self.resolution)))

    def is_free(self, x, y):
        ix, iy = self.world_to_cell(x, y)
        if not (0 <= ix < self.width and 0 <= iy < self.height):
            return False
        return bool(self.free[iy, ix])

    def clearance(self, pts):
        """Distance (m) from each point to the nearest blocked (occupied/unknown) cell centre."""
        pts = np.atleast_2d(np.asarray(pts, dtype=np.float64))
        out = np.empty(len(pts))
        for k in range(0, len(pts), 256):
            d = pts[k:k + 256, None, :] - self._blocked_xy[None, :, :]
            out[k:k + 256] = np.sqrt((d ** 2).sum(-1)).min(1) - self.resolution / 2.0
        return out

    def segment_clear(self, a, b, min_clearance, step=0.02):
        n = max(2, int(math.ceil(math.dist(a, b) / step)) + 1)
        t = np.linspace(0.0, 1.0, n)[:, None]
        pts = np.asarray(a)[None, :] * (1 - t) + np.asarray(b)[None, :] * t
        for x, y in pts:
            if not self.is_free(x, y):
                return False
        return bool(self.clearance(pts).min() >= min_clearance)


class Lattice:
    """4-connected planning graph over corridor centres."""

    def __init__(self, omap, pitch=0.5, min_clearance=0.154, offset=None, refine=0.08):
        self.omap, self.pitch, self.min_clearance, self.refine = omap, pitch, min_clearance, refine
        if offset is None:
            offset = self._auto_offset()
        self.offset = offset
        x0, y0, x1, y1 = omap.bounds
        self.ix0 = int(math.floor((x0 - offset[0]) / pitch))
        self.iy0 = int(math.floor((y0 - offset[1]) / pitch))
        self.nx = int(math.ceil((x1 - offset[0]) / pitch)) - self.ix0
        self.ny = int(math.ceil((y1 - offset[1]) / pitch)) - self.iy0
        self.pos = {}
        self.valid = np.zeros((self.ny, self.nx), dtype=bool)
        d = np.arange(-refine, refine + 1e-9, 0.01) if refine > 0 else np.array([0.0])
        grid = np.array([(a, b) for a in d for b in d])
        for j in range(self.ny):
            for i in range(self.nx):
                nx_, ny_ = self.nominal(i, j)
                pts = grid + (nx_, ny_)
                free = np.array([omap.is_free(x, y) for x, y in pts])
                if not free.any():
                    self.pos[(i, j)] = (nx_, ny_)
                    continue
                c = np.where(free, omap.clearance(pts), -1.0)
                score = np.minimum(c, 0.22) - 0.05 * np.hypot(grid[:, 0], grid[:, 1])
                k = int(np.argmax(score))
                self.pos[(i, j)] = (float(pts[k, 0]), float(pts[k, 1]))
                self.valid[j, i] = bool(free[k]) and c[k] >= min_clearance
        # edges: mask[j, i, a] True if action a from (i,j) is a collision-free continuous move
        self.edges = np.zeros((self.ny, self.nx, 4), dtype=bool)
        for j in range(self.ny):
            for i in range(self.nx):
                if not self.valid[j, i]:
                    continue
                for a, (di, dj) in enumerate(ACTIONS):
                    k, l = i + di, j + dj
                    if 0 <= k < self.nx and 0 <= l < self.ny and self.valid[l, k]:
                        self.edges[j, i, a] = omap.segment_clear(self.cell_centre(i, j),
                                                                 self.cell_centre(k, l), min_clearance)
        # keep only the largest connected component (unreachable islands are invalid)
        comp = self._largest_component()
        self.valid &= comp
        self.edges &= comp[:, :, None]
        self.cells = [(i, j) for j in range(self.ny) for i in range(self.nx) if self.valid[j, i]]
        self.index = {c: k for k, c in enumerate(self.cells)}

    def _auto_offset(self):
        best, best_off = -1.0, (0.0, 0.0)
        step = 0.02
        for ox in np.arange(0.0, self.pitch, step):
            for oy in np.arange(0.0, self.pitch, step):
                x0, y0, x1, y1 = self.omap.bounds
                xs = np.arange(math.ceil((x0 - ox) / self.pitch), math.floor((x1 - ox) / self.pitch) + 1) * self.pitch + ox
                ys = np.arange(math.ceil((y0 - oy) / self.pitch), math.floor((y1 - oy) / self.pitch) + 1) * self.pitch + oy
                pts = np.array([(x, y) for x in xs for y in ys if self.omap.is_free(x, y)])
                if len(pts) == 0:
                    continue
                c = self.omap.clearance(pts)
                score = np.minimum(c, 0.25).sum()
                if score > best:
                    best, best_off = score, (float(ox), float(oy))
        return best_off

    def _largest_component(self):
        seen = np.zeros_like(self.valid)
        best = None
        for j in range(self.ny):
            for i in range(self.nx):
                if self.valid[j, i] and not seen[j, i]:
                    comp = np.zeros_like(self.valid)
                    stack = [(i, j)]
                    seen[j, i] = comp[j, i] = True
                    while stack:
                        ci, cj = stack.pop()
                        for a, (di, dj) in enumerate(ACTIONS):
                            if self.edges[cj, ci, a]:
                                k, l = ci + di, cj + dj
                                if not seen[l, k]:
                                    seen[l, k] = comp[l, k] = True
                                    stack.append((k, l))
                    if best is None or comp.sum() > best.sum():
                        best = comp
        return best if best is not None else np.zeros_like(self.valid)

    def nominal(self, i, j):
        return ((self.ix0 + i) * self.pitch + self.offset[0], (self.iy0 + j) * self.pitch + self.offset[1])

    def cell_centre(self, i, j):
        """Refined node position (map frame)."""
        return self.pos.get((i, j), self.nominal(i, j))

    def nearest_cells(self, x, y):
        """Valid cells sorted by Euclidean distance from (x, y)."""
        return sorted(self.cells, key=lambda c: math.dist(self.cell_centre(*c), (x, y)))

    def signature(self):
        h = hashlib.sha1()
        h.update(np.ascontiguousarray(self.valid).tobytes())
        h.update(np.ascontiguousarray(self.edges).tobytes())
        h.update(repr((round(self.offset[0], 3), round(self.offset[1], 3), self.pitch, self.refine)).encode())
        h.update(repr(sorted((k, (round(v[0], 3), round(v[1], 3))) for k, v in self.pos.items())).encode())
        return h.hexdigest()[:16]

    def ascii(self):
        lines = []
        for j in range(self.ny - 1, -1, -1):
            row = ""
            for i in range(self.nx):
                row += "o" if self.valid[j, i] else "."
                row += "-" if self.edges[j, i, 0] else " "
            lines.append(row)
            lines.append("".join(("| " if self.edges[j, i, 3] else "  ") for i in range(self.nx)))
        return "\n".join(lines)
