#!/usr/bin/env python3
"""Compare a SLAM map with the geometry-derived reference map (both in the world/map frame).

Reports
  * wall alignment: distance from every SLAM-occupied cell to the nearest reference wall cell
    (median, 95th percentile, fraction within 0.05 m / 0.10 m)
  * coverage: fraction of reference free cells inside the maze that SLAM marks free
  * false free: fraction of reference wall cells that SLAM marks free
usage: map_quality.py <slam.yaml> [reference.yaml]
"""
import os
import sys

import numpy as np

sys.path.insert(0, os.path.expanduser("~/maze_testing/ros2_ws/src/maze_dqn"))
from maze_dqn.grid_map import OccupancyMap  # noqa: E402


def cell_centres(m, mask):
    iy, ix = np.nonzero(mask)
    return np.stack([m.origin[0] + (ix + 0.5) * m.resolution, m.origin[1] + (iy + 0.5) * m.resolution], 1)


def lookup(m, pts, grid):
    ix = np.floor((pts[:, 0] - m.origin[0]) / m.resolution).astype(int)
    iy = np.floor((pts[:, 1] - m.origin[1]) / m.resolution).astype(int)
    ok = (ix >= 0) & (iy >= 0) & (ix < m.width) & (iy < m.height)
    out = np.zeros(len(pts), bool)
    out[ok] = grid[iy[ok], ix[ok]]
    return out


def main():
    slam = OccupancyMap(sys.argv[1])
    ref = OccupancyMap(sys.argv[2] if len(sys.argv) > 2 else
                       os.path.expanduser("~/maze_testing/maps/reference/ground_truth_maze.yaml"))
    s_occ = cell_centres(slam, slam.occupied)
    r_occ = cell_centres(ref, ref.occupied)
    d = np.empty(len(s_occ))
    for k in range(0, len(s_occ), 512):
        diff = s_occ[k:k + 512, None, :] - r_occ[None, :, :]
        d[k:k + 512] = np.sqrt((diff ** 2).sum(-1)).min(1)
    inside = ref.free.copy()
    r_free = cell_centres(ref, inside)
    covered = lookup(slam, r_free, slam.free).mean()
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    from maze_geometry import SEGMENTS, point_segment_distance
    centre = np.array([min(point_segment_distance(x, y, *sg[1:]) for sg in SEGMENTS) <= 0.011 for x, y in r_occ])
    false_free = lookup(slam, r_occ[centre], slam.free).mean()
    print(f"SLAM map: {slam.width}x{slam.height} @ {slam.resolution} m, origin {slam.origin}")
    print(f"occupied cells: {len(s_occ)}")
    print(f"wall alignment: median {np.median(d):.3f} m, p95 {np.percentile(d, 95):.3f} m, "
          f"within 0.05 m {np.mean(d <= 0.05) * 100:.1f} %, within 0.10 m {np.mean(d <= 0.10) * 100:.1f} %")
    print(f"coverage of reference free space: {covered * 100:.1f} %")
    print(f"reference wall centre-line cells marked free by SLAM: {false_free * 100:.2f} %")


if __name__ == "__main__":
    main()
