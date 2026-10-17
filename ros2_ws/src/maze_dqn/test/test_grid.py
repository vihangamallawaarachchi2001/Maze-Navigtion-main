"""Deterministic tests for map conversion, lattice edges and the environment.

Run: python -m pytest -q test/test_grid.py   (uses the reference geometry map)
"""
import os

import pytest

from maze_dqn.grid_env import GridRouteEnv, bfs_length
from maze_dqn.grid_map import Lattice, OccupancyMap

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "..", ".."))
REF = os.path.join(ROOT, "maps", "reference", "ground_truth_maze.yaml")


@pytest.fixture(scope="module")
def lat():
    return Lattice(OccupancyMap(REF), pitch=0.5, min_clearance=0.154)


def test_image_row_inversion_and_origin():
    m = OccupancyMap(REF)
    assert not m.is_free(-1.5, 0.0)          # outer left wall
    assert not m.is_free(1.0, 0.75)          # upper_right_v (top half of map -> image top rows)
    assert m.is_free(1.25, 1.25)             # top-right cell centre free
    assert m.is_free(-1.25, -1.25)           # bottom-left cell centre free
    assert not m.is_free(-1.25, -1.0)        # bottom_left_bar (bottom half of map)


def test_lattice_matches_maze(lat):
    assert lat.offset == pytest.approx((0.25, 0.25), abs=0.03)
    assert len(lat.cells) == 36


def test_wall_blocks_edge(lat):
    # cells (col,row) with world centres (-1.25,-1.25) and (-1.25,-0.75) are separated by bottom_left_bar
    a = min(lat.cells, key=lambda c: sum((u - v) ** 2 for u, v in zip(lat.cell_centre(*c), (-1.25, -1.25))))
    i, j = a
    assert not lat.edges[j, i, 2]            # +y blocked
    assert lat.edges[j, i, 0]                # +x open


def test_env_collision_terminal_and_reset(lat):
    env = GridRouteEnv(lat, max_steps=5)
    start = min(lat.cells, key=lambda c: sum((u - v) ** 2 for u, v in zip(lat.cell_centre(*c), (-1.25, -1.25))))
    goal = min(lat.cells, key=lambda c: sum((u - v) ** 2 for u, v in zip(lat.cell_centre(*c), (1.25, 1.25))))
    env.reset(start, goal)
    _, r, done, info = env.step(2)           # into the wall
    assert done and info["event"] == "collision" and r == -100.0
    env.reset(start, goal)
    assert env.agent == start and env.steps == 0
    for _ in range(5):
        _, _, done, info = env.step(0 if env.lat.edges[env.agent[1], env.agent[0], 0] else 1)
        if done:
            break
    assert info["event"] in ("timeout", "collision")


def test_all_cells_reachable(lat):
    s = lat.cells[0]
    assert all(bfs_length(lat, s, g) is not None for g in lat.cells)
