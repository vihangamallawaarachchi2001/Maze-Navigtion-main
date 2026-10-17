"""Goal-conditioned grid environment for DQN route planning.

State  : [one-hot agent cell | one-hot goal cell | 4 local move-validity flags | normalised agent xy, goal xy]
Actions: 4 cardinal moves (ACTIONS in grid_map).
Reward : +100 goal, -100 invalid move (collision, terminal), -1 per step,
         + shaping_scale * (previous Euclidean goal distance - new distance) [cells].
Episode ends on goal, collision or max_steps (timeout).
"""
import math
import random

import numpy as np

from .grid_map import ACTIONS


class GridRouteEnv:
    def __init__(self, lattice, max_steps=60, r_goal=100.0, r_collision=-100.0, r_step=-1.0,
                 shaping_scale=1.0):
        self.lat = lattice
        self.n = len(lattice.cells)
        self.max_steps, self.r_goal, self.r_collision, self.r_step = max_steps, r_goal, r_collision, r_step
        self.shaping_scale = shaping_scale
        self.state_dim = 2 * self.n + 4 + 4
        self.n_actions = len(ACTIONS)
        self.agent = self.goal = None
        self.steps = 0

    # ---- encoding -------------------------------------------------------------
    def encode(self, agent, goal):
        s = np.zeros(self.state_dim, dtype=np.float32)
        s[self.lat.index[agent]] = 1.0
        s[self.n + self.lat.index[goal]] = 1.0
        i, j = agent
        s[2 * self.n:2 * self.n + 4] = self.lat.edges[j, i].astype(np.float32)
        s[2 * self.n + 4:] = [i / self.lat.nx, j / self.lat.ny, goal[0] / self.lat.nx, goal[1] / self.lat.ny]
        return s

    # ---- dynamics ---------------------------------------------------------------
    def reset(self, start, goal):
        assert start in self.lat.index and goal in self.lat.index and start != goal
        self.agent, self.goal, self.steps = start, goal, 0
        return self.encode(start, goal)

    def step(self, action):
        self.steps += 1
        i, j = self.agent
        if not self.lat.edges[j, i, action]:
            return self.encode(self.agent, self.goal), self.r_collision, True, {"event": "collision"}
        di, dj = ACTIONS[action]
        before = math.dist(self.agent, self.goal)
        self.agent = (i + di, j + dj)
        after = math.dist(self.agent, self.goal)
        r = self.r_step + self.shaping_scale * (before - after)
        if self.agent == self.goal:
            return self.encode(self.agent, self.goal), r + self.r_goal, True, {"event": "goal"}
        if self.steps >= self.max_steps:
            return self.encode(self.agent, self.goal), r, True, {"event": "timeout"}
        return self.encode(self.agent, self.goal), r, False, {"event": None}


def all_pairs(lattice):
    return [(a, b) for a in lattice.cells for b in lattice.cells if a != b]


def split_pairs(pairs, eval_fraction, seed):
    rng = random.Random(seed)
    pairs = list(pairs)
    rng.shuffle(pairs)
    k = int(round(len(pairs) * eval_fraction))
    return pairs[k:], pairs[:k]


def bfs_length(lattice, start, goal):
    """Reference shortest path (in moves) -- used ONLY for evaluation metrics, never for planning."""
    from collections import deque
    q, seen = deque([(start, 0)]), {start}
    while q:
        (i, j), d = q.popleft()
        if (i, j) == goal:
            return d
        for a, (di, dj) in enumerate(ACTIONS):
            if lattice.edges[j, i, a]:
                n = (i + di, j + dj)
                if n not in seen:
                    seen.add(n)
                    q.append((n, d + 1))
    return None
