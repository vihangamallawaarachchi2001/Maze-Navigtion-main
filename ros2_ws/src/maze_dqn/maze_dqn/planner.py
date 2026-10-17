"""Inference-only DQN route planner (no exploration, no learning).

plan(start_xy, start_yaw, goal_xy, goal_yaw) returns a dense list of (x, y, yaw) map-frame
waypoints or raises PlanningError. The learned network alone chooses the cell sequence;
there is no classical fallback. Every segment, including the connectors from the actual
robot pose to the first lattice node and from the last node to the clicked goal, is
checked for clearance in the occupancy map before the path is returned.
"""
import math

import torch

from .dqn import QNetwork
from .grid_env import GridRouteEnv
from .grid_map import Lattice, OccupancyMap


class PlanningError(RuntimeError):
    pass


class DQNPlanner:
    def __init__(self, checkpoint_path, map_yaml=None, connector_clearance=0.13):
        ck = torch.load(checkpoint_path, map_location="cpu", weights_only=False)
        self.omap = OccupancyMap(map_yaml or ck["map_yaml"])
        lc = ck["lattice"]
        self.lat = Lattice(self.omap, pitch=lc["pitch"], min_clearance=lc["min_clearance"],
                           offset=tuple(lc["offset"]), refine=lc.get("refine", 0.0))
        if self.lat.signature() != lc["signature"]:
            raise PlanningError("lattice built from this map differs from the training lattice "
                                f"({self.lat.signature()} != {lc['signature']}); retrain on this map")
        self.env = GridRouteEnv(self.lat, max_steps=ck["max_steps"])
        self.q = QNetwork(ck["state_dim"], ck["n_actions"], tuple(ck["hidden"]))
        self.q.load_state_dict(ck["state_dict"])
        self.q.eval()
        self.connector_clearance = connector_clearance

    # -- learned route ---------------------------------------------------------
    def learned_route(self, start_cell, goal_cell):
        s = self.env.reset(start_cell, goal_cell)
        route, visited = [start_cell], {start_cell}
        for _ in range(self.env.max_steps):
            with torch.no_grad():
                a = int(self.q(torch.from_numpy(s)[None]).argmax(1).item())
            s, _, done, info = self.env.step(a)
            if info["event"] == "collision":
                raise PlanningError(f"DQN chose blocked move {a} at {route[-1]}")
            route.append(self.env.agent)
            if info["event"] == "goal":
                return route
            if self.env.agent in visited:
                raise PlanningError(f"DQN route loops at cell {self.env.agent}")
            visited.add(self.env.agent)
        raise PlanningError("DQN route timed out")

    def _attach(self, x, y):
        """Nearest lattice node reachable from (x, y) by a clear straight connector."""
        for c in self.lat.nearest_cells(x, y)[:6]:
            if self.omap.segment_clear((x, y), self.lat.cell_centre(*c), self.connector_clearance):
                return c
        raise PlanningError(f"no clear connector from ({x:.2f},{y:.2f}) to the lattice")

    def plan(self, start_xy, goal_xy, goal_yaw=None, spacing=0.05):
        gx, gy = goal_xy
        if not self.omap.is_free(gx, gy):
            raise PlanningError("goal is not in known free space")
        if self.omap.clearance([(gx, gy)])[0] < self.connector_clearance:
            raise PlanningError("goal is too close to an obstacle for the robot footprint")
        sc, gc = self._attach(*start_xy), self._attach(gx, gy)
        cells = [sc] if sc == gc else self.learned_route(sc, gc)
        pts = [tuple(start_xy)] + [self.lat.cell_centre(*c) for c in cells] + [(gx, gy)]
        # drop the first lattice node if the robot is already past it towards the second one
        if len(pts) > 3 and self._between(pts[0], pts[1], pts[2]) and \
                self.omap.segment_clear(pts[0], pts[2], self.connector_clearance):
            pts.pop(1)
        if len(pts) > 2 and self._between(pts[-1], pts[-2], pts[-3]) and \
                self.omap.segment_clear(pts[-3], pts[-1], self.connector_clearance):
            pts.pop(-2)
        for a, b in zip(pts, pts[1:]):
            if not self.omap.segment_clear(a, b, self.connector_clearance):
                raise PlanningError(f"segment {a}->{b} is not collision-free")
        dense = []
        for a, b in zip(pts, pts[1:]):
            d = math.dist(a, b)
            if d < 1e-6:
                continue
            yaw = math.atan2(b[1] - a[1], b[0] - a[0])
            n = max(1, int(math.ceil(d / spacing)))
            for k in range(n):
                t = k / n
                dense.append((a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1]), yaw))
        final_yaw = goal_yaw if goal_yaw is not None else (dense[-1][2] if dense else 0.0)
        dense.append((gx, gy, final_yaw))
        return dense, cells

    @staticmethod
    def _between(p, a, b):
        """True if projection of p onto segment a->b lies inside it (p is 'past' a)."""
        ax, ay = a
        bx, by = b
        t = ((p[0] - ax) * (bx - ax) + (p[1] - ay) * (by - ay)) / max(1e-9, (bx - ax) ** 2 + (by - ay) ** 2)
        return 0.0 < t < 1.0

    @staticmethod
    def path_length(points):
        return float(sum(math.dist(a[:2], b[:2]) for a, b in zip(points, points[1:])))


