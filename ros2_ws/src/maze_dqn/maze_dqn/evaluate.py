#!/usr/bin/env python3
"""Greedy (epsilon = 0, no learning) evaluation of a saved DQN checkpoint on the grid.

usage: python -m maze_dqn.evaluate --ckpt <dir>/dqn_policy.pt --split <dir>/split.yaml --out <dir>
Writes grid_eval.csv (one row per start/goal pair), grid_eval_summary.yaml and plots.
"""
import argparse
import csv
import os
import time

import yaml

from .grid_env import bfs_length
from .planner import DQNPlanner, PlanningError


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--ckpt", required=True)
    ap.add_argument("--split", required=True)
    ap.add_argument("--out", required=True)
    a = ap.parse_args()
    planner = DQNPlanner(a.ckpt)
    with open(a.split) as f:
        split = yaml.safe_load(f)
    rows, summary = [], {}
    for name in ("train", "eval"):
        pairs = [(tuple(p[0]), tuple(p[1])) for p in split[name]]
        ok = opt = 0
        fails = {}
        ratios, times = [], []
        for s, g in pairs:
            t0 = time.perf_counter()
            try:
                route = planner.learned_route(s, g)
                status = "success"
            except PlanningError as e:
                route = []
                status = "loop" if "loop" in str(e) else ("collision" if "blocked" in str(e) else "timeout")
            dt = time.perf_counter() - t0
            times.append(dt)
            ref = bfs_length(planner.lat, s, g)
            moves = len(route) - 1 if route else ""
            if status == "success":
                ok += 1
                opt += int(moves == ref)
                ratios.append(ref / moves)
            else:
                fails[status] = fails.get(status, 0) + 1
            rows.append([name, *s, *g, status, moves, ref, f"{dt * 1000:.3f}"])
        n = len(pairs)
        summary[name] = {"pairs": n, "success": ok, "success_rate": round(ok / n, 4),
                         "optimal_length_rate": round(opt / n, 4),
                         "mean_ref_over_learned_length_successes": round(sum(ratios) / max(1, len(ratios)), 4),
                         "failures": fails, "mean_inference_ms": round(1000 * sum(times) / n, 3)}
    os.makedirs(a.out, exist_ok=True)
    with open(os.path.join(a.out, "grid_eval.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["split", "start_i", "start_j", "goal_i", "goal_j", "status", "moves", "bfs_moves", "inference_ms"])
        w.writerows(rows)
    with open(os.path.join(a.out, "grid_eval_summary.yaml"), "w") as f:
        yaml.safe_dump(summary, f, sort_keys=False)
    print(yaml.safe_dump(summary, sort_keys=False))


if __name__ == "__main__":
    main()
