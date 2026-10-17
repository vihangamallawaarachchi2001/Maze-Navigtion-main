#!/usr/bin/env python3
"""Summarise comparison trials and produce report figures.

usage: analyze_results.py --trials results/trials.csv --goals <evaluation_goals.yaml>
                          --map maps/slam/maze_slam.yaml --train results/dqn_training --out results/analysis

Reference shortest path: 8-connected Dijkstra on the SAME SLAM map, restricted to cells whose
clearance is >= 0.154 m (the DQN lattice clearance); start/goal snapped to the nearest such cell.
It is an evaluation reference only (approximation: grid path, 8-connectivity).
"""
import argparse
import csv
import heapq
import math
import os
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
import yaml  # noqa: E402

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "ros2_ws/src/maze_dqn"))
from maze_dqn.grid_map import OccupancyMap  # noqa: E402

COLORS = {"baseline": "#2f6db3", "dqn": "#d9822b"}


def clearance_grid(m):
    iy, ix = np.nonzero(~m.free)
    blocked = np.stack([ix, iy], 1)
    fy, fx = np.nonzero(m.free)
    cl = np.zeros(m.free.shape)
    for k in range(0, len(fx), 2000):
        d = np.sqrt(((np.stack([fx[k:k + 2000], fy[k:k + 2000]], 1)[:, None, :] - blocked[None]) ** 2).sum(-1)).min(1)
        cl[fy[k:k + 2000], fx[k:k + 2000]] = d * m.resolution - m.resolution / 2
    return cl


def shortest(m, ok, start, goal):
    def cell(p):
        c = (int((p[0] - m.origin[0]) / m.resolution), int((p[1] - m.origin[1]) / m.resolution))
        if ok[c[1], c[0]]:
            return c
        ys, xs = np.nonzero(ok)
        k = np.argmin((xs - c[0]) ** 2 + (ys - c[1]) ** 2)
        return int(xs[k]), int(ys[k])
    s, g = cell(start), cell(goal)
    dist, pq = {s: 0.0}, [(0.0, s)]
    while pq:
        d, c = heapq.heappop(pq)
        if c == g:
            return d * m.resolution
        if d > dist[c]:
            continue
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                if dx == dy == 0:
                    continue
                n = (c[0] + dx, c[1] + dy)
                if 0 <= n[0] < m.width and 0 <= n[1] < m.height and ok[n[1], n[0]]:
                    nd = d + math.hypot(dx, dy)
                    if nd < dist.get(n, 1e18):
                        dist[n] = nd
                        heapq.heappush(pq, (nd, n))
    return float("nan")


def fnum(v):
    try:
        return float(v)
    except (TypeError, ValueError):
        return float("nan")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--trials", required=True)
    ap.add_argument("--goals", required=True)
    ap.add_argument("--map", required=True)
    ap.add_argument("--train", default="")
    ap.add_argument("--out", required=True)
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    goals = yaml.safe_load(open(a.goals))["scenarios"]
    rows = list(csv.DictReader(open(a.trials)))
    m = OccupancyMap(a.map)
    ok = clearance_grid(m) >= 0.154

    def scen(r):
        g = (fnum(r["goal_x"]), fnum(r["goal_y"]))
        for s in goals:
            if math.dist(g, s["goal_map"][:2]) < 0.05:
                return s["name"]
        return "other"

    ref_cache = {}
    for r in rows:
        r["scenario"] = scen(r)
        key = (r["scenario"], r["start_x"], r["start_y"])
        if key not in ref_cache:
            ref_cache[key] = shortest(m, ok, (fnum(r["start_x"]), fnum(r["start_y"])),
                                      (fnum(r["goal_x"]), fnum(r["goal_y"])))
        r["reference_shortest_path_m"] = f"{ref_cache[key]:.3f}"
        act = fnum(r["actual_path_m"])
        r["path_efficiency"] = f"{ref_cache[key] / act:.3f}" if act > 0 and r["success"] == "1" else ""
    with open(os.path.join(a.out, "trials_with_reference.csv"), "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)

    names = [s["name"] for s in goals]
    methods = [mm for mm in ("baseline", "dqn") if any(r["method"] == mm for r in rows)]
    summary = []
    for sname in names:
        for meth in methods:
            rs = [r for r in rows if r["scenario"] == sname and r["method"] == meth]
            if not rs:
                continue
            succ = [r for r in rs if r["success"] == "1"]

            def stat(col, src):
                v = np.array([fnum(r[col]) for r in src])
                v = v[np.isfinite(v)]
                return (v.mean(), v.std(ddof=1) if len(v) > 1 else 0.0) if len(v) else (float("nan"), float("nan"))
            summary.append({
                "scenario": sname, "method": meth, "trials": len(rs), "successes": len(succ),
                "success_rate": len(succ) / len(rs),
                "collisions": sum(int(fnum(r["collision_count"]) or 0) for r in rs),
                "near_collisions": sum(int(fnum(r["near_collision_count"]) or 0) for r in rs),
                "timeouts": sum(r["timeout"] == "1" for r in rs),
                "failures": "; ".join(f"{r['action_result']}" for r in rs if r["success"] != "1"),
                "sim_time_s": stat("sim_elapsed_s", succ), "wall_time_s": stat("wall_elapsed_s", succ),
                "path_m": stat("actual_path_m", succ), "planned_m": stat("planned_path_m", rs),
                "planning_s": stat("planning_time_s", rs), "efficiency": stat("path_efficiency", succ),
                "rotation_rad": stat("total_rotation_rad", succ), "w_rms": stat("angular_velocity_rms", succ),
                "true_err_m": stat("true_final_error_m", succ), "ref_m": ref_cache.get((sname, rs[0]["start_x"], rs[0]["start_y"])),
            })
    fmt = lambda t, p=2: f"{t[0]:.{p}f} ± {t[1]:.{p}f}" if np.isfinite(t[0]) else "n/a"  # noqa: E731
    lines = ["| Scenario | Method | Success | Collisions | Near-coll. | Timeouts | Sim time (s) | Wall time (s) | "
             "Travelled (m) | Planned (m) | Ref. shortest (m) | Efficiency | Planning (s) | Σ|Δyaw| (rad) | ω RMS (rad/s) | True final err (m) |",
             "|" + "---|" * 16]
    for s in summary:
        lines.append(f"| {s['scenario']} | {s['method']} | {s['successes']}/{s['trials']} | {s['collisions']} | "
                     f"{s['near_collisions']} | {s['timeouts']} | {fmt(s['sim_time_s'], 1)} | {fmt(s['wall_time_s'], 1)} | "
                     f"{fmt(s['path_m'])} | {fmt(s['planned_m'])} | {s['ref_m']:.2f} | {fmt(s['efficiency'])} | "
                     f"{fmt(s['planning_s'], 3)} | {fmt(s['rotation_rad'])} | {fmt(s['w_rms'])} | {fmt(s['true_err_m'], 3)} |")
    fails = [f"- {s['scenario']} / {s['method']}: {s['failures']}" for s in summary if s["failures"]]
    table = "\n".join(lines) + ("\n\nFailed trials (action result):\n" + "\n".join(fails) if fails else "\n\nNo failed trials.")
    open(os.path.join(a.out, "summary.md"), "w").write(table + "\n")
    print(table)

    # ---- comparison figure
    fig, axes = plt.subplots(1, 3, figsize=(12, 3.6))
    x = np.arange(len(names))
    wdt = 0.38
    for k, (key, label) in enumerate((("sim_time_s", "Simulation travel time (s)"), ("path_m", "Travelled path (m)"),
                                      ("rotation_rad", "Total heading change (rad)"))):
        ax = axes[k]
        for j, meth in enumerate(methods):
            vals = [next((s[key] for s in summary if s["scenario"] == n and s["method"] == meth), (np.nan, 0)) for n in names]
            ax.bar(x + (j - 0.5) * wdt, [v[0] for v in vals], wdt, yerr=[v[1] for v in vals], capsize=3,
                   color=COLORS[meth], label=meth)
        if key == "path_m":
            ax.plot(x, [next(s["ref_m"] for s in summary if s["scenario"] == n) for n in names], "k_", ms=22, mew=2,
                    label="reference shortest")
        ax.set_xticks(x, [n.split("_")[0] for n in names])
        ax.set_title(label, fontsize=10)
        ax.grid(axis="y", alpha=0.3)
        ax.spines[["top", "right"]].set_visible(False)
    axes[1].legend(fontsize=8, frameon=False)
    fig.tight_layout()
    fig.savefig(os.path.join(a.out, "comparison.png"), dpi=150)

    # ---- training curves
    if a.train and os.path.exists(os.path.join(a.train, "train_log.csv")):
        tl = list(csv.DictReader(open(os.path.join(a.train, "train_log.csv"))))
        ep = np.array([int(r["episode"]) for r in tl])
        rew = np.array([float(r["reward"]) for r in tl])
        outcome = np.array([r["outcome"] for r in tl])
        win = 500
        kern = np.ones(win) / win
        fig, axes = plt.subplots(1, 2, figsize=(11, 3.6))
        axes[0].plot(ep[win - 1:], np.convolve(rew, kern, "valid"), color="#2f6db3")
        axes[0].set_title(f"Episode reward ({win}-episode moving mean)", fontsize=10)
        axes[0].set_xlabel("episode")
        for lab, col in (("goal", "#2a9d58"), ("collision", "#c0392b"), ("timeout", "#7f8c8d")):
            axes[1].plot(ep[win - 1:], np.convolve((outcome == lab).astype(float), kern, "valid"), color=col,
                         label=f"{lab} (ε-greedy)")
        ev = [(int(r["episode"]), float(r["eval_train_success"]), float(r["eval_heldout_success"])) for r in tl if r["eval_heldout_success"]]
        axes[1].plot([e[0] for e in ev], [e[1] for e in ev], "o-", color="#2f6db3", ms=3, label="greedy, train pairs")
        axes[1].plot([e[0] for e in ev], [e[2] for e in ev], "s-", color="#d9822b", ms=3, label="greedy, held-out pairs")
        axes[1].set_title("Episode outcome rate / greedy success", fontsize=10)
        axes[1].set_xlabel("episode")
        axes[1].legend(fontsize=7, frameon=False)
        for ax in axes:
            ax.grid(alpha=0.3)
            ax.spines[["top", "right"]].set_visible(False)
        fig.tight_layout()
        fig.savefig(os.path.join(a.out, "dqn_training.png"), dpi=150)


if __name__ == "__main__":
    main()
