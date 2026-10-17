#!/usr/bin/env python3
"""Train the goal-conditioned DQN route planner.

usage: python -m maze_dqn.train --config config/dqn.yaml --map <map.yaml> --out <dir>
Writes: <out>/dqn_policy.pt, train_log.csv, hyperparameters.yaml, split.yaml, lattice.txt
"""
import argparse
import csv
import hashlib
import os
import random
import time

import numpy as np
import torch
import yaml

from .dqn import DQNAgent, ReplayBuffer
from .grid_env import GridRouteEnv, all_pairs, split_pairs
from .grid_map import Lattice, OccupancyMap


def file_sha1(path):
    with open(path, "rb") as f:
        return hashlib.sha1(f.read()).hexdigest()


def greedy_rollout(agent, env, start, goal):
    s = env.reset(start, goal)
    visited, path = {start}, [start]
    for _ in range(env.max_steps):
        s, _, done, info = env.step(agent.greedy(s))
        if info["event"] == "collision":
            return path, "collision"
        path.append(env.agent)
        if info["event"] == "goal":
            return path, "success"
        if env.agent in visited:
            return path, "loop"
        visited.add(env.agent)
        if done:
            break
    return path, "timeout"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", required=True)
    ap.add_argument("--map", required=True)
    ap.add_argument("--out", required=True)
    args = ap.parse_args()
    with open(args.config) as f:
        cfg = yaml.safe_load(f)
    os.makedirs(args.out, exist_ok=True)

    seed = int(cfg["seed"])
    random.seed(seed)
    np.random.seed(seed)
    torch.manual_seed(seed)
    torch.set_num_threads(int(cfg.get("torch_threads", 2)))
    rng = random.Random(seed)

    omap = OccupancyMap(args.map)
    lat = Lattice(omap, pitch=cfg["lattice_pitch_m"], min_clearance=cfg["min_clearance_m"],
                  refine=cfg.get("node_refine_m", 0.0))
    with open(os.path.join(args.out, "lattice.txt"), "w") as f:
        f.write(f"map={omap.yaml_path}\noffset={lat.offset}\nnodes={len(lat.cells)}\n"
                f"signature={lat.signature()}\n{lat.ascii()}\n")
    print(f"lattice: {len(lat.cells)} nodes, offset {lat.offset}, signature {lat.signature()}")

    env = GridRouteEnv(lat, max_steps=cfg["max_steps"], r_goal=cfg["reward_goal"],
                       r_collision=cfg["reward_collision"], r_step=cfg["reward_step"],
                       shaping_scale=cfg["shaping_scale"])
    train_pairs, eval_pairs = split_pairs(all_pairs(lat), cfg["eval_fraction"], seed)
    with open(os.path.join(args.out, "split.yaml"), "w") as f:
        yaml.safe_dump({"train": [list(map(list, p)) for p in train_pairs],
                        "eval": [list(map(list, p)) for p in eval_pairs]}, f)

    agent = DQNAgent(env.state_dim, env.n_actions, tuple(cfg["hidden"]), cfg["lr"], cfg["gamma"],
                     cfg.get("double_dqn", True))
    buf = ReplayBuffer(cfg["buffer_size"], seed)
    episodes = cfg["episodes"]
    eps_start, eps_end = cfg["epsilon_start"], cfg["epsilon_end"]
    eps_decay_eps = int(episodes * cfg["epsilon_decay_fraction"])

    log = open(os.path.join(args.out, "train_log.csv"), "w", newline="")
    w = csv.writer(log)
    w.writerow(["episode", "epsilon", "reward", "steps", "outcome", "mean_loss", "global_step",
                "wall_s", "eval_train_success", "eval_heldout_success"])
    t0, gstep = time.time(), 0
    for ep in range(1, episodes + 1):
        eps = max(eps_end, eps_start - (eps_start - eps_end) * ep / max(1, eps_decay_eps))
        start, goal = rng.choice(train_pairs)
        s = env.reset(start, goal)
        total, losses, outcome = 0.0, [], "timeout"
        while True:
            a = agent.act(s, eps, rng)
            s2, r, done, info = env.step(a)
            # timeout is not a true terminal state: bootstrap through it
            buf.push(s, a, r, s2, float(done and info["event"] != "timeout"))
            s, total, gstep = s2, total + r, gstep + 1
            if len(buf) >= max(cfg["warmup_steps"], cfg["batch_size"]) and gstep % cfg["train_every"] == 0:
                losses.append(agent.update(buf.sample(cfg["batch_size"])))
            if gstep % cfg["target_sync_steps"] == 0:
                agent.sync_target()
            if done:
                outcome = info["event"]
                break
        ev_tr = ev_ho = ""
        if ep % cfg["eval_every"] == 0 or ep == episodes:
            sample_tr = rng.sample(train_pairs, min(200, len(train_pairs)))
            ev_tr = np.mean([greedy_rollout(agent, env, *p)[1] == "success" for p in sample_tr])
            ev_ho = np.mean([greedy_rollout(agent, env, *p)[1] == "success" for p in eval_pairs])
            print(f"ep {ep:6d} eps {eps:.3f} greedy success train {ev_tr:.3f} held-out {ev_ho:.3f} "
                  f"({time.time() - t0:.0f}s)", flush=True)
        w.writerow([ep, f"{eps:.4f}", f"{total:.2f}", env.steps, outcome,
                    f"{np.mean(losses):.5f}" if losses else "", gstep, f"{time.time() - t0:.1f}", ev_tr, ev_ho])
    log.close()
    train_time = time.time() - t0

    ckpt = {
        "state_dict": agent.q.state_dict(),
        "hidden": list(cfg["hidden"]),
        "state_dim": env.state_dim,
        "n_actions": env.n_actions,
        "lattice": {"pitch": lat.pitch, "min_clearance": lat.min_clearance, "offset": list(lat.offset),
                    "refine": lat.refine,
                    "signature": lat.signature(), "cells": [list(c) for c in lat.cells]},
        "map_yaml": omap.yaml_path,
        "map_yaml_sha1": file_sha1(omap.yaml_path),
        "max_steps": cfg["max_steps"],
        "config": cfg,
        "train_time_s": train_time,
        "global_steps": gstep,
    }
    torch.save(ckpt, os.path.join(args.out, "dqn_policy.pt"))
    with open(os.path.join(args.out, "hyperparameters.yaml"), "w") as f:
        yaml.safe_dump({**cfg, "train_time_s": round(train_time, 1), "global_steps": gstep,
                        "lattice_signature": lat.signature(), "map_yaml": omap.yaml_path,
                        "torch_version": str(torch.__version__)}, f)
    print(f"saved checkpoint after {train_time:.0f}s, {gstep} steps")


if __name__ == "__main__":
    main()
