"""Verify learning updates, target isolation and the shipped checkpoint."""
from pathlib import Path
import math

import numpy as np
import pytest
import torch

from maze_dqn.dqn import DQNAgent, ReplayBuffer
from maze_dqn.planner import DQNPlanner, PlanningError

ROOT = Path(__file__).resolve().parents[4]


def test_learning_updates_online_network_and_syncs_target():
    torch.manual_seed(7)
    agent = DQNAgent(8, 4, (16, 16), 0.001, 0.95)
    buf = ReplayBuffer(10, 7)
    for k in range(8):
        state = np.full(8, k / 8, dtype=np.float32)
        buf.push(state, k % 4, 1.0, state + 0.1, float(k == 7))
    before = [p.detach().clone() for p in agent.q.parameters()]
    loss = agent.update(buf.sample(8))
    assert math.isfinite(loss)
    assert any(not torch.equal(a, b) for a, b in zip(before, agent.q.parameters()))
    assert all(torch.equal(a, b) for a, b in zip(before, agent.target.parameters()))
    agent.sync_target()
    assert all(torch.equal(a, b) for a, b in zip(agent.q.parameters(), agent.target.parameters()))


@pytest.fixture(scope='module')
def planner():
    return DQNPlanner(str(ROOT / 'ros2_ws/src/maze_dqn/checkpoints/dqn_policy.pt'),
                      map_yaml=str(ROOT / 'maps/slam/maze_slam.yaml'))


def test_checkpoint_loads_and_plans_three_routes(planner):
    for goal in [(0.25, -1.25), (-1.25, 0.75), (1.25, 0.25)]:
        points, cells = planner.plan((-1.25, -1.25), goal)
        assert points[-1][:2] == goal
        assert len(cells) >= 2
        assert all(planner.omap.segment_clear(a[:2], b[:2], 0.13)
                   for a, b in zip(points, points[1:]))


def test_checkpoint_rejects_wall_goal(planner):
    with pytest.raises(PlanningError):
        planner.plan((-1.25, -1.25), (-1.5, 0.0))
