"""Deep Q-Network components: MLP Q-network, replay buffer, agent with target network."""
import random
from collections import deque

import numpy as np
import torch
import torch.nn as nn


class QNetwork(nn.Module):
    def __init__(self, state_dim, n_actions, hidden=(256, 256)):
        super().__init__()
        layers, d = [], state_dim
        for h in hidden:
            layers += [nn.Linear(d, h), nn.ReLU()]
            d = h
        layers.append(nn.Linear(d, n_actions))
        self.net = nn.Sequential(*layers)

    def forward(self, x):
        return self.net(x)


class ReplayBuffer:
    def __init__(self, capacity, seed):
        self.buf = deque(maxlen=capacity)
        self.rng = random.Random(seed)

    def push(self, s, a, r, s2, done):
        self.buf.append((s, a, r, s2, done))

    def sample(self, n):
        batch = self.rng.sample(self.buf, n)
        s, a, r, s2, d = zip(*batch)
        return (torch.from_numpy(np.stack(s)), torch.tensor(a, dtype=torch.int64),
                torch.tensor(r, dtype=torch.float32), torch.from_numpy(np.stack(s2)),
                torch.tensor(d, dtype=torch.float32))

    def __len__(self):
        return len(self.buf)


class DQNAgent:
    def __init__(self, state_dim, n_actions, hidden, lr, gamma, double_dqn=True):
        self.q = QNetwork(state_dim, n_actions, hidden)
        self.target = QNetwork(state_dim, n_actions, hidden)
        self.target.load_state_dict(self.q.state_dict())
        self.target.eval()
        self.opt = torch.optim.Adam(self.q.parameters(), lr=lr)
        self.gamma, self.n_actions, self.double_dqn = gamma, n_actions, double_dqn
        self.loss_fn = nn.SmoothL1Loss()

    def act(self, state, epsilon, rng):
        if rng.random() < epsilon:
            return rng.randrange(self.n_actions)
        with torch.no_grad():
            return int(self.q(torch.from_numpy(state)[None]).argmax(1).item())

    def greedy(self, state):
        with torch.no_grad():
            return int(self.q(torch.from_numpy(state)[None]).argmax(1).item())

    def update(self, batch):
        s, a, r, s2, d = batch
        q = self.q(s).gather(1, a[:, None]).squeeze(1)
        with torch.no_grad():
            if self.double_dqn:
                a2 = self.q(s2).argmax(1, keepdim=True)
                q2 = self.target(s2).gather(1, a2).squeeze(1)
            else:
                q2 = self.target(s2).max(1).values
            y = r + self.gamma * (1.0 - d) * q2  # Bellman target
        loss = self.loss_fn(q, y)
        self.opt.zero_grad()
        loss.backward()
        nn.utils.clip_grad_norm_(self.q.parameters(), 10.0)
        self.opt.step()
        return float(loss.item())

    def sync_target(self):
        self.target.load_state_dict(self.q.state_dict())
