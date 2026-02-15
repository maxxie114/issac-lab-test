#!/usr/bin/env python3
"""Standalone web server for Vultr deployment (no Isaac Sim / no GPU).

Runs the web dashboard with Gemini policy chat and a lightweight
simulated fleet state for demo purposes when Isaac Sim isn't available.

This is the Vultr deployment mode — Isaac Sim runs separately on a
GPU machine and pushes state here, or this runs a simplified sim.
"""

import asyncio
import json
import os
import random
import math
import time
import threading
import numpy as np
import uvicorn
from api.server import app, set_fleet_manager

# Minimal in-memory fleet simulation for standalone mode
from simulation.policy import Policy


class StandaloneRobot:
    def __init__(self, rid, x, y):
        self.id = rid
        self.x = x
        self.y = y
        self.state = "idle"
        self.carrying_item = False
        self.deliveries = 0
        self.target = None
        self.task_id = None
        self.speed = 0.3

    def to_dict(self):
        return {
            "id": self.id,
            "x": self.x,
            "y": self.y,
            "state": self.state,
            "carrying_item": self.carrying_item,
            "deliveries": self.deliveries,
            "target": self.target,
            "task_id": self.task_id,
        }


class StandaloneFleetManager:
    """Lightweight fleet simulator that runs without Isaac Sim."""

    PICKUP_ZONES = [[-8, -6], [-8, 0], [-8, 6], [-3, -6], [-3, 6]]
    DROPOFF_ZONES = [[12, -7], [12, -2], [12, 3], [12, 7]]

    def __init__(self, num_robots=4):
        self.policy = Policy()
        self.robots = []
        self.total_deliveries = 0
        self.total_steps = 0
        self.collisions_avoided = 0
        self.pending_tasks = 8
        self.num_robots = num_robots
        self.task_counter = 0
        self.paused = True

        spawns = [[-12, -8], [-12, -3], [-12, 2], [-12, 7]]
        for i in range(min(num_robots, len(spawns))):
            self.robots.append(StandaloneRobot(i, spawns[i][0], spawns[i][1]))

    def step(self):
        if self.paused:
            return
        self.total_steps += 1
        for r in self.robots:
            if r.state == "idle" and self.pending_tasks > 0:
                pickup = random.choice(self.PICKUP_ZONES)
                r.target = pickup[:]
                r.state = "moving_to_pickup"
                r.task_id = self.task_counter
                self.task_counter += 1
                self.pending_tasks -= 1

            if r.target:
                dx = r.target[0] - r.x
                dy = r.target[1] - r.y
                dist = math.sqrt(dx * dx + dy * dy)
                if dist < 0.3:
                    if r.state == "moving_to_pickup":
                        r.carrying_item = True
                        r.state = "moving_to_dropoff"
                        r.target = random.choice(self.DROPOFF_ZONES)[:]
                    elif r.state == "moving_to_dropoff":
                        r.carrying_item = False
                        r.state = "idle"
                        r.deliveries += 1
                        self.total_deliveries += 1
                        r.target = None
                        r.task_id = None
                        self.pending_tasks += 1
                else:
                    speed = self.policy.speed * 0.5
                    r.x += (dx / dist) * speed
                    r.y += (dy / dist) * speed

            # Check collision avoidance
            for other in self.robots:
                if other.id == r.id:
                    continue
                d = math.sqrt((r.x - other.x) ** 2 + (r.y - other.y) ** 2)
                if d < self.policy.safety_margin and d > 0:
                    self.collisions_avoided += 1

    def unstuck_robot(self, robot_id):
        if robot_id < 0 or robot_id >= len(self.robots):
            return False
        r = self.robots[robot_id]
        r.state = "idle"
        r.carrying_item = False
        r.target = None
        r.task_id = None
        self.pending_tasks += 1
        spawns = [[-12, -8], [-12, -3], [-12, 2], [-12, 7]]
        r.x = spawns[robot_id % len(spawns)][0]
        r.y = spawns[robot_id % len(spawns)][1]
        return True

    def unstuck_all(self):
        unstuck = []
        for r in self.robots:
            if r.state != "idle":
                self.unstuck_robot(r.id)
                unstuck.append(r.id)
        return unstuck

    def add_robot(self, x=None, y=None):
        i = len(self.robots)
        if x is None:
            x = -12
        if y is None:
            y = -8 + i * 5
        self.robots.append(StandaloneRobot(i, x, y))
        self.num_robots = len(self.robots)
        return i

    def update_policy(self, params):
        old = self.policy.to_dict()
        self.policy.update(params)
        new = self.policy.to_dict()
        return {k: v for k, v in new.items() if old.get(k) != v}

    def get_state(self):
        return {
            "robots": [r.to_dict() for r in self.robots],
            "policy": self.policy.to_dict(),
            "paused": self.paused,
            "stats": {
                "total_deliveries": self.total_deliveries,
                "total_steps": self.total_steps,
                "collisions_avoided": self.collisions_avoided,
                "pending_tasks": self.pending_tasks,
                "num_robots": self.num_robots,
            },
        }


def run_simulation(fleet):
    """Background thread running the lightweight simulation."""
    while True:
        fleet.step()
        time.sleep(0.05)  # ~20 steps/sec


def main():
    port = int(os.environ.get("PORT", 8000))
    num_robots = int(os.environ.get("NUM_ROBOTS", 4))

    fleet = StandaloneFleetManager(num_robots=num_robots)
    set_fleet_manager(fleet)

    # Run simulation in background
    sim_thread = threading.Thread(target=run_simulation, args=(fleet,), daemon=True)
    sim_thread.start()

    print(f"[GeminiFleet] Standalone mode (no Isaac Sim)")
    print(f"[GeminiFleet] Dashboard: http://localhost:{port}")

    uvicorn.run(app, host="0.0.0.0", port=port, log_level="info")


if __name__ == "__main__":
    main()
