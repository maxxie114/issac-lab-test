"""Robot fleet policy — controls behavior parameters."""

from dataclasses import dataclass, field, asdict
from typing import Optional
import copy


DEFAULT_POLICY = {
    "speed": 0.3,                   # m/s linear speed (0.1 - 0.5)
    "safety_margin": 0.8,           # meters, min distance to other robots
    "congestion_response": "wait",  # "wait" | "reroute" | "slow_down"
    "task_selection": "nearest",    # "nearest" | "oldest" | "random"
    "cooperation_mode": "independent",  # "independent" | "cooperative"
    "zone_preference": "balanced",  # "north" | "south" | "east" | "west" | "balanced"
    "max_concurrent_tasks": 1,      # tasks per robot
}


@dataclass
class Policy:
    speed: float = 0.3
    safety_margin: float = 0.8
    congestion_response: str = "wait"
    task_selection: str = "nearest"
    cooperation_mode: str = "independent"
    zone_preference: str = "balanced"
    max_concurrent_tasks: int = 1

    def update(self, params: dict):
        """Update policy from a dict (e.g. from Gemini response)."""
        for key, value in params.items():
            if hasattr(self, key):
                # Clamp numeric values
                if key == "speed":
                    value = max(0.05, min(0.5, float(value)))
                elif key == "safety_margin":
                    value = max(0.3, min(3.0, float(value)))
                elif key == "max_concurrent_tasks":
                    value = max(1, min(3, int(value)))
                setattr(self, key, value)

    def to_dict(self):
        return asdict(self)

    def clone(self):
        return copy.deepcopy(self)
