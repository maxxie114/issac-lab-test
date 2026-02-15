"""Fleet manager — spawns and controls multiple warehouse robots."""

import numpy as np
import random
from dataclasses import dataclass, field
from typing import List, Optional, Tuple
from .policy import Policy


@dataclass
class Task:
    task_id: int
    pickup: np.ndarray
    dropoff: np.ndarray
    assigned_to: Optional[int] = None
    completed: bool = False


@dataclass
class RobotState:
    robot_id: int
    position: np.ndarray = field(default_factory=lambda: np.zeros(3))
    orientation: np.ndarray = field(default_factory=lambda: np.array([1, 0, 0, 0]))
    state: str = "idle"  # idle | moving_to_pickup | moving_to_dropoff | waiting
    current_task: Optional[Task] = None
    deliveries: int = 0
    carrying_item: bool = False
    target: Optional[np.ndarray] = None


# Default spawn positions for robots
SPAWN_POSITIONS = [
    np.array([-12.0, -8.0, 0.0]),
    np.array([-12.0, -3.0, 0.0]),
    np.array([-12.0, 2.0, 0.0]),
    np.array([-12.0, 7.0, 0.0]),
    np.array([-12.0, -5.5, 0.0]),
    np.array([-12.0, 4.5, 0.0]),
]

ROBOT_COLORS = [
    np.array([1.0, 0.85, 0.0]),   # Yellow
    np.array([0.0, 0.9, 0.9]),    # Cyan
    np.array([1.0, 0.3, 0.7]),    # Pink
    np.array([1.0, 0.5, 0.0]),    # Orange
    np.array([0.5, 0.5, 1.0]),    # Periwinkle
    np.array([0.3, 1.0, 0.3]),    # Lime
]


class FleetManager:
    """Manages a fleet of warehouse robots in Isaac Sim."""

    def __init__(self, num_robots: int = 4):
        self.num_robots = min(num_robots, len(SPAWN_POSITIONS))
        self.policy = Policy()
        self.robot_states: List[RobotState] = []
        self.task_queue: List[Task] = []
        self.task_counter = 0
        self.total_deliveries = 0
        self.total_steps = 0
        self.collisions_avoided = 0

        # Isaac Sim objects (populated after spawn)
        self._robots = []
        self._controllers = []
        self._sim_initialized = False

    def spawn_robots(self, world, robot_usd=None):
        """Spawn robots into the Isaac Sim world."""
        from omni.isaac.core.utils.nucleus import get_assets_root_path
        from omni.isaac.wheeled_robots.robots import WheeledRobot
        from omni.isaac.wheeled_robots.controllers.differential_controller import DifferentialController
        from omni.isaac.wheeled_robots.controllers.wheel_base_pose_controller import WheelBasePoseController

        assets_root = get_assets_root_path()
        if robot_usd is None:
            robot_usd = assets_root + "/Isaac/Robots/Jetbot/jetbot.usd"

        for i in range(self.num_robots):
            pos = SPAWN_POSITIONS[i]
            name = f"robot_{i}"

            robot = world.scene.add(
                WheeledRobot(
                    prim_path=f"/World/Robots/{name}",
                    name=name,
                    wheel_dof_names=["left_wheel_joint", "right_wheel_joint"],
                    create_robot=True,
                    usd_path=robot_usd,
                    position=pos,
                )
            )
            self._robots.append(robot)

            controller = WheelBasePoseController(
                name=f"nav_{name}",
                open_loop_wheel_controller=DifferentialController(
                    name=f"diff_{name}",
                    wheel_radius=0.0325,
                    wheel_base=0.1125,
                ),
                is_holonomic=False,
            )
            self._controllers.append(controller)

            self.robot_states.append(
                RobotState(robot_id=i, position=pos.copy())
            )

        self._sim_initialized = True
        print(f"[Fleet] Spawned {self.num_robots} robots")

    def generate_tasks(self, pickup_zones, dropoff_zones, count=5):
        """Generate random delivery tasks."""
        for _ in range(count):
            self.task_counter += 1
            task = Task(
                task_id=self.task_counter,
                pickup=random.choice(pickup_zones).copy(),
                dropoff=random.choice(dropoff_zones).copy(),
            )
            self.task_queue.append(task)
        print(f"[Fleet] Generated {count} tasks, queue size: {len(self.task_queue)}")

    def step(self, pickup_zones, dropoff_zones):
        """Advance one simulation step — assign tasks, navigate robots."""
        if not self._sim_initialized:
            return

        self.total_steps += 1

        # Read current robot poses from simulation
        for i, robot in enumerate(self._robots):
            pos, orient = robot.get_world_pose()
            self.robot_states[i].position = pos
            self.robot_states[i].orientation = orient

        # Assign tasks to idle robots
        for rs in self.robot_states:
            if rs.state == "idle" and self.task_queue:
                task = self._select_task(rs)
                if task:
                    task.assigned_to = rs.robot_id
                    rs.current_task = task
                    rs.state = "moving_to_pickup"
                    rs.target = task.pickup
                    rs.carrying_item = False

        # Navigate each robot
        for i, rs in enumerate(self.robot_states):
            if rs.state == "idle":
                # Stop the robot
                self._robots[i].apply_wheel_actions(
                    self._controllers[i].forward(
                        start_position=rs.position,
                        start_orientation=rs.orientation,
                        goal_position=rs.position,
                    )
                )
                continue

            if rs.target is None:
                continue

            # Check if target reached
            dist = np.linalg.norm(rs.target[:2] - rs.position[:2])
            if dist < 0.3:
                self._on_target_reached(rs, i, pickup_zones, dropoff_zones)
                continue

            # Check for collision risk with other robots
            should_wait = False
            for j, other in enumerate(self.robot_states):
                if i == j:
                    continue
                robot_dist = np.linalg.norm(rs.position[:2] - other.position[:2])
                if robot_dist < self.policy.safety_margin:
                    if self.policy.congestion_response == "wait":
                        should_wait = True
                        self.collisions_avoided += 1
                        break
                    elif self.policy.congestion_response == "slow_down":
                        # Will use reduced speed below
                        pass

            if should_wait:
                rs.state_detail = "waiting"
                # Apply zero velocity
                from omni.isaac.core.articulations import ArticulationAction
                self._robots[i].apply_action(
                    ArticulationAction(joint_velocities=np.array([0.0, 0.0]))
                )
                continue

            # Navigate toward target
            action = self._controllers[i].forward(
                start_position=rs.position,
                start_orientation=rs.orientation,
                goal_position=rs.target,
            )
            self._robots[i].apply_action(action)

        # Replenish task queue
        if len(self.task_queue) < 3:
            self.generate_tasks(pickup_zones, dropoff_zones, count=3)

    def _select_task(self, rs: RobotState) -> Optional[Task]:
        """Pick next task based on policy."""
        if not self.task_queue:
            return None

        if self.policy.task_selection == "nearest":
            # Sort by distance to pickup
            self.task_queue.sort(
                key=lambda t: np.linalg.norm(t.pickup[:2] - rs.position[:2])
            )
        elif self.policy.task_selection == "random":
            random.shuffle(self.task_queue)
        # "oldest" = FIFO, no sort needed

        # Apply zone preference filter
        if self.policy.zone_preference != "balanced":
            preferred = [t for t in self.task_queue if self._in_zone(t.pickup)]
            if preferred:
                return preferred.pop(0)

        return self.task_queue.pop(0)

    def _in_zone(self, pos: np.ndarray) -> bool:
        """Check if position is in the preferred zone."""
        pref = self.policy.zone_preference
        if pref == "north":
            return pos[1] > 0
        elif pref == "south":
            return pos[1] < 0
        elif pref == "east":
            return pos[0] > 0
        elif pref == "west":
            return pos[0] < 0
        return True

    def _on_target_reached(self, rs: RobotState, idx: int, pickup_zones, dropoff_zones):
        """Handle robot arriving at its target."""
        if rs.state == "moving_to_pickup":
            rs.carrying_item = True
            rs.state = "moving_to_dropoff"
            rs.target = rs.current_task.dropoff
            self._controllers[idx].reset()
            print(f"[Fleet] Robot {rs.robot_id} picked up task {rs.current_task.task_id}")

        elif rs.state == "moving_to_dropoff":
            rs.carrying_item = False
            rs.current_task.completed = True
            rs.deliveries += 1
            self.total_deliveries += 1
            print(f"[Fleet] Robot {rs.robot_id} delivered task {rs.current_task.task_id} "
                  f"(total: {self.total_deliveries})")
            rs.current_task = None
            rs.target = None
            rs.state = "idle"
            self._controllers[idx].reset()

    def update_policy(self, new_params: dict):
        """Update fleet policy from Gemini-generated params."""
        old = self.policy.to_dict()
        self.policy.update(new_params)
        new = self.policy.to_dict()
        changes = {k: v for k, v in new.items() if old.get(k) != v}
        if changes:
            print(f"[Fleet] Policy updated: {changes}")
        return changes

    def get_state(self) -> dict:
        """Get full fleet state for the web dashboard."""
        return {
            "robots": [
                {
                    "id": rs.robot_id,
                    "x": float(rs.position[0]),
                    "y": float(rs.position[1]),
                    "state": rs.state,
                    "carrying_item": rs.carrying_item,
                    "deliveries": rs.deliveries,
                    "target": [float(rs.target[0]), float(rs.target[1])] if rs.target is not None else None,
                    "task_id": rs.current_task.task_id if rs.current_task else None,
                }
                for rs in self.robot_states
            ],
            "policy": self.policy.to_dict(),
            "stats": {
                "total_deliveries": self.total_deliveries,
                "total_steps": self.total_steps,
                "collisions_avoided": self.collisions_avoided,
                "pending_tasks": len(self.task_queue),
                "num_robots": self.num_robots,
            },
        }

    def reset(self, world):
        """Reset all robots to starting positions."""
        for i, rs in enumerate(self.robot_states):
            rs.state = "idle"
            rs.current_task = None
            rs.target = None
            rs.carrying_item = False
            rs.deliveries = 0
        self.total_deliveries = 0
        self.total_steps = 0
        self.collisions_avoided = 0
        self.task_queue.clear()
        self.task_counter = 0
        for ctrl in self._controllers:
            ctrl.reset()
