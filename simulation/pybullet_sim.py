"""PyBullet-based warehouse robot simulation.

Provides real physics simulation with differential-drive robots
navigating a warehouse environment. Runs headless for server
deployment or with GUI for local development.
"""

import math
import time
import numpy as np
import pybullet as p
import pybullet_data

from .policy import Policy


# Warehouse layout constants (meters)
WAREHOUSE_W = 20.0
WAREHOUSE_H = 14.0

SHELF_POSITIONS = []
for row in range(3):
    for col in range(4):
        x = -4.0 + col * 3.0
        y = -3.5 + row * 3.5
        SHELF_POSITIONS.append((x, y))

PICKUP_ZONES = [
    (-8.0, -5.0), (-8.0, 0.0), (-8.0, 5.0),
    (-5.0, -5.0), (-5.0, 5.0),
]

DROPOFF_ZONES = [
    (8.0, -5.0), (8.0, -1.5), (8.0, 1.5), (8.0, 5.0),
]

SPAWN_POSITIONS = [
    (-8.0, -2.5), (-8.0, 2.5),
    (-6.5, -5.0), (-6.5, 5.0),
    (-8.0, 0.0), (-6.5, 0.0),
]


class WarehouseSimulation:
    """PyBullet warehouse with differential-drive robots."""

    def __init__(self, num_robots=4, gui=False):
        self.num_robots = min(num_robots, len(SPAWN_POSITIONS))
        self.gui = gui
        self.policy = Policy()
        self.physics_client = None
        self.robot_ids = []
        self.robot_states = []
        self.task_queue = []
        self.task_counter = 0
        self.total_deliveries = 0
        self.total_steps = 0
        self.collisions_avoided = 0
        self.paused = True  # Start paused — user must click Start
        self._setup()

    def _setup(self):
        """Initialize PyBullet and build the scene."""
        if self.gui:
            self.physics_client = p.connect(p.GUI)
            p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
            p.resetDebugVisualizerCamera(
                cameraDistance=18, cameraYaw=0, cameraPitch=-70,
                cameraTargetPosition=[0, 0, 0]
            )
        else:
            self.physics_client = p.connect(p.DIRECT)

        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)
        p.setTimeStep(1.0 / 240.0)

        # Ground plane
        p.loadURDF("plane.urdf")

        # Warehouse floor (visual)
        floor_visual = p.createVisualShape(
            p.GEOM_BOX,
            halfExtents=[WAREHOUSE_W / 2, WAREHOUSE_H / 2, 0.005],
            rgbaColor=[0.35, 0.35, 0.35, 1],
        )
        p.createMultiBody(baseVisualShapeIndex=floor_visual, basePosition=[0, 0, 0.005])

        # Walls
        self._create_wall(0, WAREHOUSE_H / 2, WAREHOUSE_W, 0.1)   # north
        self._create_wall(0, -WAREHOUSE_H / 2, WAREHOUSE_W, 0.1)  # south
        self._create_wall(WAREHOUSE_W / 2, 0, 0.1, WAREHOUSE_H)   # east
        self._create_wall(-WAREHOUSE_W / 2, 0, 0.1, WAREHOUSE_H)  # west

        # Shelves
        for x, y in SHELF_POSITIONS:
            self._create_shelf(x, y)

        # Zone markers (flat colored boxes on the floor)
        for x, y in PICKUP_ZONES:
            self._create_zone_marker(x, y, [0.2, 0.5, 1.0, 0.7])  # blue
        for x, y in DROPOFF_ZONES:
            self._create_zone_marker(x, y, [0.2, 0.85, 0.3, 0.7])  # green

        # Spawn robots
        self._spawn_robots()

        # Generate initial tasks
        self.generate_tasks(6)

    def _create_wall(self, x, y, w, d):
        h = 1.5
        col = p.createCollisionShape(p.GEOM_BOX, halfExtents=[w / 2, d / 2, h / 2])
        vis = p.createVisualShape(p.GEOM_BOX, halfExtents=[w / 2, d / 2, h / 2],
                                  rgbaColor=[0.7, 0.7, 0.65, 1])
        p.createMultiBody(baseMass=0, baseCollisionShapeIndex=col,
                          baseVisualShapeIndex=vis, basePosition=[x, y, h / 2])

    def _create_shelf(self, x, y):
        col = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.6, 0.25, 0.6])
        vis = p.createVisualShape(p.GEOM_BOX, halfExtents=[0.6, 0.25, 0.6],
                                  rgbaColor=[0.55, 0.35, 0.15, 1])
        p.createMultiBody(baseMass=0, baseCollisionShapeIndex=col,
                          baseVisualShapeIndex=vis, basePosition=[x, y, 0.6])

    def _create_zone_marker(self, x, y, color):
        vis = p.createVisualShape(p.GEOM_BOX, halfExtents=[0.4, 0.4, 0.01],
                                  rgbaColor=color)
        p.createMultiBody(baseVisualShapeIndex=vis, basePosition=[x, y, 0.01])

    def _spawn_robots(self):
        """Create simple differential-drive robot bodies."""
        robot_colors = [
            [1.0, 0.85, 0.0, 1],   # yellow
            [0.0, 0.9, 0.9, 1],    # cyan
            [1.0, 0.3, 0.7, 1],    # pink
            [1.0, 0.5, 0.0, 1],    # orange
            [0.5, 0.5, 1.0, 1],    # periwinkle
            [0.3, 1.0, 0.3, 1],    # lime
        ]

        for i in range(self.num_robots):
            x, y = SPAWN_POSITIONS[i]
            color = robot_colors[i % len(robot_colors)]

            # Robot body: cylinder
            col = p.createCollisionShape(p.GEOM_CYLINDER, radius=0.25, height=0.2)
            vis = p.createVisualShape(p.GEOM_CYLINDER, radius=0.25, length=0.2,
                                      rgbaColor=color)
            robot_id = p.createMultiBody(
                baseMass=1.0,
                baseCollisionShapeIndex=col,
                baseVisualShapeIndex=vis,
                basePosition=[x, y, 0.15],
            )

            # Lock rotation to Z-axis only (no tipping)
            p.changeDynamics(robot_id, -1, linearDamping=0.9, angularDamping=0.9)

            self.robot_ids.append(robot_id)
            self.robot_states.append({
                "id": i,
                "state": "idle",
                "carrying_item": False,
                "deliveries": 0,
                "current_task": None,
                "target": None,
            })

    def add_robot(self, x=None, y=None):
        """Add a new robot to the fleet at runtime."""
        robot_colors = [
            [1.0, 0.85, 0.0, 1], [0.0, 0.9, 0.9, 1], [1.0, 0.3, 0.7, 1],
            [1.0, 0.5, 0.0, 1], [0.5, 0.5, 1.0, 1], [0.3, 1.0, 0.3, 1],
            [0.8, 0.2, 0.2, 1], [0.9, 0.9, 0.9, 1], [0.6, 0.0, 0.8, 1],
        ]
        i = len(self.robot_ids)

        if x is None or y is None:
            if i < len(SPAWN_POSITIONS):
                x, y = SPAWN_POSITIONS[i]
            else:
                x, y = -8.0 + (i % 4) * 1.5, -4.0 + (i // 4) * 2.0

        color = robot_colors[i % len(robot_colors)]
        col = p.createCollisionShape(p.GEOM_CYLINDER, radius=0.25, height=0.2)
        vis = p.createVisualShape(p.GEOM_CYLINDER, radius=0.25, length=0.2,
                                  rgbaColor=color)
        robot_id = p.createMultiBody(
            baseMass=1.0, baseCollisionShapeIndex=col,
            baseVisualShapeIndex=vis, basePosition=[x, y, 0.15],
        )
        p.changeDynamics(robot_id, -1, linearDamping=0.9, angularDamping=0.9)

        self.robot_ids.append(robot_id)
        self.robot_states.append({
            "id": i, "state": "idle", "carrying_item": False,
            "deliveries": 0, "current_task": None, "target": None,
        })
        self.num_robots = len(self.robot_ids)
        return i

    def generate_tasks(self, count=3):
        for _ in range(count):
            self.task_counter += 1
            pickup = list(PICKUP_ZONES[np.random.randint(len(PICKUP_ZONES))])
            dropoff = list(DROPOFF_ZONES[np.random.randint(len(DROPOFF_ZONES))])
            self.task_queue.append({
                "id": self.task_counter,
                "pickup": pickup,
                "dropoff": dropoff,
            })

    def step(self):
        """Advance simulation by one logical step (~4 physics substeps)."""
        if self.paused:
            return
        self.total_steps += 1

        # Assign tasks to idle robots
        for rs in self.robot_states:
            if rs["state"] == "idle" and self.task_queue:
                task = self._select_task(rs)
                if task:
                    rs["current_task"] = task
                    rs["state"] = "moving_to_pickup"
                    rs["target"] = task["pickup"]

        # Drive each robot toward its target
        for i, rs in enumerate(self.robot_states):
            pos, orn = p.getBasePositionAndOrientation(self.robot_ids[i])
            rx, ry = pos[0], pos[1]

            if rs["target"] is None:
                continue

            tx, ty = rs["target"]
            dx, dy = tx - rx, ty - ry
            dist = math.sqrt(dx * dx + dy * dy)

            # Check arrival
            if dist < 0.4:
                self._on_arrival(rs, i)
                continue

            # Collision avoidance: lower-ID robot has priority (prevents deadlock)
            should_wait = False
            for j, other_id in enumerate(self.robot_ids):
                if j == i:
                    continue
                other_pos = p.getBasePositionAndOrientation(other_id)[0]
                robot_dist = math.sqrt((rx - other_pos[0]) ** 2 + (ry - other_pos[1]) ** 2)
                if robot_dist < self.policy.safety_margin and i > j:
                    # Only higher-ID robot yields
                    self.collisions_avoided += 1
                    if self.policy.congestion_response == "wait":
                        should_wait = True
                        break

            if should_wait:
                continue

            # Move toward target (kinematic — direct position update)
            speed = self.policy.speed * 0.15  # per-step displacement
            move_dist = min(speed, dist)
            nx = rx + (dx / dist) * move_dist
            ny = ry + (dy / dist) * move_dist

            # Update position and face direction of movement
            yaw = math.atan2(dy, dx)
            quat = p.getQuaternionFromEuler([0, 0, yaw])
            p.resetBasePositionAndOrientation(
                self.robot_ids[i], [nx, ny, 0.15], quat
            )

        # Step physics (multiple substeps for stability)
        for _ in range(4):
            p.stepSimulation()

        # Replenish tasks
        if len(self.task_queue) < 3:
            self.generate_tasks(3)

    def _select_task(self, rs):
        if not self.task_queue:
            return None

        rid = self.robot_ids[rs["id"]]
        pos = p.getBasePositionAndOrientation(rid)[0]

        if self.policy.task_selection == "nearest":
            self.task_queue.sort(
                key=lambda t: math.sqrt(
                    (t["pickup"][0] - pos[0]) ** 2 + (t["pickup"][1] - pos[1]) ** 2
                )
            )
        elif self.policy.task_selection == "random":
            np.random.shuffle(self.task_queue)

        # Zone preference
        if self.policy.zone_preference != "balanced":
            preferred = [t for t in self.task_queue if self._in_zone(t["pickup"])]
            if preferred:
                task = preferred[0]
                self.task_queue.remove(task)
                return task

        return self.task_queue.pop(0)

    def _in_zone(self, pos):
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

    def _on_arrival(self, rs, idx):
        if rs["state"] == "moving_to_pickup":
            rs["carrying_item"] = True
            rs["state"] = "moving_to_dropoff"
            rs["target"] = rs["current_task"]["dropoff"]
        elif rs["state"] == "moving_to_dropoff":
            rs["carrying_item"] = False
            rs["deliveries"] += 1
            self.total_deliveries += 1
            rs["current_task"] = None
            rs["target"] = None
            rs["state"] = "idle"

    def unstuck_robot(self, robot_id):
        """Unstick a robot by cancelling its task and teleporting to a clear spot."""
        if robot_id < 0 or robot_id >= len(self.robot_states):
            return False

        rs = self.robot_states[robot_id]

        # Put current task back in the queue
        if rs["current_task"] is not None:
            self.task_queue.append(rs["current_task"])

        # Reset robot state
        rs["state"] = "idle"
        rs["carrying_item"] = False
        rs["current_task"] = None
        rs["target"] = None

        # Find a clear spawn position (away from other robots)
        candidates = list(SPAWN_POSITIONS) + [(-7.0, -2.5), (-7.0, 2.5), (-6.0, 0.0)]
        best_pos = candidates[0]
        best_min_dist = -1

        for cx, cy in candidates:
            min_dist = float('inf')
            for j, rid in enumerate(self.robot_ids):
                if j == robot_id:
                    continue
                other_pos = p.getBasePositionAndOrientation(rid)[0]
                d = math.sqrt((cx - other_pos[0]) ** 2 + (cy - other_pos[1]) ** 2)
                min_dist = min(min_dist, d)
            if min_dist > best_min_dist:
                best_min_dist = min_dist
                best_pos = (cx, cy)

        # Teleport the robot
        quat = p.getQuaternionFromEuler([0, 0, 0])
        p.resetBasePositionAndOrientation(
            self.robot_ids[robot_id], [best_pos[0], best_pos[1], 0.15], quat
        )
        return True

    def unstuck_all(self):
        """Unstick all robots that appear stuck (not idle but not making progress)."""
        unstuck = []
        for rs in self.robot_states:
            if rs["state"] != "idle" and rs["target"] is not None:
                rid = self.robot_ids[rs["id"]]
                pos = p.getBasePositionAndOrientation(rid)[0]
                # Check if robot is very close to a shelf (likely stuck)
                for sx, sy in SHELF_POSITIONS:
                    d = math.sqrt((pos[0] - sx) ** 2 + (pos[1] - sy) ** 2)
                    if d < 1.0:  # Within 1m of a shelf — likely stuck
                        self.unstuck_robot(rs["id"])
                        unstuck.append(rs["id"])
                        break
        return unstuck

    def update_policy(self, params):
        old = self.policy.to_dict()
        self.policy.update(params)
        new = self.policy.to_dict()
        return {k: v for k, v in new.items() if old.get(k) != v}

    def get_state(self):
        robots = []
        for i, rs in enumerate(self.robot_states):
            pos = p.getBasePositionAndOrientation(self.robot_ids[i])[0]
            robots.append({
                "id": rs["id"],
                "x": float(pos[0]),
                "y": float(pos[1]),
                "state": rs["state"],
                "carrying_item": rs["carrying_item"],
                "deliveries": rs["deliveries"],
                "target": rs["target"],
                "task_id": rs["current_task"]["id"] if rs["current_task"] else None,
            })
        return {
            "robots": robots,
            "policy": self.policy.to_dict(),
            "paused": self.paused,
            "stats": {
                "total_deliveries": self.total_deliveries,
                "total_steps": self.total_steps,
                "collisions_avoided": self.collisions_avoided,
                "pending_tasks": len(self.task_queue),
                "num_robots": self.num_robots,
            },
        }

    def render_camera(self, width=640, height=480):
        """Render a top-down camera view (returns RGB numpy array)."""
        view_matrix = p.computeViewMatrix(
            cameraEyePosition=[0, 0, 20],
            cameraTargetPosition=[0, 0, 0],
            cameraUpVector=[0, 1, 0],
        )
        proj_matrix = p.computeProjectionMatrixFOV(
            fov=60, aspect=width / height, nearVal=0.1, farVal=100,
        )
        _, _, rgb, _, _ = p.getCameraImage(
            width, height, view_matrix, proj_matrix,
            renderer=p.ER_TINY_RENDERER,
        )
        return np.array(rgb, dtype=np.uint8).reshape(height, width, 4)[:, :, :3]

    def close(self):
        p.disconnect(self.physics_client)
