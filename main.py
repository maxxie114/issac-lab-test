#!/usr/bin/env python3
"""GeminiFleet — AI-Powered Multi-Robot Warehouse Simulator.

Uses PyBullet physics engine for realistic robot simulation with
Gemini-powered natural language fleet control.

Usage:
    # With 3D viewer (local with display):
    python main.py --gui

    # Headless (server / Vultr deployment):
    python main.py

    # Custom settings:
    python main.py --num-robots 6 --port 8080
"""

import argparse
import os
import sys
import threading
import time


def main():
    parser = argparse.ArgumentParser(description="GeminiFleet Warehouse Simulator")
    parser.add_argument("--gui", action="store_true", help="Show PyBullet 3D viewer")
    parser.add_argument("--num-robots", type=int, default=4, help="Number of robots (1-6)")
    parser.add_argument("--port", type=int, default=8000, help="Web dashboard port")
    args = parser.parse_args()

    from simulation.pybullet_sim import WarehouseSimulation

    # Create simulation
    print("[GeminiFleet] Starting PyBullet warehouse simulation...")
    sim = WarehouseSimulation(num_robots=args.num_robots, gui=args.gui)
    print(f"[GeminiFleet] {args.num_robots} robots spawned in warehouse")

    # Start web server
    import uvicorn
    from api.server import app, set_fleet_manager
    set_fleet_manager(sim)

    config = uvicorn.Config(app, host="0.0.0.0", port=args.port, log_level="warning")
    server = uvicorn.Server(config)
    server_thread = threading.Thread(target=server.run, daemon=True)
    server_thread.start()
    print(f"[GeminiFleet] Web dashboard: http://localhost:{args.port}")

    # Simulation loop
    print("[GeminiFleet] Simulation running. Press Ctrl+C to stop.")
    step = 0
    try:
        while True:
            sim.step()
            step += 1

            # Log every 5 seconds (~100 steps at 20Hz)
            if step % 100 == 0:
                state = sim.get_state()
                s = state["stats"]
                print(
                    f"[Step {step}] "
                    f"Deliveries: {s['total_deliveries']} | "
                    f"Pending: {s['pending_tasks']} | "
                    f"Avoided: {s['collisions_avoided']}"
                )

            time.sleep(0.05)  # ~20 steps/sec

    except KeyboardInterrupt:
        print("\n[GeminiFleet] Shutting down...")

    sim.close()


if __name__ == "__main__":
    main()
