"""Gemini integration for natural language policy generation."""

import json
import os
import google.generativeai as genai

SYSTEM_PROMPT = """You are a warehouse robot fleet policy controller for an autonomous multi-robot system.
Given a natural language instruction from a fleet manager, generate updated policy parameters.

Available parameters you can set:
- speed (float 0.05-0.5): Robot linear speed in m/s. 0.05=very slow, 0.3=normal, 0.5=fast
- safety_margin (float 0.3-3.0): Minimum distance in meters between robots. Higher=safer
- congestion_response (string): How robots handle blocked paths. Options: "wait", "reroute", "slow_down"
- task_selection (string): How idle robots pick their next task. Options: "nearest", "oldest", "random"
- cooperation_mode (string): How robots interact. Options: "independent", "cooperative"
- zone_preference (string): Preferred warehouse zone. Options: "north", "south", "east", "west", "balanced"
- max_concurrent_tasks (int 1-3): Max tasks per robot

Current warehouse layout:
- Pickup zones (blue) on the west/left side
- Dropoff zones (green) on the east/right side
- Shelf racks in the center
- Robots start from the far west

Rules:
1. Only include parameters that should CHANGE based on the instruction
2. Include an "explanation" field with a brief reason for your choices
3. Respond with ONLY valid JSON, no markdown formatting

Example instruction: "Make the robots more careful"
Example response: {"speed": 0.15, "safety_margin": 2.0, "congestion_response": "wait", "explanation": "Reduced speed and increased safety margins for more cautious operation"}

Example instruction: "Speed things up, we're behind schedule"
Example response: {"speed": 0.45, "safety_margin": 0.5, "congestion_response": "slow_down", "explanation": "Maximized speed and reduced safety margins for faster throughput"}
"""


def configure_gemini(api_key: str = None):
    """Configure the Gemini API."""
    key = api_key or os.environ.get("GEMINI_API_KEY", "")
    if not key:
        raise ValueError("GEMINI_API_KEY not set. Pass it as argument or set env var.")
    genai.configure(api_key=key)


def generate_policy(instruction: str, current_policy: dict = None) -> dict:
    """Generate policy parameters from a natural language instruction.

    Args:
        instruction: Natural language command from the fleet manager
        current_policy: Current policy dict for context

    Returns:
        Dict with policy parameters and explanation
    """
    model = genai.GenerativeModel("gemini-2.0-flash")

    context = ""
    if current_policy:
        context = f"\n\nCurrent policy settings: {json.dumps(current_policy)}"

    prompt = f"{SYSTEM_PROMPT}{context}\n\nManager instruction: {instruction}"

    response = model.generate_content(prompt)
    text = response.text.strip()

    # Strip markdown code fences if present
    if text.startswith("```"):
        text = text.split("\n", 1)[1].rsplit("```", 1)[0].strip()

    result = json.loads(text)
    return result


def chat_with_gemini(instruction: str, current_policy: dict, fleet_state: dict) -> dict:
    """Higher-level chat that includes fleet state context."""
    model = genai.GenerativeModel("gemini-2.0-flash")

    state_summary = (
        f"Fleet has {fleet_state['stats']['num_robots']} robots, "
        f"{fleet_state['stats']['total_deliveries']} deliveries completed, "
        f"{fleet_state['stats']['pending_tasks']} tasks pending, "
        f"{fleet_state['stats']['collisions_avoided']} collisions avoided."
    )
    robot_summary = ", ".join(
        f"Robot {r['id']}: {r['state']}" for r in fleet_state["robots"]
    )

    prompt = (
        f"{SYSTEM_PROMPT}\n\n"
        f"Current policy: {json.dumps(current_policy)}\n"
        f"Fleet status: {state_summary}\n"
        f"Robot states: {robot_summary}\n\n"
        f"Manager instruction: {instruction}"
    )

    response = model.generate_content(prompt)
    text = response.text.strip()

    if text.startswith("```"):
        text = text.split("\n", 1)[1].rsplit("```", 1)[0].strip()

    return json.loads(text)
