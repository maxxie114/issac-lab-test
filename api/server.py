"""FastAPI server — web dashboard + Gemini chat + simulation state."""

import asyncio
import json
import os
from typing import Optional
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import HTMLResponse, FileResponse
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel

app = FastAPI(title="GeminiFleet", description="AI-Powered Multi-Robot Warehouse Simulator")

# Shared state — set by the main simulation process
_fleet_manager = None
_simulation_running = False
_connected_clients: list[WebSocket] = []
_message_log: list[dict] = []


def set_fleet_manager(fm):
    global _fleet_manager
    _fleet_manager = fm


def set_simulation_running(running: bool):
    global _simulation_running
    _simulation_running = running


class PolicyRequest(BaseModel):
    instruction: str
    api_key: Optional[str] = None


@app.get("/", response_class=HTMLResponse)
async def root():
    frontend_path = os.path.join(os.path.dirname(__file__), "..", "frontend", "index.html")
    with open(frontend_path) as f:
        return HTMLResponse(content=f.read())


@app.get("/api/state")
async def get_state():
    if _fleet_manager is None:
        return {"error": "Simulation not initialized"}
    return _fleet_manager.get_state()


@app.post("/api/policy")
async def update_policy(req: PolicyRequest):
    if _fleet_manager is None:
        return {"error": "Simulation not initialized"}

    try:
        from api.gemini import chat_with_gemini, configure_gemini

        api_key = req.api_key or os.environ.get("GEMINI_API_KEY")
        if not api_key:
            return {"error": "GEMINI_API_KEY not configured"}

        configure_gemini(api_key)
        fleet_state = _fleet_manager.get_state()
        result = chat_with_gemini(req.instruction, fleet_state["policy"], fleet_state)

        explanation = result.pop("explanation", "Policy updated")
        changes = _fleet_manager.update_policy(result)

        response = {
            "explanation": explanation,
            "changes": changes,
            "new_policy": _fleet_manager.policy.to_dict(),
        }

        # Log the message
        _message_log.append({"role": "user", "content": req.instruction})
        _message_log.append({"role": "assistant", "content": explanation, "changes": changes})

        # Notify WebSocket clients
        await _broadcast({"type": "policy_update", "data": response})

        return response

    except json.JSONDecodeError:
        return {"error": "Gemini returned invalid JSON. Try rephrasing your instruction."}
    except Exception as e:
        return {"error": str(e)}


@app.post("/api/reset")
async def reset_simulation():
    if _fleet_manager is None:
        return {"error": "Simulation not initialized"}
    # Reset is handled by the main loop
    return {"status": "reset_requested"}


@app.get("/api/messages")
async def get_messages():
    return {"messages": _message_log}


@app.websocket("/ws")
async def websocket_endpoint(ws: WebSocket):
    await ws.accept()
    _connected_clients.append(ws)
    try:
        while True:
            # Send state updates every 100ms
            if _fleet_manager:
                state = _fleet_manager.get_state()
                await ws.send_json({"type": "state", "data": state})
            await asyncio.sleep(0.1)
    except WebSocketDisconnect:
        _connected_clients.remove(ws)
    except Exception:
        if ws in _connected_clients:
            _connected_clients.remove(ws)


async def _broadcast(message: dict):
    for client in _connected_clients[:]:
        try:
            await client.send_json(message)
        except Exception:
            _connected_clients.remove(client)
