# GeminiFleet - AI-Powered Multi-Robot Warehouse Simulator

A multi-agent warehouse robot simulation built on **NVIDIA Isaac Sim** with **Google Gemini** natural language fleet control. Robots autonomously pick up and deliver items in a virtual warehouse, while a fleet manager controls behavior through plain English commands interpreted by Gemini AI.

## Architecture

```
┌──────────────────────────────────────────────────────────────┐
│                    Local Machine (GPU)                        │
│  ┌─────────────┐    ┌──────────────┐    ┌────────────────┐  │
│  │  Isaac Sim   │───▶│ Fleet Manager│◀───│ Policy Engine  │  │
│  │  (3D Render) │    │ (Navigation) │    │ (Gemini API)   │  │
│  └─────────────┘    └──────┬───────┘    └────────────────┘  │
│                            │                                 │
│                     ┌──────▼───────┐                        │
│                     │  FastAPI     │                        │
│                     │  WebSocket   │                        │
│                     └──────┬───────┘                        │
│                            │                                 │
│                     ┌──────▼───────┐                        │
│                     │ Web Dashboard│                        │
│                     │  (Browser)   │                        │
│                     └──────────────┘                        │
└──────────────────────────────────────────────────────────────┘

┌──────────────────────────────────────────────────────────────┐
│                  Vultr VM (No GPU)                            │
│  ┌──────────────────────────────────────────────────────┐   │
│  │  Docker: api_standalone.py                            │   │
│  │  - FastAPI + Web Dashboard                            │   │
│  │  - Standalone lightweight simulation                  │   │
│  │  - Gemini policy chat                                 │   │
│  └──────────────────────────────────────────────────────┘   │
└──────────────────────────────────────────────────────────────┘
```

## Quick Start

### Prerequisites

| Component | Requirement |
|-----------|-------------|
| **GPU** | NVIDIA RTX 4070+ (for Isaac Sim) |
| **Python** | 3.11 (required by Isaac Sim) |
| **CUDA** | 12.x |
| **OS** | Ubuntu 22.04 or Windows 10/11 |
| **Gemini API Key** | [Get one here](https://aistudio.google.com/apikey) |

### Option 1: Full Isaac Sim Mode (Local GPU)

```bash
# 1. Clone the repo
git clone <repo-url> geminifleet
cd geminifleet

# 2. Create Python 3.11 virtual environment
python3.11 -m venv .venv
source .venv/bin/activate  # Linux
# .venv\Scripts\activate   # Windows

# 3. Install Isaac Sim (large download, ~12GB)
pip install isaacsim

# 4. Install web dependencies
pip install -r requirements.txt

# 5. Set your Gemini API key
export GEMINI_API_KEY="your_key_here"

# 6. Run!
python main.py
```

Isaac Sim will open with the warehouse scene. Open http://localhost:8000 for the web dashboard.

**Command line options:**
```bash
python main.py --headless          # No GUI (faster)
python main.py --num-robots 6      # More robots (1-6)
python main.py --port 8080         # Different port
python main.py --no-prebuilt       # Build warehouse procedurally
```

### Option 2: Standalone Mode (No GPU, for Vultr)

```bash
# 1. Install dependencies
pip install -r requirements.txt

# 2. Set Gemini API key
export GEMINI_API_KEY="your_key_here"

# 3. Run standalone
python api_standalone.py
```

This runs a lightweight 2D simulation without Isaac Sim. Open http://localhost:8000.

### Option 3: Docker on Vultr

```bash
# 1. SSH into your Vultr VM
ssh root@your-vultr-ip

# 2. Clone the repo
git clone <repo-url> geminifleet
cd geminifleet

# 3. Create .env file
cp .env.example .env
# Edit .env and set GEMINI_API_KEY

# 4. Build and run
docker compose up -d

# Dashboard available at http://your-vultr-ip:8000
```

## Deploying to Vultr (Step by Step)

### 1. Create a Vultr VM

- Go to [vultr.com](https://vultr.com) and create an account
- Deploy a new **Cloud Compute** instance:
  - **OS**: Ubuntu 22.04
  - **Plan**: 1 vCPU, 1GB RAM ($5/mo is enough)
  - **Location**: Any
- Note the IP address and root password

### 2. Setup the VM

```bash
# SSH in
ssh root@YOUR_VULTR_IP

# Install Docker
curl -fsSL https://get.docker.com | sh

# Clone project
git clone <repo-url> geminifleet
cd geminifleet

# Configure
cp .env.example .env
nano .env  # Set your GEMINI_API_KEY

# Deploy
docker compose up -d

# Open firewall
ufw allow 8000/tcp
```

### 3. Access the Dashboard

Open `http://YOUR_VULTR_IP:8000` in your browser.

## Using the Dashboard

### Gemini Fleet Control

Type natural language commands in the chat panel to control robot behavior:

| Command | Effect |
|---------|--------|
| "Make robots faster" | Increases speed, reduces safety margin |
| "Prioritize safety" | Slows down, increases collision avoidance distance |
| "Focus on the north side" | Robots prefer north-side tasks |
| "Use random task selection" | Robots pick tasks randomly |
| "Make robots cooperate" | Enables cooperative mode |
| "Reroute around congestion" | Robots find alternate paths when blocked |

### What You See

- **2D Map**: Real-time top-down view of the warehouse
  - Blue squares = pickup zones (where robots collect items)
  - Green squares = dropoff zones (where robots deliver)
  - Brown rectangles = shelf racks
  - Colored circles = robots (dashed lines show planned paths)
- **Robot Panel**: Status of each robot (idle, moving, carrying)
- **Stats**: Deliveries, pending tasks, collision avoidances
- **Policy Panel**: Current fleet behavior parameters

## Project Structure

```
geminifleet/
├── main.py                  # Isaac Sim entry point (GPU mode)
├── api_standalone.py        # Standalone entry point (no GPU)
├── simulation/
│   ├── warehouse.py         # Warehouse environment (Isaac Sim scene)
│   ├── fleet.py             # Fleet manager (robot control + task assignment)
│   └── policy.py            # Policy dataclass
├── api/
│   ├── server.py            # FastAPI + WebSocket server
│   └── gemini.py            # Gemini API integration
├── frontend/
│   └── index.html           # Web dashboard (HTML5 Canvas)
├── Dockerfile               # Vultr deployment
├── docker-compose.yml
├── requirements.txt
└── .env.example
```

## Technologies

- **NVIDIA Isaac Sim** — Physics-based robot simulation with RTX rendering
- **Google Gemini 2.0 Flash** — Natural language to policy parameter generation
- **FastAPI + WebSocket** — Real-time web API
- **HTML5 Canvas** — 2D dashboard visualization
- **Docker** — Containerized deployment on Vultr

## How It Works

1. **Warehouse Setup**: Isaac Sim loads a warehouse environment with shelves, pickup zones, and dropoff zones
2. **Robot Fleet**: Jetbot robots are spawned and controlled via differential drive controllers
3. **Task System**: Delivery tasks (pickup → dropoff) are generated and assigned to idle robots
4. **Navigation**: Robots use `WheelBasePoseController` for autonomous waypoint navigation
5. **Collision Avoidance**: Robots check distances and respond based on policy (wait/reroute/slow down)
6. **Gemini AI**: Natural language commands are sent to Gemini, which returns JSON policy parameters
7. **Real-time Dashboard**: WebSocket pushes simulation state to the browser at ~10Hz

## Infrastructure Requirements

| Mode | CPU | RAM | GPU | Storage | Cost |
|------|-----|-----|-----|---------|------|
| **Isaac Sim (local)** | 4+ cores | 16GB+ | RTX 4070+ | 20GB | Your machine |
| **Vultr (web only)** | 1 vCPU | 1GB | None | 10GB | ~$5/mo |
| **Brev (full cloud)** | 4+ cores | 16GB+ | A10G/L40S | 50GB | ~$1-3/hr |

## License

MIT
