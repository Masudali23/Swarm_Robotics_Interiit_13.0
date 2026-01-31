# Centralized Intelligence for Dynamic Swarm Navigation

Centralized control for multi-robot exploration, mapping, and object-targeting. The stack ships with ROS packages for world setup, mapping, exploration, planning, and task execution, plus a React UI and a Flask+LangChain orchestration layer.

---

## Repository layout
- `ROS_WS/` – Catkin workspace containing ROS packages, orchestrator `main.py`, and helper scripts.
- `ROS_WS/ui/` – Vite + React UI; runtime data and configuration live under `ui/database/`.
- `Videos/` – Demo captures.
- `Final Maps /` – Saved maps from earlier runs.

---

## System requirements
- OS: Ubuntu 20.04 (ROS Noetic target) recommended.
- CPU/RAM: 4+ cores, 16 GB RAM suggested for multi-bot Gazebo.
- GPU: NVIDIA GPU recommended for YOLO/DeepSORT acceleration (CPU also works with lower throughput).
- Desktop session: `gnome-terminal` and `tmux` are required by the provided scripts.
- Node.js: v18+ for the UI toolchain.

---

## Prerequisites
1. Install ROS Noetic desktop-full (follow http://wiki.ros.org/noetic/Installation/Ubuntu) and source `/opt/ros/noetic/setup.bash`.
2. System tools:
   ```bash
   sudo apt update
   sudo apt install -y python3-venv python3-pip python3-catkin-tools tmux gnome-terminal
   ```
3. Node.js v18+ (via apt, nvm, or Nodesource) with `npm`.
4. Optional but recommended: `rosdep` initialized (`sudo rosdep init && rosdep update`).

---

## Setup
All commands below assume the repo root: `Swarm_Robotics_Interiit_13.0`.

### 1) Build the catkin workspace
```bash
cd ROS_WS
rosdep install --from-paths src --ignore-src -r -y
catkin build
source devel/setup.bash
echo "source $(pwd)/devel/setup.bash" >> ~/.bashrc
```

### 2) Python environment
```bash
cd ROS_WS
python3 -m venv .venv
source .venv/bin/activate
pip install --upgrade pip
pip install -r requirements.txt
# deactivate  # when done
```

### 3) UI dependencies
```bash
cd ROS_WS/ui
npm install
```

### 4) Configuration
- Adjust bot count and initial poses in `ROS_WS/ui/database/config.yaml`.
- Populate `ROS_WS/ui/database/objects.json` and `ROS_WS/ui/database/robots.json` with the objects/robots present in your world for accurate targeting.
- Ensure Gazebo models needed by your worlds are available in `~/.gazebo/models` if you use custom assets.

---

## Running the stack
### Option A: One-shot via helper script
From `ROS_WS`:
```bash
chmod +x run.sh run_2.sh
./run.sh
```
`run.sh` starts a tmux session with `roscore`, the Flask orchestrator (`main.py`), and the UI dev server. Use `run_2.sh` if you prefer the Flask process in a standalone `gnome-terminal` window.

### Option B: Manual start
In separate terminals (after `source devel/setup.bash`):
```bash
roscore
python3 main.py
cd ROS_WS/ui && npm run dev
```
The UI serves on http://localhost:5173 by default (Vite), and the Flask API runs on http://localhost:5000.

### Orchestration API (Flask + LangChain)
- Endpoint: `POST /assign`
- Payload: `{"user_input": "explore the environment"}`
- The agent interprets the request and triggers ROS launch files through `main.py`:
  - `INIT`: Spawns the world (`custom_world`), YOLO + DeepSORT trackers, and the data manager.
  - `EXPM`: Map merger + frontier exploration + TEB planner + frontier completion check.
  - `TASK`: Map merger + TEB planner + object goal publisher (`swarm_tasks`).
- `main.py` opens new `gnome-terminal` tabs per launch file and waits for nodes to come up.

### Direct ROS launch examples
- Bring up the Gazebo world: `roslaunch custom_world custom_world.launch num_bots:=4`
- Merge maps: `roslaunch custom_world map_merger.launch num_bots:=4`
- Start frontier exploration: `roslaunch explore_lite explore.launch num_bots:=4`
- Start TEB local planner: `roslaunch teb_local_planner teb_test.launch num_bots:=4`
- Move obstacles (example): `cd ROS_WS/src/store_depends/move_models && python3 move_big_office.py`

---

## Troubleshooting
- If `roslaunch` or packages are missing, re-run `rosdep install --from-paths src --ignore-src -r -y` and ensure `source devel/setup.bash` is active in every terminal.
- Scripts rely on `gnome-terminal`; on headless servers use a GUI session or replace it with your preferred terminal in `run.sh`/`run_2.sh`.
- If tmux is unavailable, install it or start processes manually as in Option B.
- For agent/API issues, confirm `main.py` is running and reachable at http://localhost:5000, and that `ui/database` JSON/YAML files contain valid data.

---

## Notes
- Demo recordings live under `Videos/`.
- Saved reference maps are in `Final Maps /` for comparison or validation.

