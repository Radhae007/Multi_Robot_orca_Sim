# Multi-Robot Navigation with ORCA Collision Avoidance
### ROS 2 Humble | Gazebo | Nav2 | TurtleBot3

> A decentralized multi-robot navigation system for 4 TurtleBot3 Waffle robots in a simulated warehouse environment. Robots navigate autonomously using Nav2 while avoiding inter-robot collisions using a custom Reciprocal Velocity Obstacle (RVO/ORCA) algorithm broadcasted over a P2P state-sharing network.

---

## Overview

This package implements and benchmarks three collision avoidance strategies for a 4-robot fleet:

| Mode | Strategy | Description |
|---|---|---|
| `bare_nav2` | None | Pure Nav2, no collision awareness. Robots collide freely. |
| `baseline` | Stop-and-Yield | Robots broadcast path previews. Lower-priority robot stops when paths intersect. |
| `orca_v2` | RVO2 + Smooth Penalties | Robots compute reciprocal velocity obstacles with Gaussian penalty curves and right-hand passing bias. |

---

## Architecture

```
                         ┌──────────────────────────────────────┐
                         │         /p2p/robot_states            │
                         │    (Shared DDS Topic, BEST_EFFORT)   │
                         └────────┬───────────┬─────────────────┘
                                  │           │
                 ┌────────────────▼───┐   ┌───▼────────────────┐
                 │   orca_p2p_node    │   │   orca_p2p_node    │
                 │       /tb1         │   │       /tb2          │
                 │                    │   │                     │
                 │  Nav2 cmd_vel_nav ──►  RVO Sampler ──► cmd_vel│
                 └────────────────────┘   └───────────────────┘
```

Each robot runs its own `orca_p2p_node` inside its namespace. The node:
1. Broadcasts its own pose + velocity to `/p2p/robot_states`
2. Listens to all peers on the same topic
3. If a peer is within `d_threshold` metres, switches from Nav2 pass-through to RVO velocity sampling
4. Publishes the safe velocity to `cmd_vel`

---

## ORCA Algorithm (v2)

The velocity sampler works by evaluating a discrete grid of `(v, w)` candidates centered around Nav2's desired velocity and selecting the one with the lowest total cost:

$$\text{cost}(v, w) = |v - v_{des}| + 0.5|w - w_{des}| + 0.1w + \sum_{\text{peers}} 500 \cdot e^{-3d}$$

| Term | Purpose |
|---|---|
| $\|v - v_{des}\|$ | Stay close to Nav2's commanded speed |
| $0.5\|w - w_{des}\|$ | Stay close to Nav2's commanded heading |
| $0.1w$ | **Right-hand passing bias** — breaks head-on symmetry |
| $500 \cdot e^{-3d}$ | **Smooth exponential penalty** — replaces the old harsh $1000/d$ spike |

---

## Repository Structure

```
tb3_multi_robot/
├── nodes/
│   ├── orca_p2p_node.py        # Core ORCA RVO collision avoidance node
│   ├── baseline_p2p_node.py    # Stop-and-Yield baseline node
│   └── benchmark_suite.py      # Automated benchmarking with reproducible trials
├── launch/
│   └── gazebo_multi_nav2_orca.launch.py  # Main launch file
├── msg/
│   └── RobotState.msg          # Custom P2P state message
├── params/
│   └── nav2_orca_params.yaml   # Nav2 parameters for all robots
├── map/
│   └── simple_warehouse.yaml   # Static map used by AMCL
└── worlds/
    └── simple_warehouse.world  # Gazebo warehouse world
```

---

## Dependencies

- **ROS 2 Humble**
- **Nav2** (`navigation2`, `nav2_bringup`)
- **Gazebo** (`gazebo_ros_pkgs`)
- **TurtleBot3** (`turtlebot3_gazebo`, `turtlebot3_navigation2`)

```bash
# Install key dependencies
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup
sudo apt install ros-humble-turtlebot3-gazebo ros-humble-turtlebot3-navigation2
```

---

## Build

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select turtlebot3_multi_robot
source install/setup.bash
```

---

## Running

Set the TurtleBot3 model before launching:
```bash
export TURTLEBOT3_MODEL=waffle
```

### Run with ORCA v2 (recommended)
```bash
ros2 launch turtlebot3_multi_robot gazebo_multi_nav2_orca.launch.py \
  p2p_executable:=orca_p2p_node.py \
  enable_orca:=true
```

### Run with Stop-and-Yield Baseline
```bash
ros2 launch turtlebot3_multi_robot gazebo_multi_nav2_orca.launch.py \
  p2p_executable:=baseline_p2p_node.py
```

### Run with Bare Nav2 (no collision avoidance)
```bash
ros2 launch turtlebot3_multi_robot gazebo_multi_nav2_orca.launch.py \
  p2p_executable:=orca_p2p_node.py \
  enable_orca:=false
```

---

## Benchmarking

Once the simulation is running, dispatch randomized navigation goals and collect metrics:

```bash
# Run 10 trials in ORCA mode (seed=42, reproducible)
ros2 run turtlebot3_multi_robot benchmark_suite.py --mode orca_v2 --trials 10

# Run 10 trials in baseline mode
ros2 run turtlebot3_multi_robot benchmark_suite.py --mode baseline --trials 10
```

Results are saved to `benchmark_results_<mode>.csv` with per-trial completion time, distance travelled, and collision count.

---

## Custom Message

```
# RobotState.msg
string robot_id
geometry_msgs/PoseStamped pose
geometry_msgs/PoseStamped goal
geometry_msgs/Twist velocity
geometry_msgs/Point[] path_preview
builtin_interfaces/Time stamp
```

---

## Launch Arguments

| Argument | Default | Description |
|---|---|---|
| `p2p_executable` | `orca_p2p_node.py` | Which P2P node to run (`orca_p2p_node.py` or `baseline_p2p_node.py`) |
| `enable_orca` | `true` | Enable RVO velocity override (set `false` for bare Nav2) |
| `params_file` | `nav2_orca_params.yaml` | Nav2 parameter file |
| `use_sim_time` | `true` | Use Gazebo simulation clock |

---

## License

Apache 2.0
