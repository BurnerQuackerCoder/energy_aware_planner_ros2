# Energy-Aware Global Planner for ROS 2 (Nav2) 🔋🗺️

[![ROS 2](https://img.shields.io/badge/ROS_2-Humble-22314E?logo=ros)](https://docs.ros.org/en/humble/)
[![C++](https://img.shields.io/badge/C++-14/17-00599C?logo=c%2B%2B)]()
[![License](https://img.shields.io/badge/License-BSD-blue.svg)]()

A custom global path planner plugin for the **ROS 2 Navigation Stack (Nav2)**. This planner dynamically balances the trade-off between the shortest distance and the most energy-efficient route based on the robot's real-time battery level.

Instead of planning blindly, the algorithm actively avoids high-friction surfaces (like carpet) when the battery reaches critical levels, significantly improving the survivability of autonomous mobile robots during long-duration indoor missions.

## ✨ Key Features

- **Custom Nav2 Plugin**: Fully integrates with `nav2_core::GlobalPlanner` and standard `nav2_costmap_2d`.
- **State-Dependent Cost Function**: Uses a modified A* algorithm where edge traversal costs shift dynamically based on battery State of Charge (SoC).
- **Sigmoid Adaptive Controller**: Smoothly transitions routing priorities using a tunable mathematical sigmoid function ($\alpha(\beta)$) to prevent path oscillation.
- **High-Friction Zones**: Defines specific coordinate zones (e.g., `carpeted_hallway`) with custom friction coefficients ($\mu$) mapped to energy penalties.
- **Interactive Battery Simulator**: Includes a real-time teleop-style script to drop or reset the battery level mid-mission and watch the global path recalculate live in RViz.

## 🧮 Mathematical Model

The planner evaluates the traversal cost $c$ between two grid nodes ($u, v$) using the following custom function:

$$
c(u,v) = (1 - \alpha(\beta)) \cdot d(u,v) + \alpha(\beta) \cdot \mu \cdot d(u,v)
$$

- **$d(u,v)$**: Euclidean distance between the nodes.
- **$\mu$**: Surface friction/energy usage factor (e.g., 20.0 for carpet).
- **$\alpha(\beta)$**: The dynamic weighting factor controlled by the battery level ($\beta$), defined by a sigmoid curve:

$$
\alpha(\beta) = \frac{1}{1 + e^{k(\beta - \beta_{crit})}}
$$

When the battery is full, $\alpha \approx 0$, and the robot takes the shortest path. When the battery drops below $\beta_{crit}$ (e.g., 20%), $\alpha \approx 1$, and the robot actively routes around high-friction zones.

## ⚙️ Dependencies

- ROS 2 (Developed and tested on **Humble**)
- `nav2_core`, `nav2_costmap_2d`, `nav2_util`
- `geometry_msgs`, `nav_msgs`, `visualization_msgs`

## 🚀 Installation & Build

Clone the repository into your ROS 2 workspace's `src` directory and build:
```bash
cd ~/ros2_ws/src
git clone <your-repo-url>
cd ~/ros2_ws
colcon build --packages-select energy_aware_planner
source install/setup.bash
```
## 🎮 Running the Demo

The package includes a full simulation setup featuring a map, the custom planner, and a battery degradation simulator.

### 1. Launch the Planner and RViz

A convenience bash script is provided to set up the FastDDS profiles, launch the Nav2 stack, and open RViz2.

```bash
cd ~/ros2_ws/src/energy_aware_planner_ros2/energy_aware_planner_ros2-main
chmod +x run_demo.sh
./run_demo.sh
```

> **Note**: High-friction zones will be visualized in RViz as flat, semi-transparent orange markers.

### 2. Run the Battery Simulator

In a **new terminal**, start the interactive battery simulator node. This allows you to manually trigger battery drain to observe how the A* planner dynamically avoids the carpeted hallway.

```bash
source ~/ros2_ws/install/setup.bash
ros2 run energy_aware_planner battery_sim.py
```

**Simulator Controls:**

| Key | Action                        |
|-----|-------------------------------|
| `d` | Drop battery by 10%           |
| `r` | Reset battery to 100%         |
| `q` | Quit                          |

### 3. Test the Planner

1. In RViz, use the **2D Pose Estimate** tool to set the robot's initial position.
2. Use the **Nav2 Goal** tool to set a destination across the orange friction zone.
3. Observe the initial path (it should cut straight through the zone because the default battery is 100%).
4. In the Battery Simulator terminal, press `d` continuously to drop the battery below 20%.
5. Set a new Nav2 Goal. The path will now automatically route around the high-friction zone!

## 🔧 Parameters

You can tune the planner's behavior in your Nav2 parameter YAML file:

| Parameter     | Type   | Default | Description |
|---------------|--------|---------|-----------|
| `beta_crit`   | double | 0.2     | The critical battery threshold (20%) where the algorithm shifts from distance-optimized to energy-optimized. |
| `k_sigmoid`   | double | 10.0    | The steepness of the sigmoid curve. Higher values create a sharper behavioral transition. |

## 📁 Package Structure

```plaintext
energy_aware_planner/
├── include/energy_aware_planner/
│   └── energy_aware_planner.hpp      # C++ Header for the Nav2 Plugin
├── src/
│   └── energy_aware_planner.cpp      # C++ Implementation of custom A* search
├── scripts/
│   └── battery_sim.py                # Interactive Python battery simulator
├── launch/
│   └── demo.launch.py                # ROS 2 Launch file for the demo
├── config/
│   ├── nav2_params.yaml              # Nav2 configuration specifying the custom plugin
│   └── energy_demo.rviz              # Pre-configured RViz workspace
├── maps/                             # Sample map and YAML
├── plugin_description.xml            # Pluginlib export definition
├── package.xml
└── CMakeLists.txt
```

## 📜 License

This software is released under the **BSD License**.
```
