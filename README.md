# roboracer-ws

A ROS 2 workspace integrating AutoDRIVE framework with F1TENTH racing implementations.

---

## Overview

This workspace combines autonomous driving capabilities through two main components:

- **autodrive_f1tenth**: Integration of AutoDRIVE framework for autonomous driving development
- **car_control_pub_sub**: Sample ROS 2 nodes demonstrating F1TENTH racing communication patterns

For detailed setup instructions and steps followed during development, please refer to [steps.md](steps.md).

## Repository Structure

```bash
roboracer-ws/
├── src/
│ ├── autodrive_f1tenth/ # AutoDRIVE ROS 2 API implementation
│ ├── car_control_pub_sub/ # F1TENTH communication examples
│ ├── simple_map/ # Mapping, waypoint logging, and visualization
│ ├── roboracer_interfaces/ # Custom messages and services (e.g., SaveWaypoints.srv)
│ └── ... # Other packages (e.g., wall_follow, gap_follow, pure_pursuit, etc.)
```

---

## Dependencies

### System Requirements
- ROS 2 (Humble recommended)
- Python 3.10 (Tested) (Recommended: Python 3.8+)
- AutoDRIVE Simulator App - Download and install following instructions at [AutoDRIVE Simulator](https://github.com/Tinker-Twins/AutoDRIVE/tree/AutoDRIVE-Simulator)


### Python Packages

A `requirements.txt` is provided. Install dependencies with:

```bash
pip3 install -r requirements.txt
```

**Key packages for WebSocket communication:**

```bash
pip3 install eventlet==0.33.3
pip3 install Flask==1.1.1
pip3 install Flask-SocketIO==4.1.0
pip3 install python-socketio==4.2.0
pip3 install python-engineio==3.13.0
pip3 install greenlet==1.0.0
pip3 install gevent==21.1.2
pip3 install gevent-websocket==0.10.1
pip3 install Jinja2==3.0.3
pip3 install itsdangerous==2.0.1
pip3 install werkzeug==2.0.3
```

**Key packages for data processing:**

```bash
pip3 install attrdict numpy pillow opencv-contrib-python
```

---

## Building the Workspace

```bash
cd ~/roboracer-ws
colcon build
source install/setup.bash
```

> **Tip:** If you restructure packages or interfaces, clean the workspace first:
> ```
> rm -rf build/ install/ log/
> ```

---

## Usage

### Launch Simulation

**Headless Mode:**

```bash
ros2 launch autodrive_f1tenth simulator_bringup_headless.launch.py
```

**RViz Mode:**

```bash
ros2 launch autodrive_f1tenth simulator_bringup_rviz.launch.py
```

**Foxglove Mode:** (custom addition; requires Foxglove)

```bash
ros2 launch autodrive_f1tenth simulator_bringup_foxglove.launch.py
```

---

### Control

> **Note:** Set the mode in the App to "Autonomous" and ensure the connection is "Connected".

**Keyboard Teleoperation:**

```bash
ros2 run autodrive_f1tenth teleop_keyboard
```

**Running Custom Publisher:**

```bash
ros2 run car_control_pub_sub custom_car_publisher 
```

**Running Custom Subscriber:**

```bash
ros2 run car_control_pub_sub custom_car_publisher 
```


---

## Running Nodes for Core Packages

### wall-follow

```bash
roboracer-ws$ colcon build --packages-select wall_follow
roboracer-ws$ ros2 launch wall_follow wall_follow.launch.py 
```

### wall-follow-ui-control

```bash
roboracer-ws$ colcon build
roboracer-ws$ source install/setup.bash
roboracer-ws$ ros2 launch wall_follow_ui_control wall_follow_ui_control.launch.py 
```

In another terminal, from [roboracer-autodrive-ui](https://github.com/Shreyas0812/roboracer-autodrive-ui):

Note: Make sure gradio is installed
```bash
roboracer-autodrive-ui$ gradio wall_follow_params_set.py 
```


### gap-follow-ui-control

```bash
roboracer-ws$ colcon build
roboracer-ws$ source install/setup.bash
roboracer-ws$ ros2 launch gap_follow_ui_control gap_follow_ui_control.launch.py 
```

In another terminal, from [roboracer-autodrive-ui](https://github.com/Shreyas0812/roboracer-autodrive-ui):
Note: Make sure gradio is installed
```bash
roboracer-autodrive-ui$ gradio gap_follow_params_set.py 
```

### pure-pursuit

```bash
roboracer-ws$ colcon build
roboracer-ws$ source install/setup.bash
roboracer-ws$ ros2 launch pure_pursuit pure_pursuit.launch.py 
```

In another terminal, from [roboracer-autodrive-ui](https://github.com/Shreyas0812/roboracer-autodrive-ui):Note: Make sure gradio is installed
```bash
roboracer-autodrive-ui$ gradio pure_pursuit_params_set.py
```


---

## simple_map Package

The [`simple_map`](src/simple_map/) package provides mapping, waypoint logging, and visualization functionality for Roboracer.

- **Mapping:**  
  Launch with:

```bash
ros2 launch simple_map simple_map.launch.py
```

- **Waypoint Saving:**  
Save waypoints to a custom location (make sure the service node is running):

```bash
ros2 service call /save_waypoints roboracer_interfaces/srv/SaveWaypoints "{source_filename: 'waypoints.csv', destination_filepath: '/your/path/waypoints.csv'}"
```

- **Visualization:**  
Visualize waypoints in RViz2:

```bash
ros2 launch simple_map visualise.launch.py
```

- **Important Note:**  
If you do not save the waypoints to another location, rebuilding the workspace will overwrite previous waypoint files. For example, if you build again before visualization, it will re-write the waypoints.

See [`simple_map/README.md`](src/simple_map/README.md) for full usage, configuration, and troubleshooting details.

---

## Attribution

- `autodrive_f1tenth` package is derived from [AutoDRIVE](https://github.com/Tinker-Twins/AutoDRIVE/tree/AutoDRIVE-Devkit/ADSS%20Toolkit/autodrive_ros2)
- `car_control_pub_sub` package is a sample based on F1TENTH Lab implementations at the University of Pennsylvania

---

## Best Practices

- Each ROS 2 package resides in its own directory under `src/` ([reference](https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.html)).
- Interface definitions (custom messages/services) are kept in a dedicated package (`roboracer_interfaces`).
- Each package should have its own `README.md` with usage and configuration details ([reference](https://docs.ros.org/en/rolling/How-To-Guides/Documenting-a-ROS-2-Package.html)).
- Use launch files for orchestrating multiple nodes.
- Source `install/setup.bash` in every new terminal before running ROS 2 commands.

---
