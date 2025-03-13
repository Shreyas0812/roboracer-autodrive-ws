# roboracer-ws

A ROS 2 workspace integrating AutoDRIVE framework with F1TENTH racing implementations.

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
│ └── car_control_pub_sub/ # F1TENTH communication examples
```

## Dependencies

### System Requirements
- ROS 2 (Humble recommended)
- Python 3.10 (Tested) (Recommended: Python 3.8+)
- AutoDRIVE Simulator App - Download and install following instructions at [AutoDRIVE Simulator](https://github.com/Tinker-Twins/AutoDRIVE/tree/AutoDRIVE-Simulator)


### Python Packages (requirements.txt is provided)

WebSocket Communication: 

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

Data Processing:

```bash
pip3 install attrdict numpy pillow opencv-contrib-python
```

## Building the Workspace

```bash
cd ~/roboracer-ws
colcon build
source install/setup.bash
```

## Usage

### Launch Simulation

Headless Mode:

```bash
ros2 launch autodrive_f1tenth simulator_bringup_headless.launch.py
```

RViz Mode:

```bash
ros2 launch autodrive_f1tenth simulator_bringup_rviz.launch.py
```

### Control 

Note: Make sure that the mode in the App is "Autonomous" and the connection is on "Connected"

Keyboard Teleoperation:

```bash
ros2 run autodrive_f1tenth teleop_keyboard
```

Running Custom Publisher 

```bash
ros2 run car_control_pub_sub custom_car_publisher 
```

Running Custom Subscriber 

```bash
ros2 run car_control_pub_sub custom_car_publisher 
```

# roboracer-ws-running-nodes

## wall-follow

```bash
roboracer-ws$ colcon build --packages-select wall_follow
roboracer-ws$ ros2 launch wall_follow wall_follow.launch.py 
```

## wall-follow

```bash
roboracer-ws$ colcon build --packages-select wall_follow
roboracer-ws$ ros2 launch wall_follow wall_follow.launch.py 
```

## wall-follow-ui-control

```bash
roboracer-ws$ colcon build
roboracer-ws$ source install/setup.bash
roboracer-ws$ ros2 launch wall_follow_ui_control wall_follow_ui_control.launch.py 
```

In another terminal, from the roboracer-autodrive-ui: (https://github.com/Shreyas0812/roboracer-autodrive-ui)

Note: Make sure gradio is installed
```bash
roboracer-autodrive-ui$ gradio wall_follow_params_set.py 
```


## gap-follow-ui-control

```bash
roboracer-ws$ colcon build
roboracer-ws$ source install/setup.bash
roboracer-ws$ ros2 launch gap_follow_ui_control gap_follow_ui_control.launch.py 
```

In another terminal, from the roboracer-autodrive-ui: (https://github.com/Shreyas0812/roboracer-autodrive-ui)
Note: Make sure gradio is installed
```bash
roboracer-autodrive-ui$ gradio gap_follow_params_set.py 
```

# Attribution

- autodrive_f1tenth package is derived from [AutoDRIVE](https://github.com/Tinker-Twins/AutoDRIVE/tree/AutoDRIVE-Devkit/ADSS%20Toolkit/autodrive_ros2)
- car_control_pub_sub package is a sample on F1TENTH Lab implementations at the University of Pennsylvania





