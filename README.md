# roboracer-ws

A ROS 2 workspace integrating AutoDRIVE framework with F1TENTH racing implementations.

## Overview

This workspace combines autonomous driving capabilities through two main components:

- **autodrive_f1tenth**: Integration of AutoDRIVE framework for autonomous driving development
- **car_control_pub_sub**: Sample ROS 2 nodes demonstrating F1TENTH racing communication patterns

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


## Attribution

- autodrive_f1tenth package is derived from [AutoDRIVE](https://github.com/Tinker-Twins/AutoDRIVE/tree/AutoDRIVE-Devkit/ADSS%20Toolkit/autodrive_ros2)
- car_control_pub_sub package is a sample on F1TENTH Lab implementations at the University of Pennsylvania





