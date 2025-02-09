Steps Followed: 

Step 1:

Visit: 
```bash
https://github.com/Tinker-Twins/AutoDRIVE/tree/AutoDRIVE-Simulator
```

Download the version corresponding to the PC


Step 2:

```bash
git clone -b AutoDRIVE-Devkit --single-branch https://github.com/Tinker-Twins/AutoDRIVE.git
```


Step 3:

```bash
cd ./AutoDRIVE/ADSS Toolkit/autodrive_ros2/autodrive_f1tenth
```

Copy this folfer to the workspace/scr

```bash
roboracer-ws$ colcon build
```

Step 4:

Change scripts-dir to scripts_dir and install-scripts to install_scripts for deprecation warning,


This is inside 

```
roboracer-ws/src/autodrive_f1tenth/setup.cfg
```


Step 5:

```bash
$ pip3 install -r requirements.txt
```

Step 6:
```bash
$ source /opt/ros/humble/setup.bash 
$ colcon build
$ source install/local_setup.bash 
```

```bash
$ ros2 launch autodrive_f1tenth simulator_bringup_headless.launch.py
```

Error: (Because of python3.10 and 3.9 differences)

autodrive_incoming_bridge-1]     from tf_transformations import quaternion_from_euler # Euler angle representation to quaternion representation
[autodrive_incoming_bridge-1] ModuleNotFoundError: No module named 'tf_transformations'

sudo apt install ros-humble-tf-transformations
pip install attrdict3

```bash
$ ros2 launch autodrive_f1tenth simulator_bringup_headless.launch.py
```

```bash
$ ros2 topic list
/autodrive/f1tenth_1/front_camera
/autodrive/f1tenth_1/imu
/autodrive/f1tenth_1/ips
/autodrive/f1tenth_1/left_encoder
/autodrive/f1tenth_1/lidar
/autodrive/f1tenth_1/right_encoder
/autodrive/f1tenth_1/steering
/autodrive/f1tenth_1/steering_command
/autodrive/f1tenth_1/throttle
/autodrive/f1tenth_1/throttle_command
/parameter_events
/rosout
```

Step 7:

sudo apt install ros-humble-rviz-imu-plugin

ros2 launch autodrive_f1tenth simulator_bringup_rviz.launch.py



Click on connect in the sim, rviz should show something 


Step 8:

```bash
ros2 run autodrive_f1tenth teleop_keyboard
```

Note: Make sure to Change the mode to autonomous in the GUI