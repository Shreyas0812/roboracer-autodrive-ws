# simple_map ROS 2 Package

This package provides simple mapping and waypoint logging functionality for the Roboracer project, including map generation, waypoint saving, and visualization.

**Table of Contents**
- [Package Creation](#package-creation)
- [File Structure](#file-structure)
- [Building the Package](#building-the-package)
- [Mapping](#mapping)
- [Waypoint Saving](#waypoint-saving)
  - [Simple Saving](#simple-saving)
  - [Centerline Waypoints](#centerline-waypoints)
  - [Resetting Waypoint Logging](#resetting-waypoint-logging)
  - [For Real Roboracer Car](#for-real-roboracer-car)
- [Visualization](#visualization)
- [Notes](#notes)

## Package Creation

location: `roboracer-ws/src$` 
```bash
ros2 pkg create simple_map --build-type ament_cmake --license MIT
```


## File Structure

Create a file structure matching the commit `85bcaa79f7b9e2851947586b3796900bd0503bd8`.

Update `CMakeLists.txt` and `package.xml` accordingly.

- `utils.py` contains helper functions.
- Main mapping and map generation logic is in `simple_map.launch.py`.

## Building the Package

After setting up the package and file structure, build the workspace:

location: `roboracer-ws`
```bash
colcon build
source install/setup.bash
```


## Mapping

**To create the map independently:**

location: `roboracer-ws`
```bash
ros2 launch simple_map simple_map.launch.py launch_waypoint_logger:=false
```


**To save the map:**

location: `roboracer-ws`
```bash
ros2 service call /save_map std_srvs/srv/Empty
```

- The map (`.pgm` and `.yaml` files) will be saved to the `maps` folder under `roboracer-ws`, similar to SLAM Toolbox.

## Waypoint Saving

### Simple Saving

Waypoints (points the car travels while Pure Pursuit is running) can be saved as follows.

**To run Pure Pursuit:**

location: `roboracer-ws`
```bash
ros2 launch pure_pursuit pure_pursuit.launch.py
```

### Centerline Waypoints

- Centerline is calculated using LiDAR, IMU, and IPS values.
- Used as a reference trajectory for [global_racetrajectory_optimization](https://github.com/TUMFTM/global_racetrajectory_optimization).

**To save waypoints and centerline waypoints without the map:**

location: `roboracer-ws`
```bash
ros2 launch simple_map simple_map.launch.py launch_simple_map:=false
```


- Waypoint logging starts when the lap count changes and stops when it changes again.

### Resetting Waypoint Logging

**To reset and start saving waypoints again:**

location: `roboracer-ws`
```bash
ros2 service call /reset_waypoint_logging std_srvs/srv/Empty
```


*(If you change the service name, update the command accordingly.)*

### For Real Roboracer Car

**To start saving waypoints:**

location: `roboracer-ws`
```bash
ros2 topic pub --once /autodrive/f1tenth_1/lap_count std_msgs/msg/Int32 "data: 1"
ros2 topic pub --once /autodrive/f1tenth_1/lap_count std_msgs/msg/Int32 "data: 2"
```


**To stop saving waypoints:**
location: `roboracer-ws`
```bash
ros2 topic pub --once /autodrive/f1tenth_1/lap_count std_msgs/msg/Int32 "data: 3"
```


## Visualization

**To visualize the saved waypoints:**

location: `roboracer-ws`
```bash
ros2 launch simple_map visualise.launch.py
```


- Topics published:
  - `/visualization/waypoints`
  - `/visualization/centerline_waypoints`

## Notes

- Files inside the `config` folder are placeholders.
- The actual saved waypoint files will be inside the `include` folder after building with `colcon build`.
- Saved waypoints will **not** be in the `config` folder.