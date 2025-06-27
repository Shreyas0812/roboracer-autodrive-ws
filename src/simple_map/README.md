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
  - [Saving a Copy of Waypoints](#saving-a-copy-of-waypoints)
  - [For Real Roboracer Car](#for-real-roboracer-car)
- [Visualization](#visualization)
- [Notes](#notes)

## Package Creation

location: `roboracer-ws/src$` 
```bash
ros2 pkg create simple_map --build-type ament_cmake --license MIT
```


## File Structure

Ensure your file structure matches the [reference commit](https://github.com/Shreyas0812/roboracer-autodrive-ws/commit/85bcaa79f7b9e2851947586b3796900bd0503bd8).

- `utils.py`: Helper functions.
- Main mapping and map generation logic is in `simple_map.launch.py`.
- Configuration files are in `config/`.
- Python nodes are in `scripts/`.

## Building the Package

After setting up the package and file structure, build the workspace:

location: `roboracer-ws`
```bash
colcon build
source install/setup.bash
```

location: `roboracer-ws`
> **Tip:** If you restructure packages or interfaces, clean the workspace first:
> ```
> rm -rf build/ install/ log/
> ```

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

### Saving a copy of waypoints

> **Note:** Change the destination paths below as needed.

**To save waypoints to a location of users choice**

location: `roboracer-ws`
```bash
ros2 service call /save_waypoints roboracer_interfaces/srv/SaveWaypoints "{source_filename: 'waypoints.csv', destination_filepath: '/home/shreyas/Documents/waypoints.csv'}"

ros2 service call /save_waypoints roboracer_interfaces/srv/SaveWaypoints "{source_filename: 'centerline_waypoints.csv', destination_filepath: '/home/shreyas/Documents/centerline_waypoints.csv'}"
```

- The `/save_waypoints` service is provided by the `waypoint_saver_service.py` node in `simple_map`.
- Make sure this node is running before calling the service.

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


- Visualization topics:
  - `/visualization/waypoints`
  - `/visualization/centerline_waypoints`
- Use RViz2 to visualize these topics. You can add them as `MarkerArray` displays in RViz.

## Notes

- Files in the `config` folder are placeholders for initial configuration.
- **Saved waypoints** (CSV files) are written to the location you specify when calling the `/save_waypoints` service.
- Saved waypoints will **not** appear in the `config` folder unless you explicitly save them there.
- The service node (`waypoint_saver_service.py`) must be running for `/save_waypoints` to work.
- If you change the service name or node, update the commands above accordingly.
- If you do not save the waypoints to another location, rebuilding the workspace will overwrite previous waypoint files. For example, if you build again before visualization, it will re-write the waypoints.


**For troubleshooting and more details, see the code comments and launch files.**
