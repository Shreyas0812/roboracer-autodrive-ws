# Steps Followed:

### To Create this package

```bash
roboracer-ws/src$ ros2 pkg create simple_map --build-type ament_cmake --license MIT
```

Create a file structure which looks like the one creted with this commit. --> 85bcaa79f7b9e2851947586b3796900bd0503bd8

Update CMakeLists.txt and package.xml

utils.py has some helper function, main update for map and generation map inside simple_map.launch.py

### Mapping -- creates pgm and yaml files the way SLAM toolbox creates

To create the map independently:
```bash
roboracer-ws$ ros2 launch simple_map simple_map.launch.py launch_waypoint_logger:=false
```

To Save the map:
```bash
roboracer-ws$ ros2 service call /save_map std_srvs/srv/Empty
```

This saves a map called simple_map to maps folder under roboracer-ws


### Waypoint Logging

#### 1. Simple Logging of the waypoints (points car travels while pure pursuit is running)

##### Side Note: To run pure pursuit:

```bash
roboracer-ws$ ros2 launch pure_pursuit pure_pursuit.launch.py 
```

TO Log Waypoints without map:
```bash
roboracer-ws$ ros2 launch simple_map simple_map.launch.py launch_simple_map:=false
```

#### 2. Centerline -- Calculated using LiDAR and IMU and IPS Values -- Used as Reference Trajectory in https://github.com/TUMFTM/global_racetrajectory_optimization

```bash
TODO: # Command to calculate the centerline to be added here
```