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


### Waypoint Saving

NOTE: The files inside config inside the project act as placeholders, the true files will be inside include folder after colcon build, the saved waypoints will be present there, not inside the config folder here. 

#### 1. Simple Saving of the waypoints (points car travels while pure pursuit is running)

##### Side Note: To run pure pursuit:

```bash
roboracer-ws$ ros2 launch pure_pursuit pure_pursuit.launch.py 
```

#### 2. Centerline -- Calculated using LiDAR and IMU and IPS Values -- Used as Reference Trajectory in https://github.com/TUMFTM/global_racetrajectory_optimization


To Save Waypoints along with CenterLine Waypoints without map:
```bash
roboracer-ws$ ros2 launch simple_map simple_map.launch.py launch_simple_map:=false
```

This starts saving waypoints when the lap changes, and stops changing when the lap changes again. 

To reset the variable and save again at any point: (Note: if you change the name of the service, this command changes too)
```bash
ros2 service call /reset_waypoint_logging std_srvs/srv/Empty
```

For Real Roboracer Car:

To start saving waypoints
```bash
ros2 topic pub /autodrive/f1tenth_1/lap_count std_msgs/msg/Int32 "data: 1"
```
followed by:
```bash
ros2 topic pub /autodrive/f1tenth_1/lap_count std_msgs/msg/Int32 "data: 2"
```

To stop saving waypoints
```bash
ros2 topic pub /autodrive/f1tenth_1/lap_count std_msgs/msg/Int32 "data: 3"
```

To Visualize the saved waypoints: 
```bash
roboracer-ws$ ros2 launch simple_map visualise.launch.py 
```

Note: Topics Published to: <br> 
/visualization/waypoints <br>
/visualization/centerline_waypoints





