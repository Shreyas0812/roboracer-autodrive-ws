# Steps Followed:

### To Create this package

```bash
roboracer-ws/src$ ros2 pkg create simple_map --build-type ament_cmake --license MIT
```

Create a file structure which looks like the one creted with this commit. --> 85bcaa79f7b9e2851947586b3796900bd0503bd8

Update CMakeLists.txt and package.xml

utils.py has some helper function, main update for map and generation map inside simple_map.launch.py

To create the map: 
```bash
roboracer-ws$ ros2 launch simple_map simple_map.launch.py 
```

To Save the map:
```bash
roboracer-ws$ ros2 service call /save_map std_srvs/srv/Empty
```

This saves a map called simple_map to maps folder