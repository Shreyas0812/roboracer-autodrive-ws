Step 1 : Creating a new package
```bash
$ cd roboracer_ws/src/
roboracer_ws/src/$ ros2 pkg create wall_follow_ui_control_node --build-type ament_cmake
```

Step 2: Making package ready for cpp and python nodes

```bash
roboracer-ws/src/wall_follow_ui_control$ touch src/wall_follow_ui_control_node.cpp
roboracer-ws/src/wall_follow_ui_control$ touch include/wall_follow_ui_control/demo_header.hpp

roboracer-ws/src/wall_follow_ui_control$ mkdir wall_follow_ui_control
roboracer-ws/src/wall_follow_ui_control$ touch wall_follow_ui_control/__init__.py
roboracer-ws/src/wall_follow_ui_control$ mkdir scripts
roboracer-ws/src/wall_follow_ui_control$ touch scripts/wall_follow_ui_control_node.py
```
Add the shebang line to `wall_follow_ui_control_node.py` so it looks like:

```bash
roboracer-ws/src/wall_follow_ui_control$ cat scripts/wall_follow_ui_control_node.py 
#!/usr/bin/env python3
```

```bash
roboracer-ws/src/wall_follow_ui_control$ mkdir launch
roboracer-ws/src/wall_follow_ui_control$ touch launch/wall_follow_ui_control.launch.py
```


```bash
roboracer-ws$ ros2 launch wall_follow_ui_control wall_follow_ui_control.launch.py 
```
