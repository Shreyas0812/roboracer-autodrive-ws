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
cd ./AutoDRIVE/ADSS Toolkit/autodrive_ros2
```

Copy this folfer to the workspace/scr

```bash
roboracer-ws$ colcon build
```

Step 4:

Change scripts-dir to scripts_dir and install-scripts to install_scripts for deprecation warning,


This is inside 

```
roboracer-ws/src/autodrive_ros2/autodrive_f1tenth/setup.cfg
roboracer-ws/src/autodrive_ros2/autodrive_nigel/setup.cfg
```

