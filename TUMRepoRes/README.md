# Waypoint Generation Using global_racetrajectory_optimization

## Overview

This folder contains waypoint files generated using the [`global_racetrajectory_optimization`](https://github.com/AhmadAmine998/global_racetrajectory_optimization) repository (a fork of the original TUMFTM/global_racetrajectory_optimization). The waypoints correspond to three different trajectory optimization objectives:

- Shortest Path
- Minimum Curvature Path
- Minimum Time Path

## Steps Performed

1. **Added Custom Waypoint File**
   - Placed a waypoint file (generated using the Waterloo repo) into the `inputs/tracks` directory.

2. **Updated Track File Path**
   - Modified the track file path in `main_globaltraj.py` to reference the new waypoint file.

3. **Generated Trajectories**
   - Changed the `opt_type` parameter in `main_globaltraj.py` to each of the following and ran the code for each:
     - `shortest_path`
     - `mincurv`
     - `mintime`
   - Saved the resulting waypoints for each trajectory type.

4. **Extended Vehicle Dynamics Data**
   - Extended both `ax_max_machines.csv` and `ggv.csv` so their velocity columns reach up to 20 m/s, matching the `v_max` in `racecar.ini`. This prevents runtime errors and ensures the velocity profile can be computed for the entire speed range.

## Folder Contents

- Waypoint files for each trajectory type (Shortest Path, Minimum Curvature, Minimum Time)
- This README

## References

- https://github.com/AhmadAmine998/global_racetrajectory_optimization
- https://github.com/TUMFTM/global_racetrajectory_optimization

For reproducibility, ensure that your `ax_max_machines.csv` and `ggv.csv` files cover all velocities up to your specified `v_max` (20 m/s), and that your configuration files point to the correct track and vehicle data.
