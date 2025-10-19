# m-explore ROS2 port

ROS2 package port for multi-robot autonomous exploration of [m-explore](https://github.com/hrnr/m-explore). Forked and adapted for frontier exploration for METR4202. 

### Contents
1. Building
2. Running
3. Features / Adaptations

Building
--------

Build as a standard colcon package. There are no special dependencies needed.

```bash
colcon build --packages-select explore_lite
```

Running
-------
To run with a params file just run it with
```
ros2 run explore_lite explore --ros-args --params-file <path_to_ros_ws>/m-explore-ros2/explore/config/params.yaml
```

Features / Adaptations
-------
#### Stop/Resume exploration
By default the exploration node will start right away the frontier-based exploration algorithm. Alternatively, you can stop the exploration by publishing to a `False` to `explore/resume` topic. This will stop the exploration and the robot will stop moving. You can resume the exploration by publishing to `True` to `explore/resume`.

#### Costmap Goal Checking
The node will subscribe to the `/local_costmap/costmap` and `/global_costmap/costmap` occupany grids, and query these objects when setting a new goal. If either of these costmaps exceed a value of 80, the node will check a 16 x 16 grid (resolution 0,05m) around the goal for a candidate position that does not exceed this value. 

#### Target extension
The node will check selected frontier goals for their proximity to the robot. If the target goal is within `target_prox_lim` (see config files) in both the x and y directions, the target will be extended `recovery_delta` metres away from the robot in the direction of the original target goal. This prevents 

#### Goal progress
The node now logs which frontier centroid it is pursuing and times out if pursuing this centroid for longer than `progress_timeout` without progress. This is a necessary change given the goal point for a given frontier centroid can be adjusted from planning pass to planning pass by the node. 