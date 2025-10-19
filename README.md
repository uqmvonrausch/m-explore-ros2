# m-explore ROS2 port

ROS2 package port for multi-robot autonomous exploration of [m-explore](https://github.com/hrnr/m-explore). Forked and adapted for frontier exploration for METR4202. No adjustments have been made to the `costmap_client.cpp` or `frontier_search.cpp` files. 

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
This node has been redesigned to run with the `turtlebot` package. It is started from the launch file in this package. 

To run with a params file just run it with
```bash
ros2 run explore_lite explore --ros-args --params-file <path_to_ros_ws>/m-explore-ros2/explore/config/params.yaml
```

A launch file is provided in the `launch` directory. By default, this launch file utilises `params.yaml` as a set of input parameters. 

```bash
ros2 launch explore_lite explore.launch.py
```

Features / Adaptations
-------
#### Frontier Selection
The existing `frontier_search.cpp` file contains a frontier search class that queries the provided `costmap_topic` parameter and extracts contiguous regions of boarder cells (free space cells which boarder unknown cells). These are returned in an ordered list, sorted by their distance from the robot and their size. In this way, the search is incentivised to pursue large unexplored areas. 

#### Stop/Resume exploration
By default the exploration node will start right away the frontier-based exploration algorithm. Alternatively, you can stop the exploration by publishing to a `False` to `explore/resume` topic. This will stop the exploration and the robot will stop moving. You can resume the exploration by publishing to `True` to `explore/resume`.

#### Costmap Goal Checking
The node will subscribe to the `/local_costmap/costmap` and `/global_costmap/costmap` occupany grids, and query these objects when setting a new goal. If either of these costmaps exceed a value of 80, the node will check a 16 x 16 grid (resolution 0,05m) around the goal for a candidate position that does not exceed this value. 

#### Target extension
The node will check selected frontier goals for their proximity to the robot. If the target goal is within `target_prox_lim` (see config files) in both the x and y directions, the target will be extended `recovery_delta` metres away from the robot in the direction of the original target goal. This prevents 

#### Goal progress
The node now logs which frontier centroid it is pursuing and times out if pursuing this centroid for longer than `progress_timeout` without progress. This is a necessary change given the goal point for a given frontier centroid can be adjusted from planning pass to planning pass by the node. Additionally, the node detects the goal result from the move base server and blacklists nearby goals that could not be achieved (within 0.5m). Otherwise, the goal is abandoned and planning re-attempted. 