# Multi-Agent Planning Packages
**IMPORTANT: this code base is not to be shared with anyone outside the lab.**

The packages have been tested on **Ubuntu 22.04**, **ROS2 Humble**.
To get started you can skip to [Getting Started](#Getting-Started). This repo contains the following packages:
* [convex_decomp_util](#convex_decomp_util): package for Safe Corridor generation based on [[1]](#1) and [[2]](#2).
* [decomp_ros](#decomp_ros): package for Sage Corridor generation and visualization based on [[3]](#3).
* [env_builder](#env_builder): ROS2 package that allows to build an evironment in the form of voxel grid and publishes it in the form of a pointcloud for visualization in rviz2.
* [jps3d](#jps3d): a modified version of [jps3d](https://github.com/KumarRobotics/jps3d) that checks for traversibilty when generating a path to make sure we can generate a Safe Corridor around it.
* [mapping_util](#mapping_util): ROS2 package for voxel grid generation (clearing out voxels that are in the field of view of the drone) - TODO.
* [path_finding_util](#path_finding_util): package for path finding and path tools such as path shortening.
* [voxel_grid_util](#voxel_grid_util): package for voxel grid class and raycasting function.
* [multi_agent_planner](#multi_agent_planner): ROS2 package for multi-agent planning (uses all the other packages).

At the end of this documentation you can find:
* [General Comments](#General-Comments): general comments about using the packages.
* [Improvements](#Improvements): improvements to the packages that are yet to be implemented.
* [References](#References): references used throughout this text.

## Getting Started
### Install gurobi
Download gurobi 10.0.* from this [link](https://www.gurobi.com/downloads/gurobi-software/). Follow the installation instructions in this [link](https://support.gurobi.com/hc/en-us/articles/4534161999889-How-do-I-install-Gurobi-Optimizer-). Finally, install the license by going to this [link](https://portal.gurobi.com/), creating a license and installing it (instructions on how to install it are shown when you create it).

Then, build gurobi and copy the library:
``` shell script
cd /opt/gurobi1002/linux64/src/build  #Note that the name of the folder gurobi1002 changes according to the Gurobi version
sudo make
sudo cp libgurobi_c++.a ../../lib/
```

### Create and build workspace
Create a ROS2 workspace and clone the repo inside the `src` folder of the workspace (or simply clone it inside an existing workspace), then build it: 
``` shell script
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/lis-epfl/multi_agent_pkgs
cd ..
colcon build --symlink-install --packages-select jps3d decomp_util decomp_ros_msgs convex_decomp_util path_finding_util voxel_grid_util env_builder_msgs env_builder mapping_util multi_agent_planner_msgs multi_agent_planner
```
For the visualization of the Safe Corridor in rviz2, you need in addition to build  `decomp_ros_util` but it has issues for now (conflicting OGRE installations between the vendor version used by rviz2 and the system wide installation). **It is not necessary for the functioning of all the other packages**.
``` shell script
colcon build --symlink-install --packages-select decomp_ros_util 
```

## Running the simulation
### Change the config parameters
You can change the following config files or use them as a basis to create other config files according to your application:
* `env_default_config.yaml`: default config file for obstacles generation in the environment in `env_builder/config`.
* `agent_default_config.yaml`: default config file for the planner parameters in `multi_agent_planner/config`.

When you launch `env_builder.launch` it uses `env_default_config.yaml` and when you launch any launch file from the package `multi_agent_planner` it uses `agent_default_config.yaml`. Some parameters are then changed in each launch file according to its purpose.

### Single agent
Launch rviz2 in a terminal (if you didn't build `decomp_ros_util` due to OGRE conflicts, the polyhedra will not appear).
``` shell script
cd ~/ros2_ws
. install/setup.bash
rviz2 -d ~/ros2_ws/src/multi_agent_pkgs/multi_agent_planner/rviz/rviz_config_multi.rviz
```
Launch the environment in another window:
``` shell script
cd ~/ros2_ws
. install/setup.bash
ros2 launch env_builder env_builder.launch.py
```
Launch the planner in another window:
``` shell script
cd ~/ros2_ws
. install/setup.bash
ros2 launch multi_agent_planner agent_planner.launch.py
```

### Multiple agents in a circular configuration
Launch rviz2 in a terminal (if you didn't build `decomp_ros_util` due to OGRE conflicts, the polyhedra will not appear).
``` shell script
cd ~/ros2_ws
. install/setup.bash
rviz2 -d ~/ros2_ws/src/multi_agent_pkgs/multi_agent_planner/rviz/rviz_config_multi.rviz
```
Launch the environment in another window (if you want an empty environement replace `env_builder.launch.py` with `env_builder_empty.launch.py`):
``` shell script
cd ~/ros2_ws
. install/setup.bash
ros2 launch env_builder env_builder.launch.py
```
Launch the agents in another window. If you want each agent to run in a different termnial, uncomment the `prefix=['xterm -fa default -fs 10 -hold -e']` line in the launch file:
``` shell script
cd ~/ros2_ws
. install/setup.bash
ros2 launch multi_agent_planner multi_agent_planner_circle.launch.py
```

### Multiple agents going through a small window
Launch rviz2 in a terminal (if you didn't build `decomp_ros_util` due to OGRE conflicts, the polyhedra will not appear).
``` shell script
cd ~/ros2_ws
. install/setup.bash
rviz2 -d ~/ros2_ws/src/multi_agent_pkgs/multi_agent_planner/rviz/rviz_config_multi.rviz
```
Launch the environment in another window:
``` shell script
cd ~/ros2_ws
. install/setup.bash
ros2 launch env_builder env_builder_window.launch.py
```
Launch the agents in another window. If you want each agent to run in a different termnial, uncomment the `prefix=['xterm -fa default -fs 10 -hold -e']` line in the launch file:
``` shell script
cd ~/ros2_ws
. install/setup.bash
ros2 launch multi_agent_planner multi_agent_planner_window.launch.py
```

## convex_decomp_util
TODO

## decomp_ros
TODO

## env_builder
TODO

## jps3d
TODO

## mapping_util
TODO

## path_finding_util
TODO

## voxel_grid_util
TODO

## multi_agent_planner
TODO

## General Comments
TODO

## Improvements
TODO

## References
<a id="1">[1]</a>
Toumieh, C. and Lambert, A., 2022. Voxel-grid based convex decomposition of 3d space for safe corridor generation. Journal of Intelligent & Robotic Systems, 105(4), p.87.

<a id="2">[2]</a>
Toumieh, C. and Lambert, A., 2022. Shape-aware Safe Corridors Generation using Voxel Grids. arXiv preprint arXiv:2208.06111

<a id="3">[3]</a>
Liu, S., Watterson, M., Mohta, K., Sun, K., Bhattacharya, S., Taylor, C.J. and Kumar, V., 2017. Planning dynamically feasible trajectories for quadrotors using safe flight corridors in 3-d complex environments. IEEE Robotics and Automation Letters, 2(3), pp.1688-1695.
