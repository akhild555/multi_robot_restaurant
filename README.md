# Service Robot Control Stack

Full planning and control stack for GRASP LML Service Robots @ Penn.

[![Build Status](https://travis-ci.com/kylevedder/ServiceRobotControlStack.svg?branch=master)](https://travis-ci.com/kylevedder/ServiceRobotControlStack)

## Requirements

 - ROS Melodic
 - Clone using `--recurse-submodules`
 - URG ROS Package (for deployment on real hardware)

## Setup

Currently, only ROS Melodic running on *buntu 18.04 is supported. Running the code on other distributions of the OS or ROS will require at least minor changes to the build system.

 - Install all tools and packages
   - Install [ROS Melodic](http://wiki.ros.org/melodic/Installation)
   - Run `./InstallPackages`
 - Setup commit hooks
   - Run `./ci/setup_hooks.sh` in the root of the repo
   
## Usage
 - To run the simulator, from the root of the repo run:
 ```
 rosbuild_ws/simulator/ut_multirobot_sim/bin/simulator --sim_config=rosbuild_ws/simulator/sim_config.lua
 ```
 
 - To run the navigation stack on the simulator, from inside `catkin_ws/` run:
 ```
 devel/lib/control_stack/nav_node src/control_stack/config/sim_config#.lua #
 ```
 You may replace the `#` with any robot ID supported by the simulator configuration. You may run several nav stacks with different robot IDs simultaneously, which will allow for multiple agents to run in the same simulator. <br />
 For Robot ID: 0-3
 ```
 devel/lib/control_stack/nav_node src/control_stack/config/sim_config0.lua 0
 devel/lib/control_stack/nav_node src/control_stack/config/sim_config1.lua 1
 devel/lib/control_stack/nav_node src/control_stack/config/sim_config2.lua 2
 devel/lib/control_stack/nav_node src/control_stack/config/sim_config3.lua 3
 ```
 
 
 - To run the restaurant state on the simulator, from inside `catkin_ws/` run:
 ```
 devel/lib/restaurant_state/environment_manager
 ```
 - To run the task allocator on the simulator, from inside `catkin_ws/` run:
 ```
 devel/lib/task_allocator/job_assignment
 ```
 
 - To view the nav stack running in simulation, from the root of the repo run:
```
rosrun rviz rviz -d rosbuild_ws/simulator/visualization.rviz
```

## License:

[MIT](../master/LICENSE)

