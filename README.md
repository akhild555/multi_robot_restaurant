# Multi-Robot Waiter System for Restaurant Automation

Modified full planning and control stack for GRASP LML Service Robots @ Penn with restaurant system specific changes.

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
 For Waiter ID: 0-7
 ```
 devel/lib/control_stack/nav_node src/control_stack/config/sim_config0.lua 0
 devel/lib/control_stack/nav_node src/control_stack/config/sim_config1.lua 1
 devel/lib/control_stack/nav_node src/control_stack/config/sim_config2.lua 2
 devel/lib/control_stack/nav_node src/control_stack/config/sim_config3.lua 3
 devel/lib/control_stack/nav_node src/control_stack/config/sim_config4.lua 4
 devel/lib/control_stack/nav_node src/control_stack/config/sim_config5.lua 5
 devel/lib/control_stack/nav_node src/control_stack/config/sim_config6.lua 6
 devel/lib/control_stack/nav_node src/control_stack/config/sim_config7.lua 7
 ```
 - To run the patrons on the simulator, from inside `catkin_ws/` run: <br />
 For Patron ID: 0-16
 ```
 devel/lib/control_stack/patron_nav_node src/control_stack/config/sim_patron_config0.lua 8
 devel/lib/control_stack/patron_nav_node src/control_stack/config/sim_patron_config1.lua 9
 devel/lib/control_stack/patron_nav_node src/control_stack/config/sim_patron_config2.lua 10
 devel/lib/control_stack/patron_nav_node src/control_stack/config/sim_patron_config3.lua 11
 devel/lib/control_stack/patron_nav_node src/control_stack/config/sim_patron_config4.lua 12
 devel/lib/control_stack/patron_nav_node src/control_stack/config/sim_patron_config5.lua 13
 devel/lib/control_stack/patron_nav_node src/control_stack/config/sim_patron_config6.lua 14
 devel/lib/control_stack/patron_nav_node src/control_stack/config/sim_patron_config7.lua 15
 devel/lib/control_stack/patron_nav_node src/control_stack/config/sim_patron_config8.lua 16
 devel/lib/control_stack/patron_nav_node src/control_stack/config/sim_patron_config9.lua 17
 devel/lib/control_stack/patron_nav_node src/control_stack/config/sim_patron_config10.lua 18
 devel/lib/control_stack/patron_nav_node src/control_stack/config/sim_patron_config11.lua 19
 devel/lib/control_stack/patron_nav_node src/control_stack/config/sim_patron_config12.lua 20
 devel/lib/control_stack/patron_nav_node src/control_stack/config/sim_patron_config13.lua 21
 devel/lib/control_stack/patron_nav_node src/control_stack/config/sim_patron_config14.lua 22
 devel/lib/control_stack/patron_nav_node src/control_stack/config/sim_patron_config15.lua 23
 devel/lib/control_stack/patron_nav_node src/control_stack/config/sim_patron_config16.lua 24
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

- Alternatively, to run everything at once, from the root of the repo run:
```
run_all.sh
```

## License:

[MIT](../master/LICENSE)

