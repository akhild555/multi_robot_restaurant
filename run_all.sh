#!/bin/sh
roscore &
rosbuild_ws/simulator/ut_multirobot_sim/bin/simulator --sim_config=rosbuild_ws/simulator/sim_config.lua &
cd catkin_ws
devel/lib/control_stack/nav_node src/control_stack/config/sim_config0.lua 0 &
cd catkin_ws
devel/lib/control_stack/nav_node src/control_stack/config/sim_config1.lua 1 &
cd catkin_ws
devel/lib/control_stack/nav_node src/control_stack/config/sim_config2.lua 2 &
cd catkin_ws
devel/lib/control_stack/nav_node src/control_stack/config/sim_config3.lua 3 &
# cd catkin_ws
# devel/lib/control_stack/nav_node src/control_stack/config/sim_config4.lua 4 &
# cd catkin_ws
# devel/lib/control_stack/nav_node src/control_stack/config/sim_config5.lua 5 &
# cd catkin_ws
# devel/lib/control_stack/nav_node src/control_stack/config/sim_config6.lua 6 &
# cd catkin_ws
# devel/lib/control_stack/nav_node src/control_stack/config/sim_config7.lua 7 &
# cd catkin_ws
# devel/lib/control_stack/patron_nav_node src/control_stack/config/sim_patron_config0.lua 8 &
# cd catkin_ws
# devel/lib/control_stack/patron_nav_node src/control_stack/config/sim_patron_config1.lua 9 &
# cd catkin_ws
# devel/lib/control_stack/patron_nav_node src/control_stack/config/sim_patron_config2.lua 10 &
# cd catkin_ws
# devel/lib/control_stack/patron_nav_node src/control_stack/config/sim_patron_config3.lua 11 &
# cd catkin_ws
# devel/lib/control_stack/patron_nav_node src/control_stack/config/sim_patron_config4.lua 12 &
# cd catkin_ws
# devel/lib/control_stack/patron_nav_node src/control_stack/config/sim_patron_config5.lua 13 &
# cd catkin_ws
# devel/lib/control_stack/patron_nav_node src/control_stack/config/sim_patron_config6.lua 14 &
# cd catkin_ws 
# devel/lib/control_stack/patron_nav_node src/control_stack/config/sim_patron_config7.lua 15 &
# cd catkin_ws
# devel/lib/control_stack/patron_nav_node src/control_stack/config/sim_patron_config8.lua 16 &
cd catkin_ws
devel/lib/control_stack/patron_nav_node src/control_stack/config/sim_patron_config9.lua 17 &
cd catkin_ws
devel/lib/control_stack/patron_nav_node src/control_stack/config/sim_patron_config10.lua 18 &
cd catkin_ws 
devel/lib/control_stack/patron_nav_node src/control_stack/config/sim_patron_config11.lua 19 &
cd catkin_ws
devel/lib/control_stack/patron_nav_node src/control_stack/config/sim_patron_config12.lua 20 &
cd catkin_ws
devel/lib/control_stack/patron_nav_node src/control_stack/config/sim_patron_config13.lua 21 &
cd catkin_ws
devel/lib/control_stack/patron_nav_node src/control_stack/config/sim_patron_config14.lua 22 &
cd catkin_ws
devel/lib/control_stack/patron_nav_node src/control_stack/config/sim_patron_config15.lua 23 &
cd catkin_ws
devel/lib/control_stack/patron_nav_node src/control_stack/config/sim_patron_config16.lua 24 &
cd catkin_ws
devel/lib/restaurant_state/environment_manager &
cd catkin_ws
devel/lib/task_allocator/job_assignment &
cd ..
rosrun rviz rviz -d rosbuild_ws/simulator/visualization.rviz &

wait
echo all processes complete
