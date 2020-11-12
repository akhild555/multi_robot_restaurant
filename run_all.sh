#!/bin/sh

rosbuild_ws/simulator/ut_multirobot_sim/bin/simulator --sim_config=rosbuild_ws/simulator/sim_config.lua &
cd catkin_ws
devel/lib/control_stack/nav_node src/control_stack/config/sim_config0.lua 0 &
cd catkin_ws
devel/lib/control_stack/nav_node src/control_stack/config/sim_config1.lua 1 &
cd catkin_ws
devel/lib/control_stack/nav_node src/control_stack/config/sim_config2.lua 2 &
# cd catkin_ws
# devel/lib/control_stack/nav_node --src/control_stack/config/sim_config3.lua 3 &
# cd catkin_ws
# devel/lib/control_stack/nav_node --src/control_stack/config/sim_config4.lua 4 &
cd ..
rosrun rviz rviz -d rosbuild_ws/simulator/visualization.rviz &

wait
echo all processes complete
