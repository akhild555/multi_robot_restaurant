#include "ros/ros.h"
#include "robot_data_manager.h"
#include "std_msgs/String.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "environment_manager_node");
  ros::NodeHandle nh;
  
  int number_of_robots = 2;

  RobotDataManager data_manager(nh, number_of_robots);
  ros::spin();
  return 0;
  
}