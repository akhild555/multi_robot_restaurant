#include "ros/ros.h"
#include "robot_data_manager.h"
#include "kitchen_manager.h"
#include "std_msgs/String.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "environment_manager_node");
  ros::NodeHandle nh;
  ros::Rate loop_rate(1);

  int number_of_robots = 4;
  int number_of_tables = 17;
  int counter = 0;

  KitchenManager kitchen_manager(nh);
  RobotDataManager data_manager(nh, number_of_robots);
  
  while (ros::ok())
  {    
    kitchen_manager.randOrderGenerator(number_of_tables, counter); // randomly generate order
    ros::spinOnce();
    loop_rate.sleep();
    counter++;
  }
  return 0;
  
}