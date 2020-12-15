#include "ros/ros.h"
#include "robot_data_manager.h"
#include "kitchen_manager.h"
#include "patron_goal_manager.h"
#include "patron_data_manager.h"
#include "data_logger.h"
#include "std_msgs/String.h"
#include "table_state.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "environment_manager_node");
  ros::NodeHandle nh;
  ros::Rate loop_rate(1);

  int number_of_robots = 4;
  int number_of_tables = 17;
  int number_of_patrons = 4;
  int counter = 1;
  ROS_INFO("Starting environment manager with %d robots and %d tables", number_of_robots, number_of_tables);

  KitchenManager kitchen_manager(nh, number_of_tables);
  RobotDataManager data_manager(nh, number_of_robots);
  PatronDataManager patron_data_manager(nh, number_of_patrons);
  PatronManager patron_goal_manager(nh, number_of_patrons);
  DataLogger data_logger(nh);
  tableVisualize table_visualizer(nh, number_of_robots);
  
  while (ros::ok())
  {    
    kitchen_manager.updateCounter(counter);
    table_visualizer.publishGlobalGoals();
    ros::spinOnce();
    loop_rate.sleep();
    counter++;
  }
  return 0;
  
}