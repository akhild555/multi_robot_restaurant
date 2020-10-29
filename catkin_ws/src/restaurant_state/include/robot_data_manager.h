#pragma once

#include "ros/ros.h"
#include <control_stack/RobotPosition.h>
#include <control_stack/RobotDatabase.h>
#include "std_msgs/String.h"


class RobotDataManager {

  ros::Publisher robot_data_publisher;
  ros::Subscriber robot_position_subscriber;
  control_stack::RobotDatabase robot_database;

  int number_of_robots;

  /**
   * Callback to handle position msg from each robot
   */
  void robotPositionCallback(const control_stack::RobotPosition& msg) {

    int index = msg.robot_index;
    if(index >= number_of_robots || index < 0) {
      ROS_ERROR("Recieved invalid index from /robot_position topic");
      return;
    }
    robot_database.robot_data[index] = msg;
    publishRobotDatabase();
  }

  /**
   * Publishes data base 
   */
  void publishRobotDatabase() {
    robot_database.stamp = ros::Time::now();
    robot_data_publisher.publish(robot_database);
  }


public:

  /**
   * Constructor
   */ 
  RobotDataManager(ros::NodeHandle& nh, int num) {

    number_of_robots = num;
    
    // initialize database 
    for(int i=0; i<number_of_robots; i++) {
      control_stack::RobotPosition robot_data;
      robot_data.stamp = ros::Time(0);
      robot_data.robot_index = i;
      robot_database.robot_data.push_back(robot_data);
    }
    
    robot_position_subscriber = nh.subscribe("/robot_position", 100, &RobotDataManager::robotPositionCallback, this);
    robot_data_publisher =  nh.advertise<control_stack::RobotDatabase>("/robot_database", 100);
    ROS_INFO("Robot data manager ready");
  }
};