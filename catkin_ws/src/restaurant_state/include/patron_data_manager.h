#pragma once

#include "ros/ros.h"
#include <control_stack/PatronPosition.h>
#include <control_stack/PatronDatabase.h>
#include "std_msgs/String.h"


class PatronDataManager {

  ros::Publisher patron_data_publisher;
  ros::Subscriber patron_position_subscriber;
  control_stack::PatronDatabase patron_database;

  int number_of_patrons;
  int patron_start_ind;

  /**
   * Callback to handle position msg from each robot
   */
  void patronsPositionCallback(const control_stack::PatronPosition& msg) {

    int index = msg.patron_index;
    if(index >= number_of_patrons + patron_start_ind || index < 0) {
      ROS_ERROR("Recieved invalid index from /patron_position topic");
      return;
    }
    patron_database.patron_data[index-patron_start_ind] = msg;
    publishPatronDatabase();
  }

  /**
   * Publishes data base 
   */
  void publishPatronDatabase() {
    patron_database.stamp = ros::Time::now();
    patron_data_publisher.publish(patron_database);
  }


public:

  /**
   * Constructor
   */ 
  PatronDataManager(ros::NodeHandle& nh, int num_patrons, int patron_start_index) {

    number_of_patrons = num_patrons;
    patron_start_ind = patron_start_index;
    
    // initialize database 
    for(int i=patron_start_ind; i < (number_of_patrons + patron_start_ind); i++) {
      control_stack::PatronPosition patron_data;
      patron_data.stamp = ros::Time(0);
      patron_data.patron_index = i;
      patron_database.patron_data.push_back(patron_data);
    }
    
    patron_position_subscriber = nh.subscribe("/patron_position", 100, &PatronDataManager::patronsPositionCallback, this);
    patron_data_publisher =  nh.advertise<control_stack::PatronDatabase>("/patron_database", 100);
    ROS_INFO("Patron data manager ready");
  }
};