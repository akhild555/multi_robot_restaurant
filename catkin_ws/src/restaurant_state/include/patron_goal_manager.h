#pragma once

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <control_stack/PatronGoal.h>
#include "control_stack/KitchenOrders.h"
#include "control_stack/RobotDatabase.h"
#include "json.h"
#include <fstream>
#include <random>
#include <numeric>
#include <string>

struct Patron_Status {
  int patron_id = 0;
  int table_number = 0;
  int goal_location = 0;
};

class PatronManager {
    
    ros::Subscriber patron_status_subscriber;
    ros::Publisher patron_assignment;

    nlohmann::json mon_restaurant_config;

    void PatronStatusCallback(const control_stack::RobotDatabase& msg)
    {
        int test = 5;

        // check if patron is free and assign order/table to patron
    }

public:

    /**
    * Constructor
    */ 
    PatronManager(ros::NodeHandle& nh) {

    std::ifstream file_input("src/control_stack/config/monarch_config.json");
    file_input >> mon_restaurant_config;

    ros::Publisher patron_assignment = nh.advertise<control_stack::PatronGoal>("/patron_goal", 1000);
    ROS_INFO("Patron manager ready");

    patron_status_subscriber = nh.subscribe("/patron_database",
            1, &PatronManager::PatronStatusCallback, this);
    }

};