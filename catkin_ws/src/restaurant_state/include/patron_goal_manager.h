#pragma once

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <control_stack/PatronGoal.h>
#include "control_stack/KitchenOrders.h"
#include "control_stack/PatronDatabase.h"
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
    control_stack::PatronGoal patron_assgn;
    geometry_msgs:: Twist t;

    void PatronStatusCallback(const control_stack::PatronDatabase& msg)
    {
        patron_assgn.patron_index = 5;
        t.linear.x = mon_restaurant_config["Table_1"]["patron"]["x"];
        t.linear.y = mon_restaurant_config["Table_1"]["patron"]["y"];
        patron_assgn.patron_goal.push_back(t);

        // check if patron is free and assign order/table to patron
        // for (const control_stack::RobotPosition& robot_msg : msg.robot_data) {
            
        // }
        
        publishPatronGoal();
        
    }

    void publishPatronGoal() {
        patron_assgn.stamp = ros::Time::now();
        patron_assignment.publish(patron_assgn);
    } 

public:

    /**
    * Constructor
    */ 
    PatronManager(ros::NodeHandle& nh) {

    std::ifstream file_input("src/control_stack/config/monarch_config.json");
    file_input >> mon_restaurant_config;

    patron_assignment = nh.advertise<control_stack::PatronGoal>("/patron_goal", 1000);
    ROS_INFO("Patron manager ready");

    patron_status_subscriber = nh.subscribe("/patron_database",
            1, &PatronManager::PatronStatusCallback, this);
    }

};