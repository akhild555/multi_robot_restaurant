#pragma once

#include "ros/ros.h"
#include <control_stack/RobotPosition.h>
#include <control_stack/RobotDatabase.h>
#include "std_msgs/String.h"

// #include <Eigen/Core.h>

#include <util/visualization.h>

#include <string>
#include <tuple>
#include <vector>

#include "json.h"

using json = nlohmann::json;
using namespace std;

class tableVisualize{
    
    int num_robots;
    ros::NodeHandle nh;
    std::vector<ros::Publisher> table_goal_publishers;
    ros::Subscriber tasks_subscriber;
    ros::Subscriber sub_robot_database;
    
    std::vector<int> robot_table_data = {};
    std::vector<bool> order_type_data = {};
    json mon_restaurant_config;
    json robot_configs;
    control_stack::RobotGoal robot_goal;
    vector<int> robot_active_status = {};

    // void getTableData(json& mon_restaurant_config){
    //     std::cout<<"input of json file";
    //     std::ifstream file_input("src/control_stack/config/monarch_config.json");
    //     file_input >> mon_restaurant_config;
    // }
    
    void RobotGoalCallBack(const control_stack::RobotGoal &msg){
        for(int i=0;i<num_robots;i++){
            // ROS_INFO("Table %d for robot %d",robot_table_data[i],i);
            if(i==msg.robot_index){
                if(robot_table_data[i] != msg.table_number){
                    robot_table_data[i] = msg.table_number;
                    order_type_data[i] = msg.order;
                }
            }
        }
        return;
    }
    void RobotDatabaseCallBack(const control_stack::RobotDatabase &msg){
        for(int i=0; i<num_robots;i++){
            // ROS_INFO("Active Status %d for robot %d",msg.robot_data[i].robot_active,i);
            if(msg.robot_data[i].robot_active == false){
            //     robot_table_data[i] = -1;
            //     order_type_data[i] = true;
                robot_active_status[i] = -1;
            }
            else
                robot_active_status[i] = 1;
            // ROS_INFO("Active Status %d for robot %d",robot_active_status[i],i);
        }
        return;
    }
    public:
    //constructor
    tableVisualize(ros::NodeHandle n, int no_of_robots){
        nh = n;
        tasks_subscriber = nh.subscribe("/robot_goal", 10, &tableVisualize::RobotGoalCallBack, this);
        sub_robot_database = nh.subscribe("/robot_database",10, &tableVisualize::RobotDatabaseCallBack, this);
        num_robots = no_of_robots;
        robot_table_data.resize(num_robots);
        order_type_data.resize(num_robots);
        table_goal_publishers.resize(num_robots);
        robot_active_status.resize(num_robots);
        for(int i=0;i<num_robots;i++){
            const std::string pub_sub_prefix_no_slash = "robot" + std::to_string(i);
            const std::string pub_sub_prefix = "/" + pub_sub_prefix_no_slash;
            robot_table_data[i] = -1;
            order_type_data[i] = true;
            robot_active_status[i] = -1;
            table_goal_publishers[i] = nh.advertise<visualization_msgs::Marker>(pub_sub_prefix + "/table_goal", 1);
        }
        std::ifstream robot_config_data("src/restaurant_state/config/robot_config.json");
        robot_config_data>>robot_configs;
        std::ifstream file_input("src/control_stack/config/monarch_config.json");
        file_input >> mon_restaurant_config;
    }
    

    void publishGlobalGoals(){

        for (int robot=0;robot<num_robots;robot++){
            const std::string pub_sub_prefix_no_slash = "robot" + std::to_string(robot);
            const std::string pub_sub_prefix = "/" + pub_sub_prefix_no_slash;
            // ROS_INFO("Robot %d is active: %d", robot,robot_active_status[robot]]));
            if(robot_table_data[robot]!=-1 && robot_active_status[robot]!=-1){
                ROS_INFO("Robot %d is active: %d", robot,robot_active_status[robot]);
                // ROS_INFO("robot ID: %d",robot);
                // ROS_INFO("Publishing to topic : %s", pub_sub_prefix.c_str());
                // ROS_INFO("Table %d for robot %d", (robot_table_data[robot],robot));
                
                std::string table_num = "Table_" + std::to_string(robot_table_data[robot]);
                std::string robot_num = "Robot_" + std::to_string(robot);
                vector<float> loc = {mon_restaurant_config[table_num]["dropoff"]["x"],
                                             mon_restaurant_config[table_num]["dropoff"]["y"]};
                vector<float> robot_color = {robot_configs[robot_num]["r"],robot_configs[robot_num]["g"],
                                                robot_configs[robot_num]["b"],robot_configs[robot_num]["a"]};
                table_goal_publishers[robot].publish(visualization::MakeCylinder(loc, 0.3, 0.1, "/map","final_delivery_goal",
                                                            robot_color[0], robot_color[1], robot_color[2], robot_color[3], 0.05, order_type_data[robot]));
            }
            else{
                ROS_INFO("Robot %d is NOT active: %d", robot,robot_active_status[robot]);
                // ROS_INFO("robot ID: %d",robot);
                // ROS_INFO("Publishing to topic : %s", pub_sub_prefix.c_str());
                // ROS_INFO("Table %d for robot %d", (robot_table_data[robot],robot));
                
                // std::string table_num = "Table_" + std::to_string(robot_table_data[robot]);
                std::string robot_num = "Robot_" + std::to_string(robot);
                vector<float> loc = {-1000,-1000};
                vector<float> robot_color = {robot_configs[robot_num]["r"],robot_configs[robot_num]["g"],
                                                robot_configs[robot_num]["b"],robot_configs[robot_num]["a"]};
                table_goal_publishers[robot].publish(visualization::MakeCylinder(loc, 0.3, 0.1, "/map","final_delivery_goal",
                                                            robot_color[0], robot_color[1], robot_color[2], robot_color[3], 0.05, true));
            }
        }
    }

};