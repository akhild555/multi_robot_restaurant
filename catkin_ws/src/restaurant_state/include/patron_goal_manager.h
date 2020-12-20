#pragma once

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <control_stack/PatronGoal.h>
#include "control_stack/KitchenOrders.h"
#include "control_stack/PatronDatabase.h"
#include "control_stack/OrderInfo.h"
#include "control_stack/OrderExpire.h"
#include "json.h"
#include <fstream>
#include <random>
#include <numeric>
#include <string>

struct Patron_Status {
  int patron_id = 0;
  int table_number = 0;
  ros::Time arrival_time = ros::Time::now() + ros::Duration(24*60*60);
  float duration = 0.0;
  bool task_assigned = false;
  bool patron_active = false;
};

class PatronManager {
    
    ros::Subscriber patron_status_subscriber;
    ros::Publisher patron_assignment;
    ros::Subscriber order_info_subscriber;
    ros::Subscriber order_expire_subscriber;

    nlohmann::json mon_restaurant_config;
    control_stack::PatronGoal patron_assgn;
    geometry_msgs:: Twist t;

    std::vector<control_stack::OrderInfo> all_orders;
    std::vector<Patron_Status> patron_statuses;
    control_stack::OrderInfo oldest_order;
    int number_of_patrons = 0;
    int patron_start_ind = 0;
    int max_patron_tasks = 2;
    bool random_generation = false;
    int seed = -9;

    void OrderInfoCallback(const control_stack::OrderInfo& msg)
    {
        all_orders.push_back(msg);
        assignPatronTable();
    }

    void OrderExpireCallback(const control_stack::OrderExpire& msg){
        
        if (all_orders.size() > 0){
            removeExpiredOrder(msg);
        }
        
        // tell patron to leave restaurant
        for (int i = 0; i < patron_statuses.size(); i++) {
            if (msg.table_number == patron_statuses[i].table_number){
                patron_statuses[i].patron_active = false;
                patronLeave(i);
            }
        }
    }

    void patronGoalCallback(const control_stack::PatronDatabase& msg){
        int num_patrons_doing_tasks = 0;
        for (int i = 0; i < patron_statuses.size(); i++) {
            if (patron_statuses[i].task_assigned == true){
                num_patrons_doing_tasks++;
            }
        }

        if (num_patrons_doing_tasks <= max_patron_tasks){
            for (int i = 0; i < patron_statuses.size(); i++) {
                int time_passed = ros::Time::now().sec - patron_statuses[i].arrival_time.sec;
                if (time_passed > patron_statuses[i].duration / 2 && !patron_statuses[i].task_assigned){
                    // if (num_patrons_doing_tasks <= max_patron_tasks){ // only two patrons doing tasks at a time
                    // }
                    patronDoTask(i);
                    patron_statuses[i].task_assigned = true;
                }
            }
        }
    }

    void patronDoTask(int patron_id) {
        float x = 0.0;
        float y = 0.0;
        int goal = 0;
        patron_assgn.stamp = ros::Time::now();
        patron_assgn.patron_index = patron_id + patron_start_ind;

        // send patron to bathroom or bar
        std::random_device rd;
        if (random_generation){
            std::mt19937 gen(rd());
            std::uniform_int_distribution<> distr(0, 2);
            goal = distr(gen);
        }
        else{
            std::mt19937 gen(seed);
            std::uniform_int_distribution<> distr(0, 2);
            goal = distr(gen);
        }
        ROS_INFO("patron goal generated: %d", goal);

        if (goal == 0){
            x = mon_restaurant_config["Bathroom"]["Female"]["x"];
            y = mon_restaurant_config["Bathroom"]["Female"]["y"];
        }
        else if (goal == 1){
            x = mon_restaurant_config["Bathroom"]["Male"]["x"];
            y = mon_restaurant_config["Bathroom"]["Male"]["y"];
        }
        else if (goal == 2){
            x = mon_restaurant_config["Drinks_bar"]["patron"]["x"];
            y = mon_restaurant_config["Drinks_bar"]["patron"]["y"];
        }

        t.linear.x = x;
        t.linear.y = y;
        patron_assgn.patron_goal.push_back(t);

        // send patron back to their table
        std::string table_number = "Table_" + std::to_string(patron_statuses[patron_id].table_number);
        x = mon_restaurant_config[table_number]["patron"]["x"];
        y = mon_restaurant_config[table_number]["patron"]["y"];
        t.linear.x = x;
        t.linear.y = y;
        patron_assgn.patron_goal.push_back(t);

        patron_assignment.publish(patron_assgn);
        patron_assgn.patron_goal.clear();
    } 

    void removeExpiredOrder(const control_stack::OrderExpire& msg){
        ROS_INFO("all_orders size before: %zu", all_orders.size());
        
        int count = 0;
        for (auto const& order : all_orders){
            count++;
            if (order.table_number == msg.table_number){
                break;
            }
        }
        all_orders.erase(all_orders.begin() + count);

        ROS_INFO("all_orders size after: %zu", all_orders.size());

    }


    void assignPatronTable() {
        if (all_orders.size() >= 8){
            // check if patron is free and assign order/table to patron
            for (int i = 0; i < patron_statuses.size(); i++) {
                if (!patron_statuses[i].patron_active)
                {
                    oldest_order = all_orders.front();
                    all_orders.erase(all_orders.begin());

                    int patron_id = i;
                    ROS_INFO("patron_id %d", patron_id);
                    int order_table_num = oldest_order.table_number;
                    float order_duration = oldest_order.order_duration;
                    storePatronStatus(patron_id, order_table_num, order_duration);                
                }
            }
        }        
    }

    void storePatronStatus(int patron_id, int table_num, float duration) {

        patron_statuses[patron_id].table_number = table_num;
        patron_statuses[patron_id].arrival_time = ros::Time::now();
        patron_statuses[patron_id].duration = duration;
        patron_statuses[patron_id].task_assigned = false;
        patron_statuses[patron_id].patron_active = true;

        patronGoToTable(patron_id);
    }

    void patronGoToTable(int patron_id) {
        patron_assgn.stamp = ros::Time::now();
        // ROS_INFO("Patron ID being published: %d", patron_id + 4);
        patron_assgn.patron_index = patron_id + patron_start_ind;
        patron_assgn.table_number = patron_statuses[patron_id].table_number;

        std::string table_number = "Table_" + std::to_string(patron_statuses[patron_id].table_number);
        float x = mon_restaurant_config[table_number]["patron"]["x"];
        float y = mon_restaurant_config[table_number]["patron"]["y"];
        t.linear.x = x;
        t.linear.y = y;
        patron_assgn.patron_goal.push_back(t);

        patron_assignment.publish(patron_assgn);
        patron_assgn.patron_goal.clear();
    } 

    void patronLeave(int patron_id) {
        patron_assgn.stamp = ros::Time::now();
        ROS_INFO("patronLeave ID being published: %d", patron_id + patron_start_ind);
        patron_assgn.patron_index = patron_id + patron_start_ind;
        patron_assgn.table_number = patron_statuses[patron_id].table_number;

        std::string patron_num = "patron_" + std::to_string(patron_id + patron_start_ind);
        float x = mon_restaurant_config["Outside"][patron_num]["x"];
        float y = mon_restaurant_config["Outside"][patron_num]["y"];
        t.linear.x = x;
        t.linear.y = y;
        patron_assgn.patron_goal.push_back(t);

        patron_assignment.publish(patron_assgn);
        patron_assgn.patron_goal.clear();
    } 


public:

    /**
    * Constructor
    */ 
    PatronManager(ros::NodeHandle& nh, int num_patrons, int patron_start_index) {
        number_of_patrons = num_patrons;
        patron_start_ind = patron_start_index;

        for (int i = 0; i < num_patrons; i++)
        {
            Patron_Status patron;
            patron.patron_id = i;
            patron.table_number = 0;
            patron.duration = 0.0;
            patron.patron_active = false;
            patron_statuses.push_back(patron);
        }

        std::ifstream file_input("src/control_stack/config/monarch_config.json");
        file_input >> mon_restaurant_config;

        patron_assignment = nh.advertise<control_stack::PatronGoal>("/patron_goal", 1000);
        
        patron_status_subscriber = nh.subscribe("/patron_database",
                1, &PatronManager::patronGoalCallback, this);

        order_info_subscriber = nh.subscribe("/order_info",
                100, &PatronManager::OrderInfoCallback, this);

        order_expire_subscriber = nh.subscribe("/order_expire",
                100, &PatronManager::OrderExpireCallback, this);
        
        ROS_INFO("Patron manager ready");
    }  
};