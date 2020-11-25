#pragma once

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "control_stack/KitchenOrders.h"
#include <random>

class KitchenManager {

  ros::Publisher order_publisher;
  control_stack::KitchenOrders order;

public:

  void randOrderGenerator(int number_of_tables, int counter)
  {
    // generate random number to determine whether to generate new order
    std::random_device pub_freq; // obtain a random number from hardware
    std::mt19937 gen(pub_freq()); // seed the generator
    std::uniform_int_distribution<> distr(1, 100); // define the range
    if (distr(gen) % 10 == 0)
    {
        createOrder(number_of_tables, counter);
    }
  }
  void createOrder(int number_of_tables, int counter)
  {
    // generate random table
    std::random_device rd; // obtain a random number from hardware
    std::mt19937 gen(rd()); // seed the generator
    std::uniform_int_distribution<> distr(1, number_of_tables); // define the range

    order.stamp = ros::Time::now(); // Add Time Stamp
    order.order_number = counter;
    order.table_number = distr(gen);
    order.drinks = true; //(distr(gen) % 2 == 0) ? true : false;
    // order.food_ordered = "pasta";
    order.food_ordered = true;
    publishKitchenOrders();
  }
  
  void publishKitchenOrders() {
    order_publisher.publish(order);
    ROS_INFO("published order");
  }



  /**
   * Constructor
   */ 
  KitchenManager(ros::NodeHandle& nh) {

    order_publisher = nh.advertise<control_stack::KitchenOrders>("/kitchen_state", 100);
    ROS_INFO("Kitchen manager ready");
  }
};