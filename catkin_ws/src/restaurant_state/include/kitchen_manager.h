#pragma once

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "control_stack/KitchenOrders.h"
#include "control_stack/RobotDatabase.h"
#include "control_stack/OrderInfo.h"
#include "control_stack/OrderExpire.h"
#include <random>
#include <numeric>
#include <string>

struct Table_Status {
  int table_number = 0;
  int current_order_num = 0;
  bool drinks_present = false;
  bool food_present = false;
  bool occupied = false;
  bool cleaned = true;
  double time_vacant = 0;
  double weight = 0;
};

struct Order_Status {
  int order_number = 0;
  int table_number = 0;
  ros::Time dropoff_time = ros::Time::now() + ros::Duration(24*60*60);
  double customer_max_time = 0;
  double clean_max_time = 0;
  std::string order_type = "";
};

class KitchenManager {

  ros::Subscriber robot_status_subscriber;
  ros::Publisher order_publisher;
  ros::Publisher order_info_publisher;
  ros::Publisher order_expire_publisher;

  control_stack::KitchenOrders order;
  control_stack::OrderInfo order_info;
  control_stack::OrderExpire order_expire;
  Table_Status new_table;
  std::vector<Table_Status> table_statuses;
  std::vector<Order_Status> order_statuses;
  std::vector<double> vacant_times;
  std::vector<double> table_weights;

  int counter = 0;
  bool food_order = true;
  bool drinks_order = true;



  void RobotStatusCallback(const control_stack::RobotDatabase& msg) {
      bool one_order_only = false;

      // update drop off times
      for (int i = 0; i < order_statuses.size(); i++) {
        for (const control_stack::RobotPosition& robot_msg : msg.robot_data) {
            if (robot_msg.order_number == order_statuses[i].order_number && !robot_msg.robot_active) {
              ROS_DEBUG("Order %d dropped off at Table %d", order_statuses[i].order_number, order_statuses[i].table_number);
              order_statuses[i].dropoff_time = ros::Time::now();
            }
        }
      }

      
      std::vector<int> finished_order_idx;
      for (int i = 0; i < order_statuses.size(); i++) {
        // check if customer has left the table
        int time_passed = ros::Time::now().sec - order_statuses[i].dropoff_time.sec;
        if (time_passed >= order_statuses[i].customer_max_time && order_statuses[i].order_type == "meal") {
          if (!one_order_only)
          {
            // clean up table
            ROS_DEBUG("Table %d is vacant", order_statuses[i].table_number);
            orderExpired(i);
            createCleanOrder(i, order_statuses[i].table_number);
            int tb = order_statuses[i].table_number - 1;
            table_statuses[tb].occupied = false;
            table_statuses[tb].time_vacant = time_passed;
            finished_order_idx.push_back(i);
            one_order_only = true;
          }
        }
        if (time_passed >= order_statuses[i].clean_max_time && order_statuses[i].order_type == "cleanup") {
          // free up table for new orders
          ROS_DEBUG("Table %d is clean", order_statuses[i].table_number);
          int tb = order_statuses[i].table_number - 1;
          table_statuses[tb].occupied = false;
          table_statuses[tb].cleaned = true;
          table_statuses[tb].time_vacant = time_passed;
          finished_order_idx.push_back(i);
        }
      }

      // discard completed orders
      for (int i = 0; i < finished_order_idx.size(); i++)      {
        order_statuses[finished_order_idx[i]] = order_statuses.back();
        order_statuses.pop_back();
      }
      
      updateTableWeights();

      if (!one_order_only) {
        randOrderGenerator();
      }
  }

  void orderExpired(int order_ind){
    order_expire.stamp = ros::Time::now();
    order_expire.order_number = order_statuses[order_ind].order_number;
    order_expire.table_number = order_statuses[order_ind].table_number;
    publishOrderExpired();    
  }

  void publishOrderExpired(){
    order_expire_publisher.publish(order_expire);
  }

  void createCleanOrder(int order_idx, int table_num){
    order.stamp = ros::Time::now();
    order.order_number = counter;
    order.table_number = table_num;
    order.drinks_ordered = false;
    order.food_ordered = false;

    if (table_statuses[table_num - 1].food_present){
      order.food_cleanup = true;
    }
    else if (table_statuses[table_num - 1].drinks_present){
      order.drink_cleanup = true;
    }
    else if ((table_statuses[table_num - 1].food_present) && (table_statuses[table_num - 1].drinks_present)) {
      order.food_cleanup = true;
      order.drink_cleanup = true;
    }

    // storeTableStatus(table_num);
    storeOrderStatus();
    publishKitchenOrders();
    ROS_DEBUG("Clean order number %d sent to table %d", order.order_number, order.table_number);
  }


  void updateTableWeights() {
    // update vacant times for each table
    for (int i= 0; i < table_statuses.size(); i++) {
      if (table_statuses[i].occupied || !table_statuses[i].cleaned) {
        vacant_times[i] = 0.0;
      }
      else {
        vacant_times[i] = table_statuses[i].time_vacant;
      }
    } 
    
    // update table weights depending on table vacant times
    int sum_of_times = std::accumulate(vacant_times.begin(), vacant_times.end(), 0); 

    for (int i= 0; i < table_statuses.size(); i++) {
      if (table_statuses[i].occupied || !table_statuses[i].cleaned) {
        table_weights[i] = 0.0;
      }
      else {
        table_weights[i] = table_statuses[i].time_vacant / sum_of_times;
      }
    } 
  }

  void randOrderGenerator()  {

    // count how many tables are occupied (weight is zero)
    int zero_weight_counter = 0;
    for (int i= 0; i < table_weights.size(); i++)
    {
      if (table_weights[i] == 0.0)
      {
        zero_weight_counter++;
      }
    } 

    // assign table number based on table weights
    if (zero_weight_counter != table_weights.size()){
      double sum_of_weights = 1.0;
      int gen_table_num = 0;

      std::random_device pub_freq; // obtain a random number from hardware
      std::mt19937 gen(pub_freq()); // seed the generator
      std::uniform_real_distribution<> distr(0.0, sum_of_weights); // define range
      double rand_num = distr(gen);

      for (int i= 0; i < table_weights.size(); i++) {
        // std::cout << "Random number = " << rand_num << std::endl;
        // std::cout << "Table " << i + 1 << " Weight = " << table_weights[i] << std::endl;
        if (rand_num < table_weights[i]) {
          gen_table_num = i + 1;
          break;
        }
        rand_num -= table_weights[i];
      }

      // std::cout << "Generated Table number = " << gen_table_num << std::endl;
      orderDrinks();
      orderFood();
      createMealOrder(gen_table_num);
    }
  }

  void orderFood() {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> distr(0, 1);
    food_order = distr(gen);
    if (food_order == false && drinks_order == false)    {
      food_order = true;
    }
  }

  void orderDrinks() {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> distr(0, 1);
    drinks_order = distr(gen);
  }

  void storeOrderStatus()  {
    // customer meal time generator
    std::default_random_engine generator_meal(time(0));
    std::normal_distribution<double> distribution_meal(300.0,45.0);
    
    // table cleanup time generator
    std::default_random_engine generator_clean(time(0));
    std::normal_distribution<double> distribution_clean(20.0,4.0);

    Order_Status current_order;
    current_order.order_number = counter;
    current_order.table_number = order.table_number;

    if (order.food_cleanup || order.drink_cleanup){
      current_order.clean_max_time =  distribution_clean(generator_clean);
      current_order.order_type = "cleanup";
    }   
    else {
      current_order.customer_max_time =  distribution_meal(generator_meal); 
      current_order.order_type = "meal";
    }
    order_statuses.push_back(current_order);
  }

  void storeTableStatus(int table_num)  {
    table_statuses[table_num - 1].occupied = true;
    table_statuses[table_num - 1].cleaned = false;
    table_statuses[table_num - 1].drinks_present = order.drinks_ordered;
    table_statuses[table_num - 1].food_present = order.food_ordered;
  }

  void createMealOrder(int table_num)  {
    order.stamp = ros::Time::now();
    order.order_number = counter;
    order.table_number = table_num;
    order.drinks_ordered = drinks_order;
    order.food_ordered = food_order;
    order.drink_cleanup = false;
    order.food_cleanup = false;

    storeTableStatus(table_num);
    storeOrderStatus();
    publishKitchenOrders();

    order_info.stamp = ros::Time::now();
    order_info.order_number = counter;
    order_info.table_number = table_num;
    // ROS_INFO("customer max time = %f", order_statuses[-1].customer_max_time);
    order_info.order_duration = order_statuses.back().customer_max_time;
    publishOrderInfo();
  }
  
  
  void publishKitchenOrders()   {
    order_publisher.publish(order);
    ROS_INFO("Published order %i", order.order_number);
  }

  void publishOrderInfo()   {
    order_info_publisher.publish(order_info);
    ROS_INFO("Published order_info %i", order.order_number);
  }

  

public:

  /**
   * Constructor
   */ 
  KitchenManager(ros::NodeHandle& nh, int number_of_tables) {

    for (int i = 0; i < number_of_tables; i++)    {
      new_table.table_number = i;
      new_table.time_vacant = 1.0;
      new_table.weight = 1.0 / (float)number_of_tables;
      table_weights.push_back(new_table.weight);
      table_statuses.push_back(new_table);
      vacant_times.push_back(0.0);
    }

    order_publisher = nh.advertise<control_stack::KitchenOrders>("/kitchen_state", 100);
    ROS_INFO("Kitchen manager ready");


    order_info_publisher = nh.advertise<control_stack::OrderInfo>("/order_info", 100);
    order_expire_publisher = nh.advertise<control_stack::OrderExpire>("/order_expire", 100);

    robot_status_subscriber = nh.subscribe("/robot_database",
          1, &KitchenManager::RobotStatusCallback, this);
  }

  void updateCounter(int count){
    counter = count;
  }

};