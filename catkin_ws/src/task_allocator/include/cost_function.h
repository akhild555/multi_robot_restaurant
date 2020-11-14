#pragma once

#include <cmath>
#include <iostream>
#include <string>
#include <vector>
#include "control_stack/KitchenOrders.h"
#include "control_stack/RobotDatabase.h"
#include "control_stack/RobotPosition.h"
#include <control_stack/RobotGoal.h>
#include <control_stack/RobotPosition.h>

class CostCalculation {

public:
  // const std::vector<int>& tasks  = {};
  std::vector<control_stack::KitchenOrders> all_orders;
  std::vector<int> robots;
  int num_robots = 0;
  std::vector<std::vector<float>> robot_loc;
  // Kitchen State
  int num_tasks = 0;
  std::vector<std::vector<float>> start_task_loc;  // Kitchen Location
  std::vector<std::vector<float>> end_task_loc;    // Table Location
  // Cost Function Output
  std::vector<std::vector<int>> cost_matrix;
  // Functions
  CostCalculation(){};
  void all_tasks(const control_stack::KitchenOrders& msg);
  void getRobots(const control_stack::RobotDatabase& robot_msg);
  // void getTasks(const control_stack::KtichenState& kitchen_msg);
  // void all_tasks(const control_stack::KitchenOrders& msg);
  // void getRobots();
  void getTasks();
  // std::vector<control_stack::KitchenOrders> all_orders
  void cost_function();
  // int num_tasks,
  //                    std::vector<std::vector<float>> start_task_loc,
  //                    int num_robots,
  //                    std::vector<int> robots,
  //                    std::vector<std::vector<float>> robot_loc);


};