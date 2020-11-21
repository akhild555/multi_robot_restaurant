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
  std::vector<int> assigned_tasks; // tables already assigned to robots
  std::vector<int> assigned_orders; // orders already assigned to robots
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
  // Get New Tasks
  void all_tasks(const control_stack::KitchenOrders& msg);
  // Get Current Robot Positions
  void getRobots(const control_stack::RobotDatabase& robot_msg);
  // Get # of Tasks from Queue
  void getTasks();
  // Calculate Task Costs
  void cost_function();


};