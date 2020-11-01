#pragma once

#include <cmath>
#include <iostream>
#include <string>
#include <vector>

class CostCalculation {


public:
  std::vector<int> robots;
  int num_robots;
  std::vector<std::vector<float>> robot_loc;
  // Kitchen State
  int num_tasks;
  std::vector<std::vector<float>> start_task_loc;  // Kitchen Location
  std::vector<std::vector<float>> end_task_loc;    // Table Location
  // Cost Function Output
  std::vector<std::vector<int>> cost_matrix;
  // Functions
  CostCalculation(){};
  void getRobots();
  void getTasks();
  void cost_function(int num_tasks,
                     std::vector<std::vector<float>> start_task_loc,
                     int num_robots,
                     std::vector<int> robots,
                     std::vector<std::vector<float>> robot_loc);


};
