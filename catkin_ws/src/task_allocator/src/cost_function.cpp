
#include <cmath>
#include <iostream>
#include <string>
#include <vector>
#include "cost_function.h"



// Add msg as input
void CostCalculation::getRobots() {
  robots = {0, 1, 2};
  num_robots = robots.size();
  robot_loc = {{0.38, 0},{4, 0},{8.62, 0}};
  // Add code to read data from msg
}

// Add msg as input
void CostCalculation::getTasks() {
  num_tasks = 3;
//   start_task_loc = {{4.5, 10.5}, {4.5, 10.5}, {4.5, 10.5}};
  start_task_loc = {{3, 10}, {4.5, 10}, {6.0, 10}};
//   end_task_loc = {{0.76, 8.24}, {3.74, 8.24}, {6.72, 8.24}};
  end_task_loc = {{0.76, 3.01}, {4.5, 5.99}, {7.48, 3.01}};
  // Add code to read data from msg
}

void CostCalculation::cost_function(
    int num_tasks,
    std::vector<std::vector<float>> start_task_loc,
    int num_robots,
    std::vector<int> robots,
    std::vector<std::vector<float>> robot_loc) {
  
  // Assign Costs to Available Robots
  for (int i = 0; i < num_robots; i++) {
    // Get Robot ID
    int robot_id = robots[i];    
    // Push Vector into 2D Vector
    cost_matrix.push_back(std::vector<int>());
    // Calculate Costs for Each Task
    
    for (int j = 0; j < num_tasks; j++) {
      int cost = (int)(pow(start_task_loc[j][0] - robot_loc[i][0], 2) +
             pow(start_task_loc[j][1] - robot_loc[i][1], 2));
      cost_matrix[i].push_back(cost);  // Push Sub-Vectors
      
    }
  }
}


