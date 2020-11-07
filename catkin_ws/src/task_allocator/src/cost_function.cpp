#include "cost_function.h"

// #include <control_stack/RobotDatabase.h>
// #include <control_stack/RobotGoal.h>
// #include <control_stack/RobotPosition.h>

#include <cmath>
#include <iostream>
#include <string>
#include <vector>

// Add msg as input
void CostCalculation::getRobots() {
  robots = {0, 1, 2};
  num_robots = robots.size();
  // robot_loc = {{0.38, 0}, {4, 0}, {8.62, 0}};
  robot_loc = {{0.98, 8.96, 0},{1.98, 8.96, 0},{2.98, 8.96, 0}};
  // Add code to read data from msg
}

// Add msg as input
void CostCalculation::getTasks() {
  num_tasks = 3;
  //   start_task_loc = {{4.5, 10.5}, {4.5, 10.5}, {4.5, 10.5}};
  // start_task_loc = {{3, 10}, {4.5, 10}, {6.0, 10}};
  start_task_loc = {{33.01, 9.46}, {33.01, 9.46}, {33.01, 9.46}};
  //   end_task_loc = {{0.76, 8.24}, {3.74, 8.24}, {6.72, 8.24}};
  end_task_loc = {{19.41, 4.14}, {22.37, 4.59}, {25.84, 4.59}};
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

// // Take in msg as Input
// void CostCalculation::getRobots(const control_stack::RobotDatabase& msg) {
//   // // Initialize Robot Index List
//   // std::vector<int> robot_list;

//   // Loop Through Each Robot's Data (robot_data), Data = "robot" variable
//   for (control_stack::RobotPosition& robot : msg.robot_data) {
//     // Check Robot's Status
//     if (robot.robot_active == 1) {
//       // Get Robot Indexes
//       robots.push_back(robot.robot_index);
//       // Push Each Robot's Position
//       robot_loc.push_back(std::vector<int>());
//       // Get Each Robot's (X,Y) Position
//       robot_loc[i].push_back(
//           robot.robot_position.linear.x);  // Push Robot x-position
//       robot_loc[i].push_back(
//           pos.robot_position.linear.y);  // Push Robot y-position
//     }
//   }
//   // Get Numer of Robots
//   num_robots = robots.size();
// }

// // Take in msg as Input
// void CostCalculation::getTasks(const control_stack::KtichenState& msg) {
//   std::vector<int> tasks;
//   // Loop Through Each Task
//   for (int task : msg.tasks) {
//     // Get Tasks Coordiantes
//     tasks.push_back(task);
//     // Read From Config File Assign Start/End Loc
//   }
//   // Determine Total Number of Tasks
//   num_tasks = msg.tasks.size();
// }

// // // Take in msg as Input
// // void CostCalculation::getTasks(const control_stack::KtichenState& msg) {
// //   num_tasks = msg.tasks.size();
// //   // Loop Through Each Task
// //   for (int task: msg.tasks) {
// //     // Get Tasks Coordiantes
// //     std::vector<float> task = std::next(msg.task_data,i);
// //     // Push Each Tasks's Position
// //     start_task_loc.push_back(std::vector<int>());
// //     end_task_loc.push_back(std::vector<int>());
// //     // Get Each Robot's (X,Y) Position
// //     start_task_loc[i].push_back(task[0]);  // Push Task x-position
// //     end_task_loc[i].push_back(task[1]);  // Push Task y-position
// //   }

// // }

// void CostCalculation::cost_function(
//     int num_tasks,
//     std::vector<std::vector<float>> start_task_loc,
//     int num_robots,
//     std::vector<int> robots,
//     std::vector<std::vector<float>> robot_loc) {
//   // Assign Costs to Available Robots
//   for (int i = 0; i < num_robots; i++) {
//     // Get Robot ID
//     int robot_id = robots[i];
//     // Push Vector into 2D Vector
//     cost_matrix.push_back(std::vector<int>());
//     // Calculate Costs for Each Task
//     for (int j = 0; j < num_tasks; j++) {
//       // Calculate Distance From Current Position to Kitchen
//       int cost = (int)(pow(start_task_loc[j][0] - robot_loc[i][0], 2) +
//                        pow(start_task_loc[j][1] - robot_loc[i][1], 2) +
//                        pow(end_task_loc[j][0] - start_task_loc[j][0], 2) +
//                        pow(end_task_loc[j][1] - start_task_loc[j][1], 2));
//       cost_matrix[i].push_back(cost);  // Push Sub-Vectors
//     }
//   }
// }
