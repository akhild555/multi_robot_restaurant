
#include <cmath>
#include <iostream>
#include <string>
#include <vector>
#include "cost_function.h"

// CostCalculation::CostCalculation(
//     getRobots();
//     getTasks();
//     cost_function();
// );

//  public:
 
//   std::vector<int> robots;
//   int num_robots;
//   std::vector<std::vector<float>> robot_loc;
//   // Kitchen State
//   int num_tasks;
//   std::vector<std::vector<float>> start_task_loc;  // Kitchen Location
//   std::vector<std::vector<float>> end_task_loc;    // Table Location
//   // Cost Function Output
//   std::vector<std::vector<float>>& cost_matrix;
//   // Functions
//   void getRobots();
//   void getTasks();
//   void cost_function(int num_tasks,
//                      std::vector<std::vector<float>> start_task_loc,
//                      int num_robots,
//                      std::vector<int> robots,
//                      std::vector<std::vector<float>> robot_loc);
// };

// Add msg as input
void CostCalculation::getRobots() {
  robots = {1, 2, 3};
  num_robots = robots.size();
  robot_loc = {{0.76, 0.76}, {3.74, 0.76}, {6.72, 0.76}};
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


// int main(int argc, char** argv) {
//   ros::init(argc, argv, "cost_func_node");
//   ros::NodeHandle n;

//   // Declare CostCalculation Class
//   CostCalculation cost_func;

//   // Subscribe to Kitchen State Topic
//   ros::Subscriber kitchen_state_sub =
//       n.subscribe("/kitchen state", 1, &CostCalculation::getTasks, &cost_func);
//   // Subscribe to Robot State Topic
//   ros::Subscriber robot_state_sub =
//       n.subscribe("/robot state", 1, &CostCalculation::getRobots, &cost_func);
//   // // Publish Cost Matrix 
//   // ros::Publisher cost_matrix_pub =
//   //     n.advertise<std_msgs::Float32MultiArray>("/cost_matrix", 1);

//   // RateLoop rate(40.0);
//   while (ros::ok()) {
//     // std_msgs::Float32MultiArray CostMatrix;
//     // CostMatrix.data.clear();
//     // CostMatrix.data = cost_matrix;
//     ros::spinOnce();
//     // cost_matrix_pub.publish(CostMatrix);
//     rate.Sleep();
//   }
//   return 0;
// }