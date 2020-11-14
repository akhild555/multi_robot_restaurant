#include "cost_function.h"
#include "json.h"
// #include <control_stack/RobotDatabase.h>
// #include <control_stack/RobotGoal.h>
// #include <control_stack/RobotPosition.h>

#include <cmath>
#include <iostream>
#include <string>
#include <vector>
#include <fstream>


using json = nlohmann::json;
void CostCalculation::all_tasks(const control_stack::KitchenOrders& msg){

  // std::cout <<"table num - all tasks " << msg.table_number << std::endl;
  all_orders.push_back(msg);
  // std::cout<<"ALL TASKS fun: all_orders.size()"<<all_orders.size()<<std::endl;
  for(int i=0; i<all_orders.size(); i++)
  {
    std::cout<<all_orders[i].order_number<<" ";
  }
  std::cout<<std::endl;
  CostCalculation::getTasks();
}
void CostCalculation::getTasks(){
  // std::vector<control_stack::KitchenOrders> all_orders) {
  // std::cout<<"IN GETTASKS beg: "<<all_orders.size()<<std::endl;

  start_task_loc.clear();
  end_task_loc.clear();
  num_tasks = num_robots;
  std::ifstream file_input("src/control_stack/config/monarch_config.json");
  json mon_restaurant_config;
  file_input >> mon_restaurant_config;

  if(num_robots > all_orders.size())
  {
    num_tasks = all_orders.size();
  }
  
  for(int i=0; i<num_tasks; i++)
  {
    start_task_loc.push_back(std::vector<float>());
    float start_x1 =  mon_restaurant_config["Kitchen"]["x"];
    float start_y1 =  mon_restaurant_config["Kitchen"]["y"];
    start_task_loc[i].push_back(start_x1);
    start_task_loc[i].push_back(start_y1);
    std::string table_num = "Table_" + std::to_string(all_orders[i].table_number); 
    float end_x1 =  mon_restaurant_config[table_num]["dropoff"]["x"];
    float end_y1 =  mon_restaurant_config[table_num]["dropoff"]["y"];
    end_task_loc.push_back(std::vector<float>());

    end_task_loc[i].push_back(end_x1);
    end_task_loc[i].push_back(end_y1);

  }
  // all_orders.erase(0, num_tasks);
  all_orders.erase(all_orders.begin(), all_orders.begin() +num_tasks);
  std::cout<<"AFTER ERASE: all_orders.size()"<<all_orders.size()<<std::endl;
  // num_tasks = all_orders.size();
}
void CostCalculation::getRobots(const control_stack::RobotDatabase& msg) {
  // // Initialize Robot Index List
  // std::vector<int> robot_list;
  int i = 0;
  robot_loc.clear();
  // std::cout<<"in get robots"<<std::endl;
  robots.clear();
  // Loop Through Each Robot's Data (robot_data), Data = "robot" variable
  for (const control_stack::RobotPosition& robot : msg.robot_data) {
    // Check Robot's Status
    // bool status = robot.robot_active;
    // std::cout<<"Robot "<<robot.robot_index<<" status "<< std::boolalpha << robot.robot_active<<std::endl;

    if (robot.robot_active == false) {
      // Get Robot Indexes
      // std::cout<<"in false "<<std::endl;
      robots.push_back(robot.robot_index);
      // Push Each Robot's Position
      // std::vector<int> robot_loc_temp;
      robot_loc.push_back(std::vector<float>());
      // Get Each Robot's (X,Y) Position
      // std::cout<<"x loc "<< robot.robot_position.linear.x<<std::endl;
      // std::cout<<"y loc "<< robot.robot_position.linear.y<<std::endl;
      robot_loc[i].push_back(
          robot.robot_position.linear.x);  // Push Robot x-position
      robot_loc[i].push_back(
          robot.robot_position.linear.y);  // Push Robot y-position
      i++;
    }
  }
  num_robots = robots.size();
  // std::cout<<"num robots"<<num_robots<<std::endl;
}



void CostCalculation::cost_function(){
    // int num_tasks,
    // std::vector<std::vector<float>> start_task_loc,
    // int num_robots,
    // std::vector<int> robots,
    // std::vector<std::vector<float>> robot_loc) {
  // Assign Costs to Available Robots
  std::cout<<"num_tasks in cost fnc"<<num_tasks<<std::endl;
  std::cout<<"num_robots in cost func"<<num_robots<<std::endl;
  cost_matrix.clear();
  std::cout<<"robot_loc.size() = "<<robot_loc.size()<<std::endl;
  std::cout<<"start_task_loc.size() = "<<start_task_loc.size()<<std::endl;
  for (int i = 0; i < num_robots; i++) {
    // Get Robot ID
    // int robot_id = robots[i];
    // Push Vector into 2D Vector
    cost_matrix.push_back(std::vector<int>());
    // Calculate Costs for Each Task
    for (int j = 0; j < num_tasks; j++) {
      
      int cost = (int)(pow(start_task_loc[j][0] - robot_loc[i][0], 2) +
                       pow(start_task_loc[j][1] - robot_loc[i][1], 2));
      cost_matrix[i].push_back(cost);  // Push Sub-Vectors
    }
  }
  // num_tasks = all_orders.size();
  std::cout<<"NUM tasks after update in cost func "<<num_tasks<<std::endl;
}