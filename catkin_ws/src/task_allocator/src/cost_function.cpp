#include "cost_function.h"

#include "json.h"
// #include <control_stack/RobotDatabase.h>
// #include <control_stack/RobotGoal.h>
// #include <control_stack/RobotPosition.h>

#include <cmath>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include <algorithm> 

#include "ros/ros.h"

using json = nlohmann::json;

// Get Current Robot Positions
void CostCalculation::getRobots(const control_stack::RobotDatabase& msg) {
  // Initialize Counter
  int i = 0;
  // Remove Old Robot Locations Before Assigning New Ones
  robot_loc.clear();
  // Remove Old Number of Available Robots
  robots.clear();
  // std::cout<<"in get robots"<<std::endl;
  // Loop Through Each Robot's Data (robot_data), Data = "robot_msg" variable

  for (const control_stack::RobotPosition& robot_msg : msg.robot_data) {
    // Check Robot's Status
    // bool status = robot.robot_active;
    // std::cout<<"Robot "<<robot.robot_index<<" status "<< std::boolalpha <<
    // robot.robot_active<<std::endl;

    // std::cout << "Size of assigned orders: " <<  assigned_orders.size() <<
    // std::endl;
    // bool already_assigned =
    //     std::find(pending_orders.begin(),
    //               pending_orders.end(),
    //               robot_msg.order_number) != pending_orders.end();

    // if ((robot_msg.order_number == 0) || already_assigned) {
    //   if (robot_msg.robot_active == false) {
      
      // robot_msg.order_number != 0 &&

      if ( (task_alloc_memory[robot_msg.robot_index] == robot_msg.order_number) && (!robot_msg.robot_active))
      {
        // std::stringstream ss;
        // ss<<"Robot "<<robot_msg.robot_index<<" available, Memory order: "<<task_alloc_memory[robot_msg.robot_index];
        // ss<<" feedback order: "<<robot_msg.order_number<<" status: "<<(bool)robot_msg.robot_active;
        // ROS_ERROR("%s",ss.str().c_str());
        
        // Get Robot Indexes
        // ROS_ERROR("ROBOT ACTIVE = FALSE!!!!!!!!!!!!! %d",
        // robot_msg.robot_index);
        robots.push_back(robot_msg.robot_index);
        // std::cout<<"getRobots No. Robots: "<<robots.size()<<std::endl;
        // Push Each Robot's Position
        robot_loc.push_back(std::vector<float>());
        // Get Each Robot's (X,Y) Position
        // std::cout<<"x loc "<< robot.robot_position.linear.x<<std::endl;
        // std::cout<<"y loc "<< robot.robot_position.linear.y<<std::endl;
        robot_loc[i].push_back(robot_msg.robot_position.linear.x);  // Push Robot x-position
        robot_loc[i].push_back(robot_msg.robot_position.linear.y);  // Push Robot y-position
        // Increase Iterator
        i++;
        pending_orders.erase(std::remove(pending_orders.begin(),
                                         pending_orders.end(),
                                         robot_msg.order_number),
                             pending_orders.end());
        // std::cout << "Size of assigned orders after erase: " <<
        // assigned_orders.size() << std::endl;
      }
    // }
  }
  // Determine Number of Available Robots
  num_robots = robots.size();
}

// Gets New Orders and Gets Orders To Be Assigned to Available Robots
void CostCalculation::all_tasks(const control_stack::KitchenOrders& msg) {
  // std::cout <<"table num - all tasks " << msg.table_number << std::endl;
  // Add New Orders to Queue
  all_orders.push_back(msg);

  // std::cout<<"ALL TASKS fun:
  // all_orders.size()"<<all_orders.size()<<std::endl; std::cout<<"Order No. ";
  // // Print Out Order Numbers in Queue
  // for(int i=0; i<all_orders.size(); i++)
  // {
  //   std::cout<<all_orders[i].order_number<<" ";
  // }
  // std::cout<<std::endl;

  // Call getTasks to Remove # of Tasks for Assignment
  // CostCalculation::getTasks();
}

// Gets Orders To Be Assigned to Available Robots
void CostCalculation::getTasks() {
  // Set Config File to Be Used
  std::ifstream file_input("src/control_stack/config/monarch_config.json");
  json mon_restaurant_config;
  file_input >> mon_restaurant_config;

  // std::vector<control_stack::KitchenOrders> all_orders) {
  // std::cout<<"IN GETTASKS beg: "<<all_orders.size()<<std::endl;

  // Clear Start Task Locations
  start_task_loc.clear();
  // Clear Mid Task Locations
  mid_task_loc.clear();
  // Clear End Task Locations
  end_task_loc.clear();
  // Clear Assigned Tasks
  assigned_tasks.clear();
  assigned_orders.clear();
  // Clear Number Locations Per Task
  task_type.clear();

  order_types.clear();

  // std::cout<<"getTasks: No. Robots"<<num_robots<<std::endl;
  // Set Number of Tasks = Number of Robots
  num_tasks = num_robots;
  // If More Robots Than Tasks, Limit To Number of Tasks Available
  if (num_robots > all_orders.size()) {
    num_tasks = all_orders.size();
  }
  // Get Task Start and End Locations
  for (int i = 0; i < num_tasks; i++) {
    // Drinks and Food Ordered
    if (all_orders[i].food_ordered == true && all_orders[i].drinks_ordered == true) {
      // Pushback Start Task Location Vector After Getting Individual Values
      start_task_loc.push_back(std::vector<float>());
      // Get Start Task Location (Default Kitchen)
      float start_x1 = mon_restaurant_config["Kitchen"]["x"];
      float start_y1 = mon_restaurant_config["Kitchen"]["y"];
      // Pushback Start Task x,y
      start_task_loc[i].push_back(start_x1);
      start_task_loc[i].push_back(start_y1);

      // Pushback Mid Task Location Vector After Getting Individual Values
      mid_task_loc.push_back(std::vector<float>());
      // Get Mid Task Location (Default Kitchen)
      float mid_x1 = mon_restaurant_config["Drinks_bar"]["x"];
      float mid_y1 = mon_restaurant_config["Drinks_bar"]["y"];
      // Pushback Mid Task x,y
      mid_task_loc[i].push_back(mid_x1);
      mid_task_loc[i].push_back(mid_y1);

      // Convert Table Number to Read into Config Files
      std::string table_num = "Table_" + std::to_string(all_orders[i].table_number);
      // Get End Task Location
      float end_x1 = mon_restaurant_config[table_num]["dropoff"]["x"];
      float end_y1 = mon_restaurant_config[table_num]["dropoff"]["y"];
      // Pushback End Task Location Vector After Getting Individual Values
      end_task_loc.push_back(std::vector<float>());
      // Pushback End Task x,y
      end_task_loc[i].push_back(end_x1);
      end_task_loc[i].push_back(end_y1);
      assigned_tasks.push_back(all_orders[i].table_number);  // keep track of assigned table
      assigned_orders.push_back(all_orders[i].order_number);  // keep track of assigned order
      order_types.push_back(true); // storing the order type in a variable of the class
      task_type.push_back("Drinks and Food Ordered ");  // keep track of task type
      // std::cout << "Drinks and Food Ordered " << std::endl;

    }
    // Only Food Ordered
    else if (all_orders[i].food_ordered == true && all_orders[i].drinks_ordered == false) {
      // Pushback Start Task Location Vector After Getting Individual Values
      start_task_loc.push_back(std::vector<float>());
      // Get Start Task Location (Default Kitchen)
      float start_x1 = mon_restaurant_config["Kitchen"]["x"];
      float start_y1 = mon_restaurant_config["Kitchen"]["y"];
      // Pushback Start Task x,y
      start_task_loc[i].push_back(start_x1);
      start_task_loc[i].push_back(start_y1);

      // Pushback Mid Task Placeholder Location Vector
      mid_task_loc.push_back(std::vector<float>());
      // Pushback Mid Task x,y
      mid_task_loc[i].push_back(-1);
      mid_task_loc[i].push_back(-1);

      // Convert Table Number to Read into Config Files
      std::string table_num =
          "Table_" + std::to_string(all_orders[i].table_number);
      // Get End Task Location
      float end_x1 = mon_restaurant_config[table_num]["dropoff"]["x"];
      float end_y1 = mon_restaurant_config[table_num]["dropoff"]["y"];
      // Pushback End Task Location Vector After Getting Individual Values
      end_task_loc.push_back(std::vector<float>());
      // Pushback End Task x,y
      end_task_loc[i].push_back(end_x1);
      end_task_loc[i].push_back(end_y1);
      assigned_tasks.push_back(all_orders[i].table_number);  // keep track of assigned table
      assigned_orders.push_back(all_orders[i].order_number);  // keep track of assigned order
      order_types.push_back(true);  // storing the order type in a variable of the class 
      task_type.push_back("Only Food Ordered ");  // keep track of task type
      // std::cout << "Only Food Ordered" << std::endl;

    }
    // Only Drinks Ordered
    else if (all_orders[i].food_ordered == false && all_orders[i].drinks_ordered == true) {
      // Pushback Start Task Location Vector After Getting Individual Values
      start_task_loc.push_back(std::vector<float>());
      // Get Start Task Location (Default Kitchen)
      float start_x1 = mon_restaurant_config["Drinks_bar"]["x"];
      float start_y1 = mon_restaurant_config["Drinks_bar"]["y"];
      // Pushback Start Task x,y
      start_task_loc[i].push_back(start_x1);
      start_task_loc[i].push_back(start_y1);

      // Pushback Mid Task Placeholder Location Vector
      mid_task_loc.push_back(std::vector<float>());
      // Pushback Mid Task x,y
      mid_task_loc[i].push_back(-1);
      mid_task_loc[i].push_back(-1);

      // Convert Table Number to Read into Config Files
      std::string table_num = "Table_" + std::to_string(all_orders[i].table_number);
      // Get End Task Location
      float end_x1 = mon_restaurant_config[table_num]["dropoff"]["x"];
      float end_y1 = mon_restaurant_config[table_num]["dropoff"]["y"];
      // Pushback End Task Location Vector After Getting Individual Values
      end_task_loc.push_back(std::vector<float>());
      // Pushback End Task x,y
      end_task_loc[i].push_back(end_x1);
      end_task_loc[i].push_back(end_y1);
      assigned_tasks.push_back(all_orders[i].table_number);  // keep track of assigned table
      assigned_orders.push_back(all_orders[i].order_number);  // keep track of assigned order
      order_types.push_back(true);  // storing the order type in a variable of the class 
      task_type.push_back("Only Drinks Ordered ");  // keep track of task type
      // std::cout << "Only Drinks Ordered " << std::endl;

    }
    // Food and Drink Cleanup
    else if (all_orders[i].drink_cleanup == true && all_orders[i].food_cleanup == true) {
      // Convert Table Number to Read into Config Files
      std::string table_num = "Table_" + std::to_string(all_orders[i].table_number);
      // Get End Task Location
      float start_x1 = mon_restaurant_config[table_num]["dropoff"]["x"];
      float start_y1 = mon_restaurant_config[table_num]["dropoff"]["y"];
      // Pushback Start Task Location Vector After Getting Individual Values
      start_task_loc.push_back(std::vector<float>());
      // Pushback Start Task x,y
      start_task_loc[i].push_back(start_x1);
      start_task_loc[i].push_back(start_y1);

      // Pushback Mid Task Location Vector After Getting Individual Values
      mid_task_loc.push_back(std::vector<float>());
      // Get Mid Task Location (Default Kitchen)
      float mid_x1 = mon_restaurant_config["Glassware_cleaning"]["x"];
      float mid_y1 = mon_restaurant_config["Glassware_cleaning"]["y"];
      // Pushback Mid Task x,y
      mid_task_loc[i].push_back(mid_x1);
      mid_task_loc[i].push_back(mid_y1);

      // Get End Task Location
      float end_x1 = mon_restaurant_config["Tableware_cleaning"]["x"];
      float end_y1 = mon_restaurant_config["Tableware_cleaning"]["y"];
      // Pushback End Task Location Vector After Getting Individual Values
      end_task_loc.push_back(std::vector<float>());
      // Pushback End Task x,y
      end_task_loc[i].push_back(end_x1);
      end_task_loc[i].push_back(end_y1);
      assigned_tasks.push_back(all_orders[i].table_number);  // keep track of assigned table
      assigned_orders.push_back(all_orders[i].order_number);  // keep track of assigned order
      order_types.push_back(false);  // storing the order type in a variable of the class 
      task_type.push_back("Food and Drink Cleanup ");  // keep track of task type
      // std::cout << "Food and Drink Cleanup " << std::endl;

    }
    // Food Cleanup Only
    else if (all_orders[i].drink_cleanup == false && all_orders[i].food_cleanup == true) {
      // Convert Table Number to Read into Config Files
      std::string table_num = "Table_" + std::to_string(all_orders[i].table_number);
      // Get End Task Location
      float start_x1 = mon_restaurant_config[table_num]["dropoff"]["x"];
      float start_y1 = mon_restaurant_config[table_num]["dropoff"]["y"];
      // Pushback Start Task Location Vector After Getting Individual Values
      start_task_loc.push_back(std::vector<float>());
      // Pushback Start Task x,y
      start_task_loc[i].push_back(start_x1);
      start_task_loc[i].push_back(start_y1);

      // Pushback Mid Task Placeholder Location Vector
      mid_task_loc.push_back(std::vector<float>());
      // Pushback Mid Task x,y
      mid_task_loc[i].push_back(-1);
      mid_task_loc[i].push_back(-1);

      // Get End Task Location
      float end_x1 = mon_restaurant_config["Tableware_cleaning"]["x"];
      float end_y1 = mon_restaurant_config["Tableware_cleaning"]["y"];
      // Pushback End Task Location Vector After Getting Individual Values
      end_task_loc.push_back(std::vector<float>());
      // Pushback End Task x,y
      end_task_loc[i].push_back(end_x1);
      end_task_loc[i].push_back(end_y1);
      assigned_tasks.push_back(all_orders[i].table_number);  // keep track of assigned table
      assigned_orders.push_back(all_orders[i].order_number);  // keep track of assigned order
      order_types.push_back(false);  // storing the order type in a variable of the class
      task_type.push_back("Food Cleanup Only ");  // keep track of task type
      // std::cout << "Food Cleanup Only " << std::endl;

    }
    // Drink Cleanup Only
    else if (all_orders[i].drink_cleanup == true && all_orders[i].food_cleanup == false) {
      // Convert Table Number to Read into Config Files
      std::string table_num = "Table_" + std::to_string(all_orders[i].table_number);
      // Get End Task Location
      float start_x1 = mon_restaurant_config[table_num]["dropoff"]["x"];
      float start_y1 = mon_restaurant_config[table_num]["dropoff"]["y"];
      // Pushback Start Task Location Vector After Getting Individual Values
      start_task_loc.push_back(std::vector<float>());
      // Pushback Start Task x,y
      start_task_loc[i].push_back(start_x1);
      start_task_loc[i].push_back(start_y1);

      // Pushback Mid Task Placeholder Location Vector
      mid_task_loc.push_back(std::vector<float>());
      // Pushback Mid Task x,y
      mid_task_loc[i].push_back(-1);
      mid_task_loc[i].push_back(-1);

      // Pushback End Task Location Vector After Getting Individual Values
      end_task_loc.push_back(std::vector<float>());
      // Get End Task Location (Default Kitchen)
      float end_x1 = mon_restaurant_config["Glassware_cleaning"]["x"];
      float end_y1 = mon_restaurant_config["Glassware_cleaning"]["y"];
      // Pushback End Task x,y
      end_task_loc[i].push_back(end_x1);
      end_task_loc[i].push_back(end_y1);
      assigned_tasks.push_back(all_orders[i].table_number);  // keep track of assigned table
      assigned_orders.push_back(all_orders[i].order_number);  // keep track of assigned order
      order_types.push_back(false);  // storing the order type in a variable of the class
      task_type.push_back("Drink Cleanup Only ");  // keep track of task type
      // std::cout << "Drink Cleanup Only " << std::endl;
    }
  }
  // all_orders.erase(0, num_tasks);
  // std::cout<<"getTasks: start_task_loc.size()
  // "<<start_task_loc.size()<<std::endl; Remove Tasks to Be Assigned from Queue
  all_orders.erase(all_orders.begin(), all_orders.begin() + num_tasks);
  // Print Number of Remaining Orders
  // std::cout<<"getTasks: No. Remaining Orders:
  // "<<all_orders.size()<<std::endl; num_tasks = all_orders.size();
}

// Calculate Costs to Complete Tasks for Available Robots
void CostCalculation::cost_function() {
  // std::cout<<"Cost Func No. Tasks"<<num_tasks<<std::endl;
  // std::cout<<"Cost Func No. Robots"<<num_robots<<std::endl;

  // std::cout<<"Cost Func robot_loc.size() = "<<robot_loc.size()<<std::endl;
  // std::cout<<"Cost Func start_task_loc.size() =
  // "<<start_task_loc.size()<<std::endl;

  // Clear Cost Matrix Before Assigning New Values
  cost_matrix.clear();
  for (int i = 0; i < num_robots; i++) {
    // Push Vector into 2D Vector
    cost_matrix.push_back(std::vector<int>());
    // Calculate Costs for Each Task
    for (int j = 0; j < num_tasks; j++) {
      if (mid_task_loc[j][0] && mid_task_loc[j][1] == -1) {
        // std::cout<<"2 "<<std::endl;
        int cost = (int)(pow(start_task_loc[j][0] - robot_loc[i][0], 2) +
                         pow(start_task_loc[j][1] - robot_loc[i][1], 2) +
                         pow(start_task_loc[j][0] - end_task_loc[j][0], 2) +
                         pow(start_task_loc[j][1] - end_task_loc[j][1], 2));
        cost_matrix[i].push_back(cost);  // Push Sub-Vectors
      }
      else if (mid_task_loc[j][0] && mid_task_loc[j][1] != -1) {
        // std::cout<<"3 "<<std::endl;
        int cost = (int)(pow(start_task_loc[j][0] - robot_loc[i][0], 2) +
                         pow(start_task_loc[j][1] - robot_loc[i][1], 2) +
                         pow(start_task_loc[j][0] - mid_task_loc[j][0], 2) +
                         pow(start_task_loc[j][1] - mid_task_loc[j][1], 2) +
                         pow(mid_task_loc[j][0] - end_task_loc[j][0], 2) +
                         pow(mid_task_loc[j][1] - end_task_loc[j][1], 2));
        cost_matrix[i].push_back(cost);  // Push Sub-Vectors
      }
    }
  }
  // num_tasks = all_orders.size();
  // std::cout<<"NUM tasks after update in cost func "<<num_tasks<<std::endl;
}

// Get Random Assignments for Tasks
std::vector<int> CostCalculation::getRandomAssignments() {
  // Initialize Assignment Vector
  std::vector<int> rand_assign;
  // Create Assignment Vector
  for (int i = 0; i < num_tasks; i++) {
    rand_assign.push_back(i);
    std::cout<< "Not Random Assign" << rand_assign[i] <<std::endl; // Print Pre-Random
  }
  // Shuffle Assignment Vector to Randomize
  std::random_shuffle (rand_assign.begin(), rand_assign.end());

  for (int i = 0; i < num_tasks; i++) {
    std::cout<< "Random Assign" << rand_assign[i] <<std::endl; // Print Random
  }

  return rand_assign;
}