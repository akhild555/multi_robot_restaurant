#include "ros/ros.h"
#include "std_msgs/String.h"
#include "hungarian.hpp"
#include "cost_function.h"
#include "std_msgs/Int32MultiArray.h"
#include <control_stack/RobotGoal.h>
#include <control_stack/KitchenOrders.h>
#include <control_stack/RobotDatabase.h>
#include "json.h"
#include <vector>
#include <sstream>
#include <fstream>
#include <string>

void test_callback_1(const control_stack::RobotDatabase& msg)
{
  std::cout << "in test callback database" << std::endl;
}
void test_callback_2(const control_stack::KitchenOrders& msg)
{
  std::cout << "in test callback kitchen" << std::endl;
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "job_assignment");
  ros::NodeHandle n;
  // Instantiate CostCalculation object
  CostCalculation cost_func;
  // Subscribe to /robot_database, Extract Current Robot Positions
  ros::Subscriber robot_db = n.subscribe("/robot_database", 1, &CostCalculation::getRobots, &cost_func);
  // Subscribe to /kitchen_state, Extract New Tasks
  ros::Subscriber kitchen_test = n.subscribe("/kitchen_state", 1000, &CostCalculation::all_tasks, &cost_func);
  // Create Publisher to Robot Goal
  ros::Publisher job_assignment = n.advertise<control_stack::RobotGoal>("/robot_goal", 1000);
  // Set Callback Function Rate
  ros::Rate loop_rate(1);
  std::vector<std::vector<float>> wait_loc = {{10.5, 0.73}, {12, 0.73}, {10, 5.12}, {10, 6.62}};
  std::vector<int> wait_occ = {0,0,0,0};
  std::vector<int> rob_occ = {0,0,0,0};
  // Set Config File to Be Used
  std::ifstream file_input("src/control_stack/config/monarch_config.json");
  nlohmann::json mon_restaurant_config;
  file_input >> mon_restaurant_config;
  // Get Allocation Algorithm Booleans
  bool hungarian_alg = mon_restaurant_config["Allocation_algorithm"]["Hungarian"];
  bool random_alg = mon_restaurant_config["Allocation_algorithm"]["Random"];
  // Staggered allocation flags
  bool stagger = mon_restaurant_config["Staggered allocation"]["stagger"];
  int stagger_num = mon_restaurant_config["Staggered allocation"]["stagger_num"];
  int stagger_n_rob;
  if (stagger){
    // how many free robots to wait for before starting allocation
    stagger_n_rob = stagger_num-1;
  }
  else{
    stagger_n_rob = 0;
  }

  while (ros::ok())
  {
    // std::cout<<"Num tasks in main func: "<<cost_func.num_tasks<<std::endl;

    // Check There Are Tasks to Allocate and Robots Available
    // std::cout<<"job_assignment: No. Remaining Orders: "<< cost_func.all_orders.size()<<std::endl;
    // std::cout<<"job_assignment: Num robots: "<< cost_func.num_robots<<std::endl;

    if(cost_func.all_orders.size() && cost_func.num_robots>stagger_n_rob) // Assign when stagger_n_robots are free
    // if(cost_func.all_orders.size()>1 && cost_func.num_robots>1) // Assign 2 robots, Bug: Robot 1 goes to Robot 0 table
    // if(cost_func.all_orders.size() && cost_func.num_robots>0)
    {
      // std::cout<<"Inside if condition main: num_tasks "<<cost_func.num_tasks<<std::endl;
      cost_func.getTasks(); // Get Tasks
      std::vector<int> assignments; // Initialize Assignments Vector
      // Check Algorithm for Assignment
      if (hungarian_alg) // Use Hungarian Algorithm
      {
        // Calculate Cost Matrix
        cost_func.cost_function();
        // Calculate Hungarian Matrix
        Hungarian::Result r = runHungarian(cost_func.cost_matrix, Hungarian::MODE_MINIMIZE_COST);
        // Print Cost Matrix
        // std::cout << "Cost Matrix"<< std::endl;
        // Hungarian::PrintMatrix(r.cost);
        // Print Hungarian Matrix
        // std::cout << "Assignment Matrix"<< std::endl;
        // Hungarian::PrintMatrix(r.assignment);
        // Get Job Assignment Vector
        assignments = assignment_msg(r, cost_func);
      
        // Printing Out Job Assignment Vector 
        // for(int i =0; i<assignments.size(); i++)
        // {
        //   std::cout<<"assgn["<<i<<"]"<< assignments[i]<<std::endl;
        // }
      }
      else if (random_alg) // Perform Random Assignments
      {
        assignments = cost_func.getRandomAssignments();
      }

      // Create Robot Goal Msg
      control_stack::RobotGoal robot_assgn;
      // Create Sub-Robot Goal Msg
      geometry_msgs:: Twist t;
      // Loop Through Assignments
      for (int i = 0; i < assignments.size(); i++)
      {
        // Get Robot Index
        robot_assgn.robot_index = cost_func.robots[i];
        // Get Table Number
        robot_assgn.table_number = cost_func.assigned_tasks[assignments[i]];
        std::cout << "Robot " << robot_assgn.robot_index << " is assigned to Table " << robot_assgn.table_number << std::endl;
        // Get Start Task Location 
        t.linear.x = cost_func.start_task_loc[assignments[i]][0];
        t.linear.y = cost_func.start_task_loc[assignments[i]][1];
        // Pushback Mid Task Location
        robot_assgn.robot_goal.push_back(t);
        if (cost_func.mid_task_loc[assignments[i]][0] && cost_func.mid_task_loc[assignments[i]][1] != -1){
          t.linear.x = cost_func.mid_task_loc[assignments[i]][0];
          t.linear.y = cost_func.mid_task_loc[assignments[i]][1];
          robot_assgn.robot_goal.push_back(t);
        }

        // Get End Task Location
        t.linear.x = cost_func.end_task_loc[assignments[i]][0];
        t.linear.y = cost_func.end_task_loc[assignments[i]][1];
        // Pushback Goal Task Location
        robot_assgn.robot_goal.push_back(t);

        // Staging Location Code
        if (stagger){ 
        //find closest wait location

          int min_dist = 1000;
          int wait_ind = 0;
          for (int j = 0; j < wait_loc.size(); j++) {
            //Reset wait_occ and rob_occ if the robot is being allocated a task
            if (wait_occ[j] != 0){
              if (rob_occ[j] == robot_assgn.robot_index){
                wait_occ[j] = 0;
                rob_occ[j] = 0;
              }
            }

            if (wait_occ[j] == 0){
              int wait_dist = (int)(pow(t.linear.x - wait_loc[j][0], 2) +
                                pow(t.linear.y - wait_loc[j][1], 2));

              if (wait_dist < min_dist){
                min_dist = wait_dist;
                wait_ind = j;
              }
            }
          }
        
          t.linear.x = wait_loc[wait_ind][0];
          t.linear.y = wait_loc[wait_ind][1];
          wait_occ[wait_ind] = 1;
          rob_occ[wait_ind] = robot_assgn.robot_index;
          // for (int k = 0; k < wait_occ.size(); k++) {
          //   // std::cout<<"Wait occ : " << wait_occ[k];
          //   // std::cout<<"   Robot occ : " << rob_occ[k] << std::endl;
          // }
          // Pushback Wait Location
          robot_assgn.robot_goal.push_back(t);
        }

        // Add Time Stamp
        robot_assgn.stamp = ros::Time::now();
        // Publish order number
        robot_assgn.order_number = cost_func.assigned_orders[i];
        std::stringstream ss;
        if(cost_func.order_types[i] == true){
          std::cout<<"Order Type order"<< std::endl;
          // ROS_INFO("%s", ss.str().c_str());
          robot_assgn.order = true;
        }
        else if(cost_func.order_types[i] == false){
          std::cout<<"Order Type cleanup"<< std::endl;
          // ROS_INFO("%s", ss.str().c_str());
          robot_assgn.order = false;
        }
        std::cout << "Published order number = " << robot_assgn.order_number  << std::endl;
        std::cout << cost_func.task_type[assignments[i]] << std::endl<<std::endl; // Publish Type of Task

        cost_func.task_alloc_memory[robot_assgn.robot_index] = robot_assgn.order_number;
        // std::stringstream ss;
        // std::cout<<"Order "<<robot_assgn.order_number<<" Robot "<<robot_assgn.robot_index<<std::endl<<std::endl;
        // ROS_INFO("%s",ss.str().c_str());

        // Publish Robot Goals
        job_assignment.publish(robot_assgn);
        // Clear Robot Goals, Prepare for Next Robot's Goals
        robot_assgn.robot_goal.clear();
        cost_func.pending_orders.push_back(cost_func.assigned_orders[i]);
      }
      // cost_func.all_orders.erase(cost_func.all_orders.begin(), cost_func.all_orders.begin() +cost_func.num_tasks);
      // std::cout<<"After erasing allocated tasks: all_orders.size()"<<cost_func.all_orders.size()<<std::endl;
      // cost_func.robot_loc.clear();
      // cost_func.start_task_loc.clear();
      // cost_func.end_task_loc.clear();
      // cost_func.cost_matrix.clear();
    }
    // Set num_tasks to Remaining Number of Orders
    // cost_func.num_tasks = cost_func.all_orders.size();

    // ROS_INFO("In cost func while loop!");
    ros::spinOnce();
    loop_rate.sleep();
  }


  
  // int count = 0;
  // while (ros::ok())
  // {
    
  //   control_stack::RobotGoal robot_assgn;
  //   geometry_msgs:: Twist t;

  //   for (int i = 0; i < assignments.size(); i++)
  //   {
  //     robot_assgn.robot_index = robots[i];      
  //     t.linear.x = start_task_loc[assignments[i]][0];
  //     t.linear.y = start_task_loc[assignments[i]][1];
  //     robot_assgn.robot_goal.push_back(t);
  //     t.linear.x = end_task_loc[assignments[i]][0];
  //     t.linear.y = end_task_loc[assignments[i]][1];
  //     robot_assgn.robot_goal.push_back(t);
  //     job_assignment.publish(robot_assgn);
  //     robot_assgn.robot_goal.clear();

  //   }
    
	// 	//Publish array
		
  //   ROS_INFO("I published something!");
  //   ros::spinOnce();

  //   loop_rate.sleep();
  //   ++count;
  // }

  // ros::spin();
  return 0;
}