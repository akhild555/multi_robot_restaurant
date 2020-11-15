#include "ros/ros.h"
#include "std_msgs/String.h"
#include "hungarian.hpp"
#include "cost_function.h"
#include "std_msgs/Int32MultiArray.h"
#include <control_stack/RobotGoal.h>
#include <control_stack/KitchenOrders.h>
#include <control_stack/RobotDatabase.h>
#include <vector>
#include <sstream>
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
  ros::Subscriber robot_db = n.subscribe("/robot_database", 1000, &CostCalculation::getRobots, &cost_func);
  // Subscribe to /kitchen_state, Extract New Tasks
  ros::Subscriber kitchen_test = n.subscribe("/kitchen_state", 1000, &CostCalculation::all_tasks, &cost_func);
  // Create Publisher to Robot Goal
  ros::Publisher job_assignment = n.advertise<control_stack::RobotGoal>("/robot_goal", 1000);
  // Set Callback Function Rate
  ros::Rate loop_rate(40);

  while (ros::ok())
  {
    // std::cout<<"Num tasks in main func: "<<cost_func.num_tasks<<std::endl;

    // Check There Are Tasks to Allocate and Robots Available
    if(cost_func.num_tasks>0 && cost_func.num_robots>0)
    {
      
      // std::cout<<"Inside if condition main: num_tasks "<<cost_func.num_tasks<<std::endl;

      // Calculate Cost Matrix
      cost_func.cost_function();
      // Calculate Hungarian Matrix
      Hungarian::Result r = runHungarian(cost_func.cost_matrix, Hungarian::MODE_MINIMIZE_COST);
      // Print Hungarian Matrix
      Hungarian::PrintMatrix(r.assignment);
      // Get Job Assignment Vector
      std::vector<int> assignments = assignment_msg(r, cost_func);
      
      // Printing Out Job Assignment Vector 
      for(int i =0; i<assignments.size(); i++)
      {
        std::cout<<"assgn["<<i<<"]"<< assignments[i]<<std::endl;
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
        // Get Start Task Location 
        t.linear.x = cost_func.start_task_loc[assignments[i]][0];
        t.linear.y = cost_func.start_task_loc[assignments[i]][1];
        // Pushback Start Task Location
        robot_assgn.robot_goal.push_back(t);
        // Get End Task Location
        t.linear.x = cost_func.end_task_loc[assignments[i]][0];
        t.linear.y = cost_func.end_task_loc[assignments[i]][1];
        // Pushback Goal Task Location
        robot_assgn.robot_goal.push_back(t);
        // Publish Robot Goals
        job_assignment.publish(robot_assgn);
        // Clear Robot Goals, Prepare for Next Robot's Goals
        robot_assgn.robot_goal.clear();
      }
      // cost_func.all_orders.erase(cost_func.all_orders.begin(), cost_func.all_orders.begin() +cost_func.num_tasks);
      // std::cout<<"After erasing allocated tasks: all_orders.size()"<<cost_func.all_orders.size()<<std::endl;
      
      // cost_func.robot_loc.clear();
      // cost_func.start_task_loc.clear();
      // cost_func.end_task_loc.clear();
      // cost_func.cost_matrix.clear();
    }
    // Set num_tasks to Remaining Number of Orders
    cost_func.num_tasks = cost_func.all_orders.size();

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