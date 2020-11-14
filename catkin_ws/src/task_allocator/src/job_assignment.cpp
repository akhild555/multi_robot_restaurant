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
  ros::Subscriber robot_db = n.subscribe("/robot_database", 1000, &CostCalculation::getRobots, &cost_func);
  ros::Subscriber kitchen_test = n.subscribe("/kitchen_state", 1000, &CostCalculation::all_tasks, &cost_func);
  ros::Publisher job_assignment = n.advertise<control_stack::RobotGoal>("/robot_goal", 1000);
  ros::Rate loop_rate(40);

  while (ros::ok())
  {
    // std::cout<<"Num tasks in main func: "<<cost_func.num_tasks<<std::endl;
    if(cost_func.num_tasks>0 && cost_func.num_robots>0) 
    {
      
      // calculate cost matrix
      std::cout<<"Inside if condintion main: num_tasks "<<cost_func.num_tasks<<std::endl;
      cost_func.cost_function();
      // const Hungarian::Matrix cost_x = ;
      Hungarian::Result r = runHungarian(cost_func.cost_matrix, Hungarian::MODE_MINIMIZE_COST);
      Hungarian::PrintMatrix(r.assignment);
      // get job assignment vector
      std::vector<int> assignments = assignment_msg(r, cost_func);
      // Hungarian::PrintMatrix(assignments);
      for(int i =0; i<assignments.size(); i++)
      {
        std::cout<<"assgn["<<i<<"]"<< assignments[i]<<std::endl;
      }
      control_stack::RobotGoal robot_assgn;
      geometry_msgs:: Twist t;

      for (int i = 0; i < assignments.size(); i++)
      {
        robot_assgn.robot_index = cost_func.robots[i];      
        t.linear.x = cost_func.start_task_loc[assignments[i]][0];
        t.linear.y = cost_func.start_task_loc[assignments[i]][1];
        robot_assgn.robot_goal.push_back(t);
        t.linear.x = cost_func.end_task_loc[assignments[i]][0];
        t.linear.y = cost_func.end_task_loc[assignments[i]][1];
        robot_assgn.robot_goal.push_back(t);
        job_assignment.publish(robot_assgn);
        robot_assgn.robot_goal.clear();
      }
      // cost_func.all_orders.erase(cost_func.all_orders.begin(), cost_func.all_orders.begin() +cost_func.num_tasks);
      // std::cout<<"After erasing allocated tasks: all_orders.size()"<<cost_func.all_orders.size()<<std::endl;
      
      // cost_func.robot_loc.clear();
      // cost_func.start_task_loc.clear();
      // cost_func.end_task_loc.clear();
      // cost_func.cost_matrix.clear();
    }
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