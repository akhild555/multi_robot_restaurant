#include "ros/ros.h"
#include "std_msgs/String.h"
#include "hungarian.hpp"
#include "cost_function.h"
#include "std_msgs/Int32MultiArray.h"
#include <control_stack/RobotGoal.h>
#include <vector>
#include <sstream>
#include <string>



int main(int argc, char **argv)
{

  ros::init(argc, argv, "job_assignment");
  ros::NodeHandle n;
  ros::Publisher job_assignment = n.advertise<control_stack::RobotGoal>("/robot_goal", 1000);
  
  // Instantiate CostCalculation object
  CostCalculation cost_func;
  
  // set # robots and locations
  cost_func.getRobots();

  // set # tasks
  cost_func.getTasks();
  

  // get robot and task info
  int num_tasks = cost_func.num_tasks;
  std::vector<std::vector<float>> start_task_loc = cost_func.start_task_loc;
  int num_robots = cost_func.num_robots;
  std::vector<int> robots = cost_func.robots;
  std::vector<std::vector<float>> robot_loc = cost_func.robot_loc;
  std::vector<std::vector<float>> end_task_loc = cost_func.end_task_loc;
  
  // calculate cost matrix
  cost_func.cost_function(num_tasks, start_task_loc, num_robots, robots, robot_loc);
  
  // calculate job assignment
  const Hungarian::Matrix cost_matrix = cost_func.cost_matrix;
  Hungarian::Result r = runHungarian(cost_matrix, Hungarian::MODE_MINIMIZE_COST);
  
  // get job assignment vector
  std::vector<int> assignments = assignment_msg(r, cost_func);

  // for(int i =0; i<3; i++)
  //   {
  //       std::cout<<"assgn["<<i<<"]"<< assignments[i]<<std::endl;
  //   }

  // Subscribe to Kitchen State Topic
  // ros::Subscriber kitchen_state_sub =
  //     n.subscribe("/kitchen state", 1, &CostCalculation::getTasks, &cost_func);
  // Subscribe to Robot State Topic
  // ros::Subscriber robot_state_sub =
  //     n.subscribe("/robot state", 1, &CostCalculation::getRobots, &cost_func);
  // // Publish Cost Matrix 
  // ros::Publisher cost_matrix_pub =
  //     n.advertise<std_msgs::Float32MultiArray>("/cost_matrix", 1);

  
  
  ros::Rate loop_rate(1);

  
  int count = 0;
  while (count<2)
  {
    
    control_stack::RobotGoal robot_assgn;
    geometry_msgs:: Twist t;
		//Clear array
		// robot_assgn.data.clear();
		//for loop, pushing data in the size of the array
		// for (int i = 0; i < num_robots; i++)
		// {
		// 	assgn_array.data.push_back(assignments[i]);
		// }

    for (int i = 0; i < assignments.size(); i++)
    {
      robot_assgn.robot_index = robots[i];      
      t.linear.x = start_task_loc[assignments[i]][0];
      t.linear.y = start_task_loc[assignments[i]][1];
      robot_assgn.robot_goal.push_back(t);
      t.linear.x = end_task_loc[assignments[i]][0];
      t.linear.y = end_task_loc[assignments[i]][1];
      robot_assgn.robot_goal.push_back(t);
      job_assignment.publish(robot_assgn);
      robot_assgn.robot_goal.clear();

    }
    
		//Publish array
		
    ROS_INFO("I published something!");
    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }

  
}