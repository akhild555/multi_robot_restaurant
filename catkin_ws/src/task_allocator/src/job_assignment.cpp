#include "ros/ros.h"
#include "std_msgs/String.h"
#include "hungarian.hpp"
#include <vector>
#include <sstream>
#include <string>
#include "std_msgs/Int32MultiArray.h"

int main(int argc, char **argv)
{

  ros::init(argc, argv, "job_assignment");
  ros::NodeHandle n;
  ros::Publisher job_assignment = n.advertise<std_msgs::Int32MultiArray>("assgn_array", 1000);
  ros::Rate loop_rate(10);

  
  int count = 0;
  while (1)
  {
    
    // std_msgs::String msg;

    // std::stringstream ss;
    // ss << "hello world " << count;
    // msg.data = ss.str();

    // ROS_INFO("%s", msg.data.c_str());

    // run hungarian method on example cost matrices
    std::vector<int> assignment = runHungarian();
    int n_robo = assignment.size();
    std_msgs::Int32MultiArray assgn_array;
		//Clear array
		assgn_array.data.clear();
		//for loop, pushing data in the size of the array
		for (int i = 0; i < n_robo; i++)
		{
			//assign array a random number between 0 and 255.
			assgn_array.data.push_back(assignment[i]);
		}
		//Publish array
		job_assignment.publish(assgn_array);
    ROS_INFO("I published something!");
    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }

  
}