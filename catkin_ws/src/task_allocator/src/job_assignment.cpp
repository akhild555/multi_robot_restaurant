#include "ros/ros.h"
#include "std_msgs/String.h"
#include "hungarian.hpp"
#include <vector>
#include <sstream>
#include <string>

int main(int argc, char **argv)
{

  ros::init(argc, argv, "job_assignment");
  ros::NodeHandle n;
  ros::Publisher job_assignment = n.advertise<std_msgs::String>("job_assignment", 1000);
  ros::Rate loop_rate(10);

  
  int count = 0;
  while (count < 1)
  {
    
    // std_msgs::String msg;

    // std::stringstream ss;
    // ss << "hello world " << count;
    // msg.data = ss.str();

    // ROS_INFO("%s", msg.data.c_str());

    // run hungarian method on example cost matrices
    runHungarian();

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }

  
}