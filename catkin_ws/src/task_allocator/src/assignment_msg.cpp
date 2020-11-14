#include "ros/ros.h"
#include "std_msgs/String.h"
#include "hungarian.hpp"
#include "cost_function.h"
#include "std_msgs/Int32MultiArray.h"
#include <vector>
#include <sstream>
#include <string>

std::vector<int> assignment_msg(Hungarian::Result result, CostCalculation cost_func)
{
    Hungarian::Matrix assignment = result.assignment;

    // const int n_robo = cost_func.num_robots; //no of robots
    // const int n_tasks = cost_func.num_tasks; //no of tasks
    int effective_alloc = cost_func.num_robots;
    if(cost_func.num_tasks < cost_func.num_robots)
    {
        effective_alloc = cost_func.num_tasks;
    }
    std::vector<int> assgn (effective_alloc, 0);
    // std::vector<int> assgn (n_robo, 0);
    for(int i =0; i<assgn.size(); i++)
    {
        for(int j = 0; j<cost_func.num_tasks; j++)
        {
        //cout<<result[i][j]<<endl;
            if(assignment[i][j]!=0)
            {
                assgn[i] = j;
            }     
        }
    }
    for(int i =0; i<assgn.size(); i++)
    {
        std::cout<<"from assignment msg"<<std::endl;
        std::cout<<"assgn["<<i<<"]"<< assgn[i]<<std::endl;
    }
    
    return assgn;
}

//  int effective_alloc = n_robo;
//     if(n_tasks<n_robo)
//     {
//         effective_alloc = n_tasks;
//     }
//     std::vector<int> assgn (effective_alloc, 0);
//     for(int i =0; i<n_robo; i++)
//     {
//         for(int j = 0; j<n_tasks; j++)
//         {
//             //cout<<result[i][j]<<endl;
//             if(assignment[i][j]!=0)
//             {
//                 assgn[i] = j;
//             }     
//         }
//     }