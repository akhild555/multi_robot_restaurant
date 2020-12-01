#pragma once

#include "ros/ros.h"
#include "control_stack/KitchenOrders.h"
#include "control_stack/RobotGoal.h"
#include "control_stack/RobotPosition.h"
#include <fstream>

struct LogData {
    int order_num = -1;
    int robot_id = -1;
    ros::Time task_generation_time;
    ros::Time task_allocation_time;
    ros::Time task_completion_time;
};

class DataLogger {

    ros::Subscriber task_generation_subscriber;
    ros::Subscriber task_allocator_subscriber;
    ros::Subscriber robot_status_subscriber;
    
    std::vector<LogData> log_buffer;
    std::string log_file_name;
    std::string buffer_file_name;
    std::string file_header;

    void TaskGenerationCallback(control_stack::KitchenOrders generated_task) {
        LogData log_data;
        log_data.order_num = generated_task.order_number; 
        log_data.task_generation_time = generated_task.stamp; 
        log_buffer.push_back(log_data);
    }

    void TaskAllocationCallback(control_stack::RobotGoal allocated_task) {
        ROS_DEBUG("TaskAllocationCallback order num %i", allocated_task.order_number);
        for(LogData& log_data: log_buffer) {
            if (log_data.order_num == allocated_task.order_number) {
                log_data.robot_id = allocated_task.robot_index; 
                log_data.task_allocation_time = allocated_task.stamp; 
                ROS_DEBUG("Set robot id to %d", allocated_task.robot_index);
                return;
            }
        }
    }
    
    void RobotStatusCallback(control_stack::RobotPosition robot_status) {
        for(int i=0; i<log_buffer.size(); i++) {
            if (log_buffer[i].order_num == robot_status.order_number && !robot_status.robot_active) { 
                log_buffer[i].task_completion_time = robot_status.stamp; 
                // write to log file and remove from buffer
                append_to_file(log_buffer[i], log_file_name);
                log_buffer.erase(log_buffer.begin() + i);
                write_to_file(log_buffer, buffer_file_name);
                return;
            }
        }
    }
    
    std::string get_log_entry(LogData log_data) {
        std::string entry;
        int generate_time = log_data.task_generation_time.sec;
        int alloc_time    = log_data.task_allocation_time.sec;
        int deliver_time = log_data.task_completion_time.sec;
        
        entry += std::to_string(log_data.order_num) + ",";
        entry += std::to_string(log_data.robot_id) + ",";
        entry += std::to_string(generate_time) + ",";
        entry += std::to_string(alloc_time) + ",";
        entry += std::to_string(deliver_time) + ",";

        int time_to_alloc = alloc_time - generate_time; // time to allocate a task
        time_to_alloc = (time_to_alloc<0) ? -1 : time_to_alloc;

        int time_to_deliver = deliver_time - alloc_time; // time for robot to complete a task
        time_to_deliver = (time_to_deliver<0) ? -1 : time_to_deliver;

        int total_time = time_to_alloc + time_to_deliver; // total time a customer waits
        total_time = (time_to_deliver == -1) ? -1 : total_time;
        
        entry += std::to_string(time_to_alloc) + ","; 
        entry += std::to_string(time_to_deliver) + ","; 
        entry += std::to_string(total_time) + "\n"; 

        return entry;
    }

    void append_to_file(LogData log_data, std::string file_name) {
        std::ofstream file;
        file.open(file_name.c_str(), std::ios_base::app);
        std::string entry = get_log_entry(log_data);
        ROS_INFO("Adding log : %s", entry.c_str());
        file << entry.c_str();
    }
    
    void write_to_file(std::vector<LogData> log_buffer, std::string file_name) {
        std::ofstream file;
        file.open(file_name.c_str());
        file << file_header.c_str();
        for (LogData log_data: log_buffer) {
            std::string entry = get_log_entry(log_data);
            file << entry.c_str();
        }
    }

    void write_to_file(std::string str, std::string file_name) {
        std::ofstream file;
        file.open(file_name.c_str());
        file << str.c_str();
    }
 
    public:

    DataLogger(ros::NodeHandle& nh) {

        task_generation_subscriber = nh.subscribe("/kitchen_state",
                 100, &DataLogger::TaskGenerationCallback, this);

        task_allocator_subscriber = nh.subscribe("/robot_goal",
                 100, &DataLogger::TaskAllocationCallback, this);

        robot_status_subscriber = nh.subscribe("/robot_position",
                 100, &DataLogger::RobotStatusCallback, this);

        log_file_name = "completed_task_log";
        buffer_file_name = "incomplete_task_log";
        file_header = "order, robot, generate time, allocate time, deliver time, time to allocate, time to deliver, total time\n";
        write_to_file(file_header, log_file_name);
        write_to_file(".", buffer_file_name);

    }

};