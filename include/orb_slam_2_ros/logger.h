//
// Created by urock on 19.07.18.
//

#ifndef ORB_SLAM_2_ROS_LOGGER_H
#define ORB_SLAM_2_ROS_LOGGER_H

#include <ros/ros.h>
#include <sys/stat.h>
#include<fstream>
#include<iostream>

#include <std_msgs/Int32.h>
#include "std_msgs/String.h"

class LoggerMaster {
public:
    LoggerMaster(ros::NodeHandle& private_nh);

    void get_log_path(std::string &log_path);

private:
    ros::NodeHandle nh_;
    std::string top_log_path;
    std::string log_path;
    ros::Publisher log_path_pub;

    // Get current date/time, format is YYYY-MM-DD.HH:mm:ss
    const std::string currentDateTime();
    void print_time(std::ofstream &base_link_file_, ros::Time &start_time, ros::Time &current_time);
};

#endif //ORB_SLAM_2_ROS_LOGGER_H
