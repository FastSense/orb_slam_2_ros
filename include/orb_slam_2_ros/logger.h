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
#include <cv_bridge/cv_bridge.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

class LoggerMaster {
public:
    LoggerMaster(ros::NodeHandle& private_nh);
    ~LoggerMaster();

    void get_log_path(std::string &log_path);
    void log_slam_data(const ros::Time &current_time,
                       const ros::Duration &proc_time,
                       int slam_state,
                       bool track_ok,
                       const geometry_msgs::Transform &slam_pose,
                       const geometry_msgs::Transform &init_pose,
                       const geometry_msgs::Transform &px4_estimation

    );

private:
    ros::NodeHandle nh_;
    ros::Publisher path_pub_;
    std::string top_log_path;
    std::string log_path;
    ros::Publisher log_path_pub;
    std::string log_file;
    std::ofstream log_stream;
    ros::Time start_time;

    // Get current date/time, format is YYYY-MM-DD.HH:mm:ss
    const std::string currentDateTime();
    void print_time(const ros::Time &current_time);
};

#endif //ORB_SLAM_2_ROS_LOGGER_H
