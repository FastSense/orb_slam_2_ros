//
// Created by urock on 19.07.18.
//


#include "orb_slam_2_ros/logger.h"


// Get current date/time, format is YYYY-MM-DD.HH:mm:ss
const std::string LoggerMaster::currentDateTime() {
    time_t     now = time(0);
    struct tm  tstruct;
    char       buf[80];
    tstruct = *localtime(&now);
    // Visit http://en.cppreference.com/w/cpp/chrono/c/strftime
    // for more information about date/time format
    strftime(buf, sizeof(buf), "%Y-%m-%d.%X", &tstruct);
    return buf;
}

void LoggerMaster::print_time(std::ofstream &base_link_file_, ros::Time &start_time, ros::Time &current_time) {
    unsigned int dt_sec = current_time.sec - start_time.sec;
    unsigned int dt_ms = (current_time.nsec - start_time.nsec)/1000000;
    base_link_file_ << dt_sec << "." << dt_ms << ",";
}

void LoggerMaster::get_log_path(std::string &log_path_out) {
    log_path_out = log_path;
}


LoggerMaster::LoggerMaster(ros::NodeHandle& private_nh) {

    ROS_WARN("Hello from Logger Master");

    log_path = "No_path";

    do {
        if (private_nh.getParam("log_path", top_log_path)) {
            ROS_INFO("Got param log_path: %s", top_log_path.c_str());
        } else {
            ROS_ERROR("Failed to get param 'log_path'");
            break;
        }

        std::string current_date = currentDateTime();

        log_path = top_log_path + current_date + '/';

        if (mkdir(log_path.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH)) {
            ROS_ERROR("Error creating log directory %s", log_path.c_str());
        }

    } while(0);


};