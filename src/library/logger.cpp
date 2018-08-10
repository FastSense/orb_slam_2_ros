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

void LoggerMaster::print_time(const ros::Time &current_time) {
//    unsigned int dt_sec = current_time.sec - start_time.sec;
//    unsigned int dt_ms = (current_time.nsec - start_time.nsec)/1000000;
//    log_stream << dt_sec << "." << dt_ms << ",";

    ros::Duration delta=current_time-start_time;

    //log_stream << delta.sec << "." << delta.nsec/1000000 << ",";

//    double msec=delta.toNSec()/1000000;
//    int sec=msec/1000;
//    msec-=sec*1000;
//    log_stream << sec << "." << msec << ",";

    log_stream << std::to_string((double)delta.toNSec()/1000000000.0) << ",";

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

    path_pub_ = nh_.advertise<std_msgs::String>("master_log_path", 1, true);
    std_msgs::String msg;
    msg.data = log_path;
    path_pub_.publish(msg);
    ROS_WARN("LoggerMaster: published path -> %s", log_path.c_str());

    log_file = log_path + "slam.log";
    log_stream.open(log_file);
    if (!log_stream.is_open()){
        ROS_ERROR("Error opening log file -> %s",log_file.c_str());
        return;
    }

    start_time = ros::Time::now();

};

LoggerMaster::~LoggerMaster() {
    log_stream.close();
}



// Checks if a matrix is a valid rotation matrix.
bool isRotationMatrix(cv::Mat &R)
{
    cv::Mat Rt;
    cv::transpose(R, Rt);
    cv::Mat shouldBeIdentity = Rt * R;
    cv::Mat I = cv::Mat::eye(3,3, shouldBeIdentity.type());

    return  cv::norm(I, shouldBeIdentity) < 1e-6;

}

// Calculates rotation matrix to euler angles
// The result is the same as MATLAB except the order
// of the euler angles ( x and z are swapped ).
cv::Vec3f rotationMatrixToEulerAngles(cv::Mat &R)
{
    assert(isRotationMatrix(R));

    float sy = sqrt(R.at<float>(0,0) * R.at<float>(0,0) +  R.at<float>(1,0) * R.at<float>(1,0) );

    bool singular = sy < 1e-6; // If

    float x, y, z;
    if (!singular)
    {
        x = atan2(R.at<float>(2,1) , R.at<float>(2,2));
        y = atan2(-R.at<float>(2,0), sy);
        z = atan2(R.at<float>(1,0), R.at<float>(0,0));
    }
    else
    {
        x = atan2(-R.at<float>(1,2), R.at<float>(1,1));
        y = atan2(-R.at<float>(2,0), sy);
        z = 0;
    }
    return cv::Vec3f(x, y, z);
}

// ts, slam state, x, y, z, yaw, pith, roll, track time, x_init, y_init, z_init, yaw_init, pith_init, roll_init
// The industry standard is Z-Y-X because that corresponds to yaw, pitch and roll
void LoggerMaster::log_slam_data(const ros::Time &current_time,
                                 const ros::Duration &proc_time ,
                                 int slam_state,
                                 bool track_ok,
                                 const geometry_msgs::Transform &slam_pose,
                                 const geometry_msgs::Transform &init_pose,
                                 const geometry_msgs::Transform &px4_estimation)
{
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    double yaw = 0.0;
    double pitch = 0.0;
    double roll = 0.0;

    double x_i = 0.0;
    double y_i = 0.0;
    double z_i = 0.0;
    double yaw_i = 0.0;
    double pitch_i = 0.0;
    double roll_i = 0.0;

    double x_p = 0.0;
    double y_p = 0.0;
    double z_p = 0.0;
    double yaw_p = 0.0;
    double pitch_p = 0.0;
    double roll_p = 0.0;

    if (track_ok) {

        tf2::Quaternion Qtrn;
        double Roll, Pitch, Yaw;
        Qtrn.setValue(slam_pose.rotation.x,slam_pose.rotation.y,
                      slam_pose.rotation.z,slam_pose.rotation.w);
        tf2::Matrix3x3(Qtrn).getRPY(Roll, Pitch, Yaw);

        x = slam_pose.translation.x;
        y = slam_pose.translation.y;
        z = slam_pose.translation.z;
        yaw = Yaw;
        pitch = Pitch;
        roll = Roll;

        Qtrn.setValue(init_pose.rotation.x,init_pose.rotation.y,
                      init_pose.rotation.z,init_pose.rotation.w);
        tf2::Matrix3x3(Qtrn).getRPY(Roll, Pitch, Yaw);

        x_i = init_pose.translation.x;
        y_i = init_pose.translation.y;
        z_i = init_pose.translation.z;
        yaw_i = Yaw;
        pitch_i = Pitch;
        roll_i = Roll;


        Qtrn.setValue(px4_estimation.rotation.x,px4_estimation.rotation.y,
                      px4_estimation.rotation.z,px4_estimation.rotation.w);
        tf2::Matrix3x3(Qtrn).getRPY(Roll, Pitch, Yaw);

        x_p = px4_estimation.translation.x;
        y_p = px4_estimation.translation.y;
        z_p = px4_estimation.translation.z;
        yaw_p = Yaw;
        pitch_p = Pitch;
        roll_p = Roll;




//        if (!RT.empty()) {
//            cv::Mat RTI = RT.inv();
//
//            //        ROS_WARN("RT: w -> %d, h -> %d", RT.cols, RT.rows);
//            //        for (int j = 0; j < RT.rows; j++)
//            //            ROS_WARN("%f\t%f\t%f\t%f", RT.at<float>(j, 0), RT.at<float>(j, 1), RT.at<float>(j, 2), RT.at<float>(j, 3));
//
//            //std::cout << RTI << std::endl;
//            // extract x, y, z
//            x = RTI.at<float>(0, 3);
//            y = RTI.at<float>(1, 3);
//            z = RTI.at<float>(2, 3);
//
//
//            // get rotation matrix
//            cv::Mat R = RTI(cv::Rect(0, 0, 3, 3));
//
//            //        ROS_WARN("R: w -> %d, h -> %d", R.cols, R.rows);
//            //        for (int j = 0; j < R.rows; j++)
//            //            ROS_WARN("%f\t%f\t%f\n", R.at<double>(j, 0), R.at<double>(j, 1), R.at<double>(j, 2));
//
//            // extract yaw, pitch, roll
//            cv::Vec3f angles = rotationMatrixToEulerAngles(R);
//            yaw = angles[2];
//            pitch = angles[1];
//            roll = angles[0];
//        }
//
//        if (!InitPose.empty()) {
//            cv::Mat InitPoseI = InitPose.inv();
//
//            //        ROS_WARN("RT: w -> %d, h -> %d", RT.cols, RT.rows);
//            //        for (int j = 0; j < RT.rows; j++)
//            //            ROS_WARN("%f\t%f\t%f\t%f", RT.at<float>(j, 0), RT.at<float>(j, 1), RT.at<float>(j, 2), RT.at<float>(j, 3));
//
//            //std::cout << RTI << std::endl;
//            // extract x, y, z
//            x_i = InitPoseI.at<float>(0, 3);
//            y_i = InitPoseI.at<float>(1, 3);
//            z_i = InitPoseI.at<float>(2, 3);
//
//
//            // get rotation matrix
//            cv::Mat InitR = InitPoseI(cv::Rect(0, 0, 3, 3));
//
//            //        ROS_WARN("R: w -> %d, h -> %d", R.cols, R.rows);
//            //        for (int j = 0; j < R.rows; j++)
//            //            ROS_WARN("%f\t%f\t%f\n", R.at<double>(j, 0), R.at<double>(j, 1), R.at<double>(j, 2));
//
//            // extract yaw, pitch, roll
//            cv::Vec3f angles = rotationMatrixToEulerAngles(InitR);
//            yaw_i = angles[2];
//            pitch_i = angles[1];
//            roll_i = angles[0];
//        }
    }

    // print log
    print_time(current_time);
    log_stream << slam_state << ",";
    log_stream << x << "," << y << "," << z << ",";
    log_stream << yaw << "," << pitch << "," << roll <<",";
    log_stream << std::to_string((double)proc_time.toNSec()/1000000000.0) <<  ",";

    log_stream << x_i << "," << y_i << "," << z_i << ",";
    log_stream << yaw_i << "," << pitch_i << "," << roll_i <<",";

    log_stream << x_p << "," << y_p << "," << z_p << ",";
    log_stream << yaw_p << "," << pitch_p << "," << roll_p <<"\n";


    log_stream.flush();
}