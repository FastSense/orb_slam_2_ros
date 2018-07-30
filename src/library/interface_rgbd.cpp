#include <cv_bridge/cv_bridge.h>

#include "orb_slam_2_ros/interface_rgbd.h"

namespace orb_slam_2_interface {


OrbSlam2InterfaceRgbd::OrbSlam2InterfaceRgbd(const ros::NodeHandle& nh,
                                                 const ros::NodeHandle& nh_private)
    : OrbSlam2Interface(nh, nh_private) {

    ROS_WARN("Urock OrbSlam2InterfaceRgbd");

    // Getting data and params
    subscribeToTopics();
    //advertiseTopics();
//    getParametersFromRos();
    slam_system_ = std::shared_ptr<ORB_SLAM2::System>(
        new ORB_SLAM2::System(vocabulary_file_path_, settings_file_path_,
                              ORB_SLAM2::System::RGBD, true));

    rgbImage = cv::Mat(480,640,CV_8UC1);
}

void OrbSlam2InterfaceRgbd::subscribeToTopics() {


    rgb_sub_ = nh_.subscribe("camera/left/image_raw", 1,
                               &OrbSlam2InterfaceRgbd::rgbCallback, this);
    depth_sub_ = nh_.subscribe("camera/depth/disparity", 1,
                             &OrbSlam2InterfaceRgbd::depthCallback, this);

}

void OrbSlam2InterfaceRgbd::rgbCallback(const sensor_msgs::ImageConstPtr& msg) {
//    ROS_ERROR("OrbSlam2InterfaceRgbd::rgbCallback start");
    cv_bridge::CvImageConstPtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvShare(msg);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv_ptr->image.copyTo(rgbImage);
    rgb_header = msg->header;
//    ROS_ERROR("OrbSlam2InterfaceRgbd::rgbCallback end");
}

void OrbSlam2InterfaceRgbd::depthCallback(const sensor_msgs::ImageConstPtr& depth_msg) {
//    ROS_ERROR("OrbSlam2InterfaceRgbd::depthCallback start");
//    ROS_ERROR("OrbSlam2InterfaceRgbd:: depth size %d, %d, type -> %s", depth_msg.get()->height,depth_msg.get()->width, depth_msg.get()->encoding.c_str());
    cv_bridge::CvImageConstPtr depth_ptr;
    try {
        depth_ptr = cv_bridge::toCvShare(depth_msg);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
//    ROS_ERROR("rs_depth_imageCallback: depth_float_img size %d, %d, step -> %d", depth_float_img.cols, depth_float_img.rows, depth_float_img.step[0]);

    ros::Time BeforeProc=ros::Time::now();

//    ROS_ERROR("OrbSlam2InterfaceRgbd:: rgb size %d, %d", rgbImage.cols, rgbImage.rows);
//    ROS_ERROR("OrbSlam2InterfaceRgbd:: depth size %d, %d, step -> %d", depth_ptr->image.cols, depth_ptr->image.rows, depth_ptr->image.step[0]);

    cv::Mat T_C_W_opencv =
        slam_system_->TrackRGBD(rgbImage, depth_ptr->image,
                                depth_ptr->header.stamp.toSec());

//    ROS_ERROR("OrbSlam2InterfaceRgbd::depthCallback TrackRGBD 2");

    std_msgs::Int32 msg;
    msg.data = slam_system_->GetTrackingState();

    ros::Duration proc_time=ros::Time::now()-BeforeProc;

    state_pub.publish(msg);

    // If tracking successfull
    if (!T_C_W_opencv.empty()) {
        // Converting to kindr transform and publishing
        Transformation T_C_W, T_W_C;
        convertOrbSlamPoseToKindr(T_C_W_opencv, &T_C_W);
        T_W_C = T_C_W.inverse();
//        publishCurrentPose(T_W_C, rgb_header);
        publishCurrentPose(T_W_C, depth_ptr->header);
        // Saving the transform to the member for publishing as a TF
        T_W_C_ = T_W_C;


    }

    if (use_master_logger) {
        logger->log_slam_data(depth_ptr->header.stamp, proc_time, msg.data, T_C_W_opencv);
    }
//    ROS_ERROR("OrbSlam2InterfaceRgbd::depthCallback end");
}




}  // namespace orb_slam_2_interface