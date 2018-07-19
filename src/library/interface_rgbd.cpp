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
    cv_bridge::CvImageConstPtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvShare(msg);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv_ptr->image.copyTo(rgbImage);
    rgb_header = msg->header;
}

void OrbSlam2InterfaceRgbd::depthCallback(const sensor_msgs::ImageConstPtr& depth_msg) {
    cv_bridge::CvImageConstPtr depth_ptr;
    try {
        depth_ptr = cv_bridge::toCvShare(depth_msg);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv::Mat T_C_W_opencv =
        slam_system_->TrackRGBD(rgbImage, depth_ptr->image,
                                depth_ptr->header.stamp.toSec());

    std_msgs::Int32 msg;
    msg.data = slam_system_->GetTrackingState();

    state_pub.publish(msg);

    // If tracking successfull
    if (!T_C_W_opencv.empty()) {
        // Converting to kindr transform and publishing
        Transformation T_C_W, T_W_C;
        convertOrbSlamPoseToKindr(T_C_W_opencv, &T_C_W);
        T_W_C = T_C_W.inverse();
        publishCurrentPose(T_W_C, rgb_header);
        // Saving the transform to the member for publishing as a TF
        T_W_C_ = T_W_C;
    }
}

//void OrbSlam2InterfaceRgbd::depth_model_callback(const sensor_msgs::CameraInfoConstPtr& depth_info_msg) {
//
//    sensor_msgs::CameraInfoPtr temp_ptr( new sensor_msgs::CameraInfo(*depth_info_msg) );
//    depth_info_msg_saved_ptr = temp_ptr;
//
//    depth_model_.fromCameraInfo(depth_info_msg);
//    ROS_WARN("ORB_SLAM_2_ROS: In depth_model_callback");
//    ROS_WARN("Depth model fx -> %f",depth_model_.fx());
//    ROS_WARN("Depth model fy -> %f",depth_model_.fy());
//    ROS_WARN("Depth model height -> %d",depth_model_.cameraInfo().height);
//    ROS_WARN("Depth model width -> %d",depth_model_.cameraInfo().width);
//    depth_model_sub.shutdown();
//}
//
//void OrbSlam2InterfaceRgbd::rgb_model_callback(const sensor_msgs::CameraInfoConstPtr& rgb_info_msg) {
//
//    sensor_msgs::CameraInfoPtr temp_ptr( new sensor_msgs::CameraInfo(*rgb_info_msg) );
//    rgb_info_msg_saved_ptr = temp_ptr;
//
//    rgb_model_.fromCameraInfo(rgb_info_msg);
//    ROS_WARN("ORB_SLAM_2_ROS: In rgb_model_callback");
//    ROS_WARN("RGB model fx -> %f",rgb_model_.fx());
//    ROS_WARN("RGB model fy -> %f",rgb_model_.fy());
//    ROS_WARN("RGB model height -> %d",rgb_model_.cameraInfo().height);
//    ROS_WARN("RGB model width -> %d",rgb_model_.cameraInfo().width);
//    rgb_model_sub.shutdown();
//}



}  // namespace orb_slam_2_interface