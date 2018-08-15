#include <cv_bridge/cv_bridge.h>

#include "orb_slam_2_ros/interface_rgbd.h"

namespace orb_slam_2_interface {


OrbSlam2InterfaceRgbd::OrbSlam2InterfaceRgbd(const ros::NodeHandle& nh,
                                             const ros::NodeHandle& nh_private, bool use_pangolin)
    : OrbSlam2Interface(nh, nh_private) {

    ROS_WARN("Urock OrbSlam2InterfaceRgbd");

    // Getting data and params
    subscribeToTopics();
    //advertiseTopics();
//    getParametersFromRos();
    slam_system_ = std::shared_ptr<ORB_SLAM2::System>(
        new ORB_SLAM2::System(vocabulary_file_path_, settings_file_path_,
                              ORB_SLAM2::System::RGBD, use_pangolin));

//    rgbImage = cv::Mat(480,640,CV_8UC1);
}

void OrbSlam2InterfaceRgbd::subscribeToTopics() {


//    rgb_sub_ = nh_.subscribe("camera/left/image_raw", 1,
//                               &OrbSlam2InterfaceRgbd::rgbCallback, this);
//    depth_sub_ = nh_.subscribe("camera/depth/disparity", 1,
//                             &OrbSlam2InterfaceRgbd::depthCallback, this);

    // Subscribing to the rgbd
    left_sub_ = std::shared_ptr<message_filters::Subscriber<sensor_msgs::Image>>(
        new message_filters::Subscriber<sensor_msgs::Image>(
            nh_, "camera/left/image_raw", 1));
    disparity_sub_ = std::shared_ptr<message_filters::Subscriber<sensor_msgs::Image>>(
        new message_filters::Subscriber<sensor_msgs::Image>(
            nh_, "camera/depth/disparity", 1));
    // Creating a synchronizer
    sync_ = std::shared_ptr<message_filters::Synchronizer<sync_pol>>(
        new message_filters::Synchronizer<sync_pol>(sync_pol(10), *left_sub_,
                                                    *disparity_sub_));
    // Registering the synchronized image callback
    sync_->registerCallback(
        boost::bind(&OrbSlam2InterfaceRgbd::rgbdImageCallback, this, _1, _2));

}

void OrbSlam2InterfaceRgbd::rgbdImageCallback(const sensor_msgs::ImageConstPtr& msg_left,
                                              const sensor_msgs::ImageConstPtr& msg_depth) {

    cv_bridge::CvImageConstPtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvShare(msg_left);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("rgbdImageCallback: cv_bridge exception: %s", e.what());
        return;
    }



    cv_bridge::CvImageConstPtr depth_ptr;
    try {
        depth_ptr = cv_bridge::toCvShare(msg_depth);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("rgbdImageCallback: cv_bridge exception: %s", e.what());
        return;
    }

    static double FirstTS;

    if (FirstTS==0) {
        FirstTS=msg_depth->header.stamp.toSec();
    }

    // There might be an issue when using use_imu_, need to check px4 -> cam frame transform
    // Also sometimes this node fails to read last mavros px4 position and it fucks up everything


    cv::Mat ConstantVelPoseEstimation;
    cv::Mat Px4PoseEstimation;

    if (px4_pose_ready) {
        transform_px4_pose();
        convertRosPoseToOrbSlam(px_cam_in_world, Px4PoseEstimation);
    }

    ros::Time BeforeProc=ros::Time::now();
    cv::Mat T_C_W_opencv =
        slam_system_->TrackRGBD(cv_ptr->image, depth_ptr->image,
                                depth_ptr->header.stamp.toSec() - FirstTS,
                                ConstantVelPoseEstimation,
                                px4_pose_ready && use_imu_,
                                Px4PoseEstimation);

    ros::Duration proc_time=ros::Time::now()-BeforeProc;
    px4_pose_ready = false;

    std_msgs::Int32 msg;
    msg.data = slam_system_->GetTrackingState();

    state_pub.publish(msg);

    // If tracking successfull
    bool track_ok = !T_C_W_opencv.empty();
    if (track_ok) {
        // Converting to kindr transform and publishing
        Transformation T_C_W, T_W_C;
        convertOrbSlamPoseToKindr(T_C_W_opencv, &T_C_W);
        T_W_C = T_C_W.inverse();
        publishCurrentPose(T_W_C, depth_ptr->header);
        // Saving the transform to the member for publishing as a TF
        T_W_C_ = T_W_C;
    }

    if (use_master_logger) {
        geometry_msgs::Transform slam_pose;
        geometry_msgs::Transform const_vel_init_pose;
        if (track_ok) {
            convertOrbSlamPoseToRosGeometry(T_C_W_opencv, &slam_pose);
            convertOrbSlamPoseToRosGeometry(ConstantVelPoseEstimation, &const_vel_init_pose);
        }
        logger->log_slam_data(msg_left->header.stamp, proc_time, msg.data, track_ok, slam_pose, const_vel_init_pose, px_cam_in_world);
    }
}

//void OrbSlam2InterfaceRgbd::rgbCallback(const sensor_msgs::ImageConstPtr& msg) {
////    ROS_ERROR("OrbSlam2InterfaceRgbd::rgbCallback start");
//    cv_bridge::CvImageConstPtr cv_ptr;
//    try {
//        cv_ptr = cv_bridge::toCvShare(msg);
//    } catch (cv_bridge::Exception& e) {
//        ROS_ERROR("cv_bridge exception: %s", e.what());
//        return;
//    }
//
//    cv_ptr->image.copyTo(rgbImage);
//    rgb_header = msg->header;
////    ROS_ERROR("OrbSlam2InterfaceRgbd::rgbCallback end");
//}
//
//void OrbSlam2InterfaceRgbd::depthCallback(const sensor_msgs::ImageConstPtr& depth_msg) {
////    ROS_ERROR("OrbSlam2InterfaceRgbd::depthCallback start");
////    ROS_ERROR("OrbSlam2InterfaceRgbd:: depth size %d, %d, type -> %s", depth_msg.get()->height,depth_msg.get()->width, depth_msg.get()->encoding.c_str());
//    cv_bridge::CvImageConstPtr depth_ptr;
//    try {
//        depth_ptr = cv_bridge::toCvShare(depth_msg);
//    } catch (cv_bridge::Exception& e) {
//        ROS_ERROR("cv_bridge exception: %s", e.what());
//        return;
//    }
//    ROS_ERROR("rs_depth_imageCallback: rbg ts -> %f, depth ts -> %f",rgb_header.stamp.toSec(), depth_ptr->header.stamp.toSec());
////    ROS_ERROR("rs_depth_imageCallback: depth_float_img size %d, %d, step -> %d", depth_float_img.cols, depth_float_img.rows, depth_float_img.step[0]);
//
//    ros::Time BeforeProc=ros::Time::now();
//
////    ROS_ERROR("OrbSlam2InterfaceRgbd:: rgb size %d, %d", rgbImage.cols, rgbImage.rows);
////    ROS_ERROR("OrbSlam2InterfaceRgbd:: depth size %d, %d, step -> %d", depth_ptr->image.cols, depth_ptr->image.rows, depth_ptr->image.step[0]);
//
//    cv::Mat mInitialPoseEstimation;
//    cv::Mat T_C_W_opencv =
//        slam_system_->TrackRGBD(rgbImage, depth_ptr->image,
//                                depth_ptr->header.stamp.toSec(), mInitialPoseEstimation);
//
////    ROS_ERROR("OrbSlam2InterfaceRgbd::depthCallback TrackRGBD 2");
//
//    std_msgs::Int32 msg;
//    msg.data = slam_system_->GetTrackingState();
//
//    ros::Duration proc_time=ros::Time::now()-BeforeProc;
//
//    state_pub.publish(msg);
//
//    bool track_ok = !T_C_W_opencv.empty();
//
//    if (track_ok) {
//        // Converting to kindr transform and publishing
//        Transformation T_C_W, T_W_C;
//        convertOrbSlamPoseToKindr(T_C_W_opencv, &T_C_W);
//        T_W_C = T_C_W.inverse();
////        publishCurrentPose(T_W_C, rgb_header);
//        publishCurrentPose(T_W_C, depth_ptr->header);
//        // Saving the transform to the member for publishing as a TF
//        T_W_C_ = T_W_C;
//
//
//    }
//
//    if (use_master_logger) {
//        logger->log_slam_data(depth_ptr->header.stamp, proc_time, msg.data, track_ok, T_C_W_opencv, mInitialPoseEstimation);
//    }
////    ROS_ERROR("OrbSlam2InterfaceRgbd::depthCallback end");
//}




}  // namespace orb_slam_2_interface