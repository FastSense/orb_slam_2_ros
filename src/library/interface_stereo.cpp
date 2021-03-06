#include <cv_bridge/cv_bridge.h>

#include "orb_slam_2_ros/interface_stereo.hpp"

namespace orb_slam_2_interface {

OrbSlam2InterfaceStereo::OrbSlam2InterfaceStereo(const ros::NodeHandle& nh,
                                                 const ros::NodeHandle& nh_private, bool use_pangolin)
    : OrbSlam2Interface(nh, nh_private) {
    // Getting data and params
    subscribeToTopics();

    nh_private.param("imgs_folder", imgs_folder, std::string("/home/sergey/Pictures/"));
    fs_logfile=std::ofstream(imgs_folder+"ToSlam.txt");

    slam_system_ = std::shared_ptr<ORB_SLAM2::System>(
        new ORB_SLAM2::System(vocabulary_file_path_, settings_file_path_,
                              ORB_SLAM2::System::STEREO, use_pangolin));
}

void OrbSlam2InterfaceStereo::subscribeToTopics() {
    // Subscribing to the stereo images
    left_sub_ = std::shared_ptr<message_filters::Subscriber<sensor_msgs::Image>>(
        new message_filters::Subscriber<sensor_msgs::Image>(
            nh_, "camera/left/image_raw", 1));
    right_sub_ = std::shared_ptr<message_filters::Subscriber<sensor_msgs::Image>>(
        new message_filters::Subscriber<sensor_msgs::Image>(
            nh_, "camera/right/image_raw", 1));
    // Creating a synchronizer
    sync_ = std::shared_ptr<message_filters::Synchronizer<sync_pol>>(
        new message_filters::Synchronizer<sync_pol>(sync_pol(10), *left_sub_,
                                                    *right_sub_));
    // Registering the synchronized image callback
    sync_->registerCallback(
        boost::bind(&OrbSlam2InterfaceStereo::stereoImageCallback, this, _1, _2));
}

void OrbSlam2InterfaceStereo::stereoImageCallback(
    const sensor_msgs::ImageConstPtr& msg_left,
    const sensor_msgs::ImageConstPtr& msg_right) {

    static double FirstTS;

    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptr_left;
    try {
        cv_ptr_left = cv_bridge::toCvShare(msg_left);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    cv_bridge::CvImageConstPtr cv_ptr_right;
    try {
        cv_ptr_right = cv_bridge::toCvShare(msg_right);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    if (FirstTS==0)
    {
        FirstTS=cv_ptr_left->header.stamp.toSec();
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
    cv::Mat T_C_W_opencv =slam_system_->TrackStereo(cv_ptr_left->image, cv_ptr_right->image,
                                                    cv_ptr_left->header.stamp.toSec()-FirstTS,
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
        publishCurrentPose(T_W_C, msg_left->header);
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

}  // namespace orb_slam_2_interface