#include "orb_slam_2_ros/interface.hpp"

#include <glog/logging.h>
#include <minkindr_conversions/kindr_msg.h>
#include <minkindr_conversions/kindr_tf.h>
#include <opencv2/core/core.hpp>
#include <opencv2/core/eigen.hpp>



namespace orb_slam_2_interface {

OrbSlam2Interface::OrbSlam2Interface(const ros::NodeHandle& nh,
                                     const ros::NodeHandle& nh_private)
    : nh_(nh),
      nh_private_(nh_private),
      verbose_(kDefaultVerbose),
      frame_id_(kDefaultFrameId),
      child_frame_id_(kDefaultChildFrameId) {

    use_master_logger = false;
    nh_private_.getParam("use_master_logger", use_master_logger);
    if (use_master_logger) {
        logger = new LoggerMaster(nh_private_);
    }
    // Getting data and params
    advertiseTopics();
    getParametersFromRos();

    if (use_imu_) {
        px4_sub = nh_.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 1,
                                                            &OrbSlam2Interface::px4_pose_callback, this);

        ROS_WARN("OrbSlam2Interface subscribed to %s topic", px4_sub.getTopic().c_str());
        save_cam_base_tf();
    }
    px4_pose_ready = false;
}

void OrbSlam2Interface::advertiseTopics() {
    // Advertising topics
    T_pub_ = nh_private_.advertise<geometry_msgs::TransformStamped>(
        "transform_cam", 1);
    // Creating a callback timer for TF publisher
    tf_timer_ = nh_.createTimer(ros::Duration(0.01),
                                &OrbSlam2Interface::publishCurrentPoseAsTF, this);
    state_pub = nh_.advertise<std_msgs::Int32>("slam_state", 10);
}

void OrbSlam2Interface::getParametersFromRos() {
    // Getting the paths to the files required by orb slam
    CHECK(nh_private_.getParam("vocabulary_file_path", vocabulary_file_path_))
    << "Please provide the vocabulary_file_path as a ros param.";
    CHECK(nh_private_.getParam("settings_file_path", settings_file_path_))
    << "Please provide the settings_file_path as a ros param.";
    // Optional params
    nh_private_.getParam("verbose", verbose_);
    nh_private_.getParam("frame_id", frame_id_);
    nh_private_.getParam("child_frame_id", child_frame_id_);

    nh_private_.param("use_imu", use_imu_, false);
}

void OrbSlam2Interface::publishCurrentPose(const Transformation& T,
                                           const std_msgs::Header& header) {
    // Creating the message
    geometry_msgs::TransformStamped msg;
    // Filling out the header
    msg.header = header;
    // Setting the child and parent frames
    msg.child_frame_id = child_frame_id_;
    // Converting from a minkindr transform to a transform message
    tf::transformKindrToMsg(T, &msg.transform);
    // Publishing the current transformation.
    T_pub_.publish(msg);
}

void OrbSlam2Interface::publishCurrentPoseAsTF(const ros::TimerEvent& event) {
    tf::Transform tf_transform;
    tf::transformKindrToTF(T_W_C_, &tf_transform);
    tf_broadcaster_.sendTransform(tf::StampedTransform(
        tf_transform, ros::Time::now(), frame_id_, child_frame_id_));
}

void OrbSlam2Interface::convertOrbSlamPoseToKindr(const cv::Mat& T_cv,
                                                  Transformation* T_kindr) {
    // Argument checks
    CHECK_NOTNULL(T_kindr);
    CHECK_EQ(T_cv.cols, 4);
    CHECK_EQ(T_cv.rows, 4);
    // Open CV mat to Eigen matrix (float)
    Eigen::Matrix4f T_eigen_f;
    cv::cv2eigen(T_cv, T_eigen_f);
    // Eigen matrix (float) to Eigen matrix (double)
    Eigen::Matrix4d T_eigen_d = T_eigen_f.cast<double>();
    // Extracting and orthonormalizing the rotation matrix
    Eigen::Matrix3d R_unnormalized = T_eigen_d.block<3, 3>(0, 0);
    Eigen::AngleAxisd aa(R_unnormalized);
    Eigen::Matrix3d R = aa.toRotationMatrix();
    // Constructing the transformation
    Quaternion q_kindr(R);
    Eigen::Vector3d t_kindr(T_eigen_d.block<3, 1>(0, 3));
    *T_kindr = Transformation(q_kindr, t_kindr);
}


void OrbSlam2Interface::convertOrbSlamPoseToRosGeometry(const cv::Mat& T_cv,
                                                        geometry_msgs::Transform* msg){
    Transformation T_C_W, T_W_C;
    convertOrbSlamPoseToKindr(T_cv, &T_C_W);
    T_W_C = T_C_W.inverse();
    tf::transformKindrToMsg(T_W_C, msg);

}



void eigen2cv_u(const Eigen::Matrix<double, 4, 4>& src, cv::Mat& dst)
{
    if (!(src.Flags & Eigen::RowMajorBit))
    {
        cv::Mat _src(src.cols(), src.rows(), cv::DataType<double>::type,
                     (void*)src.data(), src.stride() * sizeof(double));
        cv::transpose(_src, dst);
    }
    else
    {
        cv::Mat _src(src.rows(), src.cols(), cv::DataType<double>::type,
                     (void*)src.data(), src.stride() * sizeof(double));
        _src.copyTo(dst);
    }
}

void OrbSlam2Interface::convertRosPoseToOrbSlam(const geometry_msgs::Transform& msg,
                                                cv::Mat& T_cv) {
    Transformation Tr, TrI;
    tf::transformMsgToKindr(msg, &Tr);
    TrI = Tr.inverse();

    Eigen::Matrix<double , 4, 4> TranMat = TrI.getTransformationMatrix();
    eigen2cv_u(TranMat, T_cv);
    T_cv.convertTo(T_cv, CV_32F);
}


void transformMsgToTF2(const geometry_msgs::Transform& msg, tf2::Transform& tf2) {
    tf2 = tf2::Transform(tf2::Quaternion(msg.rotation.x, msg.rotation.y, msg.rotation.z, msg.rotation.w),
                         tf2::Vector3(msg.translation.x, msg.translation.y, msg.translation.z));
}

void transformTF2ToMsg(const tf2::Transform& tf2, geometry_msgs::Transform& msg)
{
    msg.translation.x = tf2.getOrigin().x();
    msg.translation.y = tf2.getOrigin().y();
    msg.translation.z = tf2.getOrigin().z();
    msg.rotation.x = tf2.getRotation().x();
    msg.rotation.y = tf2.getRotation().y();
    msg.rotation.z = tf2.getRotation().z();
    msg.rotation.w = tf2.getRotation().w();
}

void OrbSlam2Interface::save_cam_base_tf() {
    static tf2_ros::Buffer tfBuffer;
    static tf2_ros::TransformListener tfListener(tfBuffer);

    geometry_msgs::TransformStamped CB_ts;
    geometry_msgs::TransformStamped BC_ts;

    try{
        ros::Time now = ros::Time(0);
        if (tfBuffer.canTransform("base_link", "vi_sensor/camera_left_link", now, ros::Duration(3.0))) {
            CB_ts = tfBuffer.lookupTransform("base_link", "vi_sensor/camera_left_link", now);
        } else {
            ROS_ERROR("OrbSlam2Interface::save_cam_base_tf: timeout waiting for transform 1");
            return;
        }
    }
    catch (tf2::TransformException &ex) {
        ROS_WARN("%s",ex.what());
        return;
    }

    try{
        ros::Time now = ros::Time(0);
        if (tfBuffer.canTransform("vi_sensor/camera_left_link", "base_link", now, ros::Duration(3.0))) {
            BC_ts = tfBuffer.lookupTransform("vi_sensor/camera_left_link", "base_link", now);
        } else {
            ROS_ERROR("OrbSlam2Interface::save_cam_base_tf: timeout waiting for transform 2");
            return;
        }
    }
    catch (tf2::TransformException &ex) {
        ROS_WARN("%s",ex.what());
        return;
    }

    transformMsgToTF2(CB_ts.transform, CB_tf2);
    transformMsgToTF2(BC_ts.transform, BC_tf2);
}

void OrbSlam2Interface::transform_px4_pose() {

    geometry_msgs::TransformStamped P_ts;
    tf2::Transform P_tf2;
    P_ts.transform.translation.x = px_pose_.position.x;
    P_ts.transform.translation.y = px_pose_.position.y;
    P_ts.transform.translation.z = px_pose_.position.z;

    P_ts.transform.rotation.x = px_pose_.orientation.x;
    P_ts.transform.rotation.y = px_pose_.orientation.y;
    P_ts.transform.rotation.z = px_pose_.orientation.z;
    P_ts.transform.rotation.w = px_pose_.orientation.w;

    transformMsgToTF2(P_ts.transform, P_tf2);

    tf2::Transform CW_tf2;
    CW_tf2.mult(P_tf2, BC_tf2.inverse());


    tf2::Transform S_TF;
    S_TF.mult(CB_tf2.inverse(), CW_tf2);

    geometry_msgs::Transform S_t;
    transformTF2ToMsg(S_TF, S_t);

    px_cam_in_world.translation.z = S_t.translation.x;
    px_cam_in_world.translation.x = -S_t.translation.y;
    px_cam_in_world.translation.y = -S_t.translation.z;

    px_cam_in_world.rotation.z = S_t.rotation.x;
    px_cam_in_world.rotation.x = -S_t.rotation.y;
    px_cam_in_world.rotation.y = -S_t.rotation.z;
    px_cam_in_world.rotation.w = S_t.rotation.w;
}

// px4_pose = px4 estimation of base_link in world frame
void OrbSlam2Interface::px4_pose_callback (const geometry_msgs::PoseStamped px4_pose){

    px_pose_ = px4_pose.pose;

    px4_pose_ready = true;

//    ROS_WARN("OrbSlam2Interface::px4_pose_callback");
//    ROS_WARN("xyz -> (%f, %f, %f) quat -> (%f, %f, %f, %f)", px_cam_in_world.translation.x,
//             px_cam_in_world.translation.y,
//             px_cam_in_world.translation.z,
//             px_cam_in_world.rotation.x,
//             px_cam_in_world.rotation.y,
//             px_cam_in_world.rotation.z,
//             px_cam_in_world.rotation.w);


}

}  // namespace orb_slam_2_interface

