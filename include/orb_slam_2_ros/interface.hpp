#ifndef ORB_SLAM_2_INTERFACE
#define ORB_SLAM_2_INTERFACE

#include <memory>
#include <string>

#include <geometry_msgs/TransformStamped.h>
#include <System.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <tf/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <Eigen/Geometry>

#include "orb_slam_2_ros/types.hpp"
#include "std_msgs/Int32.h"

#include "orb_slam_2_ros/logger.h"

namespace orb_slam_2_interface {

// Default values for parameters
static const bool kDefaultVerbose = true;
static const std::string kDefaultFrameId = "world";
static const std::string kDefaultChildFrameId = "cam0";

// Class handling global alignment calculation and publishing
class OrbSlam2Interface {
 public:
  // Constructor
  OrbSlam2Interface(const ros::NodeHandle& nh,
                    const ros::NodeHandle& nh_private);

 protected:
  // Subscribes and Advertises to the appropriate ROS topics
  void advertiseTopics();
  void getParametersFromRos();

  // Callbacks
  void imageCallback(const sensor_msgs::ImageConstPtr& msg);

  // Publishing functions
  void publishCurrentPose(const Transformation& T,
                          const std_msgs::Header& header);
  void publishCurrentPoseAsTF(const ros::TimerEvent& event);

  // Helper functions
  void convertOrbSlamPoseToKindr(const cv::Mat& T_cv, Transformation* T_kindr);

    void convertOrbSlamPoseToRosGeometry(const cv::Mat& T_cv,
                                                            geometry_msgs::Transform* msg);
    void convertRosPoseToOrbSlam(const geometry_msgs::Transform& msg,
                                                    cv::Mat& T_cv);

    void px4_pose_callback (const geometry_msgs::PoseStamped px4_pose);
    void save_cam_base_tf();
    void transform_px4_pose();

  // Node handles
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  // Publishers
  ros::Publisher T_pub_;
  tf::TransformBroadcaster tf_broadcaster_;
  ros::Timer tf_timer_;
    ros::Publisher state_pub;
    ros::Publisher log_path_pub;

  // The orb slam system
  std::shared_ptr<ORB_SLAM2::System> slam_system_;

  // The current pose
  Transformation T_W_C_;

  // Parameters
  bool verbose_;
    bool use_master_logger;
  std::string vocabulary_file_path_;
  std::string settings_file_path_;

  // Transform frame names
  std::string frame_id_;
  std::string child_frame_id_;

  // Folder for input images logging (followed with "/")
  std::string imgs_folder ;

    LoggerMaster *logger;

    ros::Subscriber px4_sub;
    geometry_msgs::Transform px_cam_in_world;
    geometry_msgs::Pose px_pose_;
    tf2::Transform CB_tf2;
    tf2::Transform BC_tf2;
    bool px4_pose_ready;
    bool use_imu_;
};

}  // namespace orb_slam_2_interface

#endif /* ORB_SLAM_2_INTERFACE */
