#ifndef ORB_SLAM_2_ROS_INTERFACE_RGBD_H
#define ORB_SLAM_2_ROS_INTERFACE_RGBD_H

#include <ros/ros.h>
#include <sensor_msgs/Image.h>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>

#include "orb_slam_2_ros/interface.hpp"

#include <image_geometry/pinhole_camera_model.h>
#include <tf2_ros/transform_listener.h>
#include <eigen_conversions/eigen_msg.h>
#include <image_transport/image_transport.h>


namespace orb_slam_2_interface {

// The synchronization policy used by the interface to sync stereo images
typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,
    sensor_msgs::Image>
    sync_pol;

// Class handling global alignment calculation and publishing
class OrbSlam2InterfaceRgbd : public OrbSlam2Interface {
public:
    // Constructor
    OrbSlam2InterfaceRgbd(const ros::NodeHandle& nh,
                            const ros::NodeHandle& nh_private);

    void depth_model_callback(const sensor_msgs::CameraInfoConstPtr& depth_info_msg);
    void rgb_model_callback(const sensor_msgs::CameraInfoConstPtr& rgb_info_msg);

    void rgbCallback(const sensor_msgs::ImageConstPtr& msg);
    void depthCallback(const sensor_msgs::ImageConstPtr& depth_msg);

    template<typename T>
    void convert(const sensor_msgs::ImageConstPtr& depth_msg,
                 const sensor_msgs::ImagePtr& registered_msg,
                 const Eigen::Affine3d& depth_to_rgb);

protected:
    // Subscribes to the appropriate ROS topics
    void subscribeToTopics();

    // Callbacks
    void ImageCallback(const sensor_msgs::ImageConstPtr& msg_left,
                             const sensor_msgs::ImageConstPtr& msg_disparity);

    // Subscribers
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::Image>> left_sub_;
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::Image>> disparity_sub_;
    std::shared_ptr<message_filters::Synchronizer<sync_pol>> sync_;

    image_geometry::PinholeCameraModel depth_model_, rgb_model_;

    ros::Subscriber depth_model_sub;
    ros::Subscriber rgb_model_sub;

    ros::Subscriber rgb_sub_;
    ros::Subscriber depth_sub_;

    Eigen::Affine3d depth_to_rgb;
    boost::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    boost::shared_ptr<tf2_ros::TransformListener> tf_;
    image_transport::CameraPublisher pub_registered_;
    sensor_msgs::CameraInfoPtr rgb_info_msg_saved_ptr;
    sensor_msgs::CameraInfoPtr depth_info_msg_saved_ptr;

    cv::Mat rgbImage;
    std_msgs::Header rgb_header;
//    boost::shared_ptr<sensor_msgs::CameraInfo> rgb_info_msg_saved_ptr;
};

// Encapsulate differences between processing float and uint16_t depths
template<typename T> struct DepthTraits {};

template<>
struct DepthTraits<uint16_t>
{
    static inline bool valid(uint16_t depth) { return depth != 0; }
    static inline float toMeters(uint16_t depth) { return depth * 0.001f; } // originally mm
    static inline uint16_t fromMeters(float depth) { return (depth * 1000.0f) + 0.5f; }
    static inline void initializeBuffer(std::vector<uint8_t>& buffer) {} // Do nothing - already zero-filled
};

template<>
struct DepthTraits<float>
{
    static inline bool valid(float depth) { return std::isfinite(depth); }
    static inline float toMeters(float depth) { return depth; }
    static inline float fromMeters(float depth) { return depth; }

    static inline void initializeBuffer(std::vector<uint8_t>& buffer)
    {
        float* start = reinterpret_cast<float*>(&buffer[0]);
        float* end = reinterpret_cast<float*>(&buffer[0] + buffer.size());
        std::fill(start, end, std::numeric_limits<float>::quiet_NaN());
    }
};

}  // namespace orb_slam_2_interface

#endif //ORB_SLAM_2_ROS_INTERFACE_RGBD_H
