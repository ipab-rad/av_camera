#pragma once

#include "image_transport/image_transport.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include <builtin_interfaces/msg/time.hpp>

#include <mutex>
#include <unordered_map>
#include <chrono>


namespace camera
{

class TimestampInspector : public rclcpp::Node
{
public:
    TimestampInspector(const rclcpp::NodeOptions &options);

private:

    void subscribe_to_cameras(std::vector<std::string> &cam_names);

    rmw_qos_profile_t get_topic_qos_profie(rclcpp::Node *node, const std::string &optic);

    bool received_before(uint8_t status_binary, uint8_t cam_idx)
    {
        // Create a mask based on the index (idx=0 -> bit 7, idx=1 -> bit 6, ..., idx=7 -> bit 0)
        return (status_binary & (1 << (7 - cam_idx))) != 0; // 7-idx to reverse the order (idx=0 -> bit 7)
    }

    // Data members
    std::unordered_map<std::string, sensor_msgs::msg::Image::ConstSharedPtr> images_;
    std::unordered_map<std::string, sensor_msgs::msg::CameraInfo::ConstSharedPtr> cam_info_map_;

    std::mutex data_mutex_;

    std::string lidar_frame_;
    std::vector<std::string> cam_names_;
    // std::vector<builtin_interfaces::msg::Time> cameras_timestamps_(6);
    std::vector<builtin_interfaces::msg::Time> prev_cam_ros_stamps_;
    // std::vector<builtin_interfaces::msg::Time> curr_cam_stamps_;
    uint8_t received_status_bin_ = 0;
    std::unordered_map<std::string, uint8_t> cam_idx_map_;

    std::vector<std::chrono::time_point<std::chrono::high_resolution_clock>> prev_cam_host_time_;
    std::vector<std::chrono::milliseconds> cams_pub_period_ms_vec_;

    const int ALL_CAM_RECEIVED = 252;
    // Subscribers
    std::vector<image_transport::CameraSubscriber> image_subscribers_;

};
} // namespace camera