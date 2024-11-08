
#include "av_camera_inspector/timestamp_inspector.hpp"

#include <thread>
#include <numeric>

#include <builtin_interfaces/msg/time.hpp>

namespace camera
{

using builtin_interfaces::msg::Time;

TimestampInspector::TimestampInspector(const rclcpp::NodeOptions &options)
    : Node("cam_timestamp_inspector", options)
{

    cam_names_ = this->declare_parameter<std::vector<std::string>>(
        "camera_names", {"fsp_l", "lspf_r", "lspr_l", "rsp_l", "rspf_l", "rspr_r"});

    received_status_bin_ = 0;

    subscribe_to_cameras(cam_names_);

    RCLCPP_INFO(this->get_logger(), "Camera timestamp inspector initialised!");
}

void TimestampInspector::subscribe_to_cameras(std::vector<std::string> &cam_names)
{

    // Create sync sub for each camera
    int cam_idx = 0 ;
    for (const auto &camera_name : cam_names)
    {
        cam_idx_map_[camera_name] = cam_idx;
        prev_cam_ros_stamps_.push_back(Time());
        prev_cam_host_time_.push_back(std::chrono::high_resolution_clock::now());
        cams_pub_period_ms_vec_.push_back(std::chrono::milliseconds{0});
        std::string cam_topic = "/sensor/camera/" + camera_name + "/image_raw";

        image_subscribers_.emplace_back(image_transport::create_camera_subscription(
            this, cam_topic,
            [this, camera_name, cam_idx](const sensor_msgs::msg::Image::ConstSharedPtr &image_msg,
                                const sensor_msgs::msg::CameraInfo::ConstSharedPtr &cam_info_msg)
            {

                auto curr_host_time = std::chrono::high_resolution_clock::now();

                std::lock_guard<std::mutex> lock(data_mutex_);

                if (received_before(received_status_bin_, cam_idx_map_[camera_name]) && cam_names_.size()!= 1)
                {

                    if (received_status_bin_ != ALL_CAM_RECEIVED)
                    {
                        RCLCPP_WARN(this->get_logger(), "Camera %s received again before all images were received!",camera_name.c_str() );
                    }
                }
                else
                {

                    received_status_bin_ |=
                        (1 << (7 - cam_idx_map_[camera_name])); // 7-i for correct bit positioning

                    Time past_stamp = prev_cam_ros_stamps_[cam_idx_map_[camera_name]];

                    Time curr_stamp = image_msg->header.stamp;

                    if (past_stamp.sec == curr_stamp.sec){

                        if (past_stamp.nanosec == curr_stamp.nanosec)
                        {
                            RCLCPP_WARN(this->get_logger(),
                                        "(%s) Current camera ROS timestamp is the same as previous!",
                                        camera_name.c_str());
                        }
                    }

                    auto &prev_host_time = prev_cam_host_time_[cam_idx_map_[camera_name]];
                    cams_pub_period_ms_vec_[cam_idx_map_[camera_name]] =
                        std::chrono::duration_cast<std::chrono::milliseconds>(curr_host_time -
                                                                              prev_host_time);

                    if (received_status_bin_ == ALL_CAM_RECEIVED)
                    {
                        // This camera was the last one remining for this period
                        RCLCPP_INFO(this->get_logger(),
                                    "All cam images were received, last received: %s\n",
                                    camera_name.c_str());

                        // Print stats
                        std::string stats_str;
                        for (size_t idx = 0; idx < cam_names_.size(); ++idx)
                        {
                            auto cam_pub_period_ms = cams_pub_period_ms_vec_[idx];

                            double period_sec = cam_pub_period_ms.count() / 1000.0;

                            double rate_hz = period_sec > 0 ? 1.0 / period_sec : 0;

                            // Format the output string
                            stats_str += "\t" + cam_names_[idx] + ": " +
                                         std::to_string(cam_pub_period_ms.count()) + " ms " + "(" +
                                         std::to_string(rate_hz) + " Hz)\n";
                        }
                        RCLCPP_INFO(this->get_logger(), "Camera stats: \n%s", stats_str.c_str());

                        received_status_bin_ = 0;
                    }

                    prev_cam_ros_stamps_[cam_idx_map_[camera_name]] = curr_stamp;
                    prev_cam_host_time_[cam_idx_map_[camera_name]] = curr_host_time;
                    
                }
        
            },
            "raw"));

        ++cam_idx;
    }
}


rmw_qos_profile_t TimestampInspector::get_topic_qos_profie(rclcpp::Node *node,
                                                             const std::string &topic)
{
    std::string topic_resolved =
        node->get_node_base_interface()->resolve_topic_or_service_name(topic, false);
    auto topics_info = node->get_publishers_info_by_topic(topic_resolved);
    if (topics_info.size())
    {
        auto profile = topics_info[0].qos_profile().get_rmw_qos_profile();
        profile.history = rmw_qos_profile_sensor_data.history;
        profile.depth = rmw_qos_profile_sensor_data.depth;
        return profile;
    }
    else
    {
        return rmw_qos_profile_sensor_data;
    }
}

} // camera namespace

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(camera::TimestampInspector)