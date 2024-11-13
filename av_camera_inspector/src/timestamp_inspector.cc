
#include "av_camera_inspector/timestamp_inspector.hpp"

#include <thread>
#include <numeric>

#include <builtin_interfaces/msg/time.hpp>

namespace camera
{


TimestampInspector::TimestampInspector(const rclcpp::NodeOptions &options)
    : Node("cam_timestamp_inspector", options)
{

    cam_names_ = this->declare_parameter<std::vector<std::string>>(
        "camera_names", {"fsp_l", "lspf_r", "lspr_l", "rsp_l", "rspf_l", "rspr_r"});

    ALL_CAM_RECEIVED = 252; // TODO: Compute this automatically

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
        prev_cam_ros_stamps_.push_back(ROSTime());
        curr_cam_ros_stamps_.push_back(ROSTime());
        prev_cam_host_time_.push_back(std::chrono::high_resolution_clock::now());
        curr_cam_host_time_.push_back(std::chrono::high_resolution_clock::now());
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

                    curr_cam_ros_stamps_[cam_idx_map_[camera_name]] = image_msg->header.stamp;
                    curr_cam_host_time_[cam_idx_map_[camera_name]] = curr_host_time;

                    if (received_status_bin_ == ALL_CAM_RECEIVED)
                    {
                        // This camera was the last one remining for this period
                        RCLCPP_INFO(this->get_logger(),
                                    "All cam images were received, last one received: %s\n",
                                    camera_name.c_str());

                        // Print stats recieved period | received_period ms | stamp_period | stamp_freq
                        std::string stats_str;
                        for (size_t idx = 0; idx < cam_names_.size(); ++idx)
                        {
                            // Compute received period and freq
                            auto & prev_host_time = prev_cam_host_time_[cam_idx_map_[camera_name]];
                            auto & curr_host_time = curr_cam_host_time_[cam_idx_map_[camera_name]];
                            auto cam_pub_period_ms = std::chrono::duration_cast<std::chrono::milliseconds>(curr_host_time -
                                                                              prev_host_time);

                            double period_sec = cam_pub_period_ms.count() / 1000.0;

                            double rate_hz = period_sec > 0 ? 1.0 / period_sec : 0;

                            // Compute image stamp period and freq
                            auto &prev_ros_stamp = prev_cam_ros_stamps_[cam_idx_map_[camera_name]];
                            auto &curr_ros_stamp = curr_cam_ros_stamps_[cam_idx_map_[camera_name]];

                            double stamp_period_ms = get_ros_stamp_diff(prev_ros_stamp, curr_ros_stamp);

                            double stamp_freq_hz = 1000.0 / (stamp_period_ms);

                            // Create stat sentence
                            stats_str += "\t" + cam_names_[idx] + ": " +
                                         std::to_string(cam_pub_period_ms.count()) + " ms " + "(" +
                                         std::to_string(rate_hz) + " Hz) | " +
                                         std::to_string(stamp_period_ms) + " ms" +
                                         " (" + std::to_string(stamp_freq_hz) + " Hz)\n";
                        }

                        RCLCPP_INFO(this->get_logger(), "\n\t Host receiving period | ROS stamp period\n%s", stats_str.c_str());

                        print_camera_timestamps_stats(cam_names_, curr_cam_ros_stamps_);
                        RCLCPP_INFO(rclcpp::get_logger(""), "--------------------------------------------------");

                        received_status_bin_ = 0;

                        prev_cam_ros_stamps_ = curr_cam_ros_stamps_;
                        prev_cam_host_time_ = curr_cam_host_time_;
                    }
                    
                }
        
            },
            "raw"));

        ++cam_idx;
    }
}

void TimestampInspector::print_camera_timestamps_stats(const std::vector<std::string> &camera_names, const std::vector<ROSTime> &times)
{
    // Check if the input vectors are empty or have mismatched sizes
    if (camera_names.empty() || times.empty() || camera_names.size() != times.size())
    {
        RCLCPP_INFO(this->get_logger(), "No valid camera timestamps to process.");
        return;
    }

    // Create a vector of pairs to associate each camera name with its timestamp
    std::vector<std::pair<std::string, ROSTime>> camera_times;
    for (size_t i = 0; i < camera_names.size(); ++i)
    {
        camera_times.emplace_back(camera_names[i], times[i]);
    }

    // Sort by timestamps (oldest to newest)
    std::sort(camera_times.begin(), camera_times.end(), [](const auto &a, const auto &b)
              { return (a.second.sec < b.second.sec) || (a.second.sec == b.second.sec && a.second.nanosec < b.second.nanosec); });

    // Print sorted timestamps with camera names
    RCLCPP_INFO(this->get_logger(), "Camera Timestamps (in seconds.nanoseconds):");
    for (const auto &camera_time : camera_times)
    {
        RCLCPP_INFO(this->get_logger(), "%s -> %d.%d",
                    camera_time.first.c_str(), camera_time.second.sec, camera_time.second.nanosec);
    }

    // Calculate and print time differences in milliseconds
    for (size_t i = 1; i < camera_times.size(); ++i)
    {
        auto &t1 = camera_times[i - 1].second;
        auto &t2 = camera_times[i].second;

        double diff_ms = get_ros_stamp_diff(t1, t2);

        RCLCPP_INFO(this->get_logger(), "Difference between %s and %s: %f ms",
                    camera_times[i - 1].first.c_str(), camera_times[i].first.c_str(), diff_ms);
    }
}

double TimestampInspector::get_ros_stamp_diff(ROSTime &t1, ROSTime &t2)
{
    // Calculate the difference in seconds and nanoseconds
    int64_t sec_diff = t2.sec - t1.sec;
    int32_t nsec_diff = t2.nanosec - t1.nanosec;

    // Handle cases where nanoseconds in t1 are greater than in t2
    if (nsec_diff < 0)
    {
        sec_diff -= 1;
        nsec_diff += 1000000000; // Add a full second in nanoseconds
    }

    // Convert to milliseconds
    double diff_ms = sec_diff * 1000.0 + static_cast<double>(nsec_diff) / 1000000.0;
    return diff_ms;
}

bool TimestampInspector::received_before(uint8_t status_binary, uint8_t cam_idx)
{
    // Create a mask based on the index (idx=0 -> bit 7, idx=1 -> bit 6, ..., idx=7 -> bit 0)
    return (status_binary & (1 << (7 - cam_idx))) != 0; // 7-idx to reverse the order (idx=0 -> bit 7)
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