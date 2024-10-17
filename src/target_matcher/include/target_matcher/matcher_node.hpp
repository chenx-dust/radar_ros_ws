#pragma once

#include <map>
#include <array>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <radar_interface/msg/target_array.hpp>
#include <radar_interface/msg/detected_target_array.hpp>
#include <radar_interface/msg/match_result.hpp>
#include <radar_interface/msg/feedback_target_array.hpp>

class MatcherNode : public rclcpp::Node {
public:
    using ValueArray = std::array<long, 12>;
    using TargetValueMap = std::map<long, ValueArray>;

private:
    rclcpp::Subscription<radar_interface::msg::TargetArray>::SharedPtr target_sub;
    std::vector<rclcpp::Subscription<radar_interface::msg::DetectedTargetArray>::SharedPtr> detected_target_subs;
    rclcpp::Subscription<radar_interface::msg::FeedbackTargetArray>::SharedPtr feedback_sub;

    rclcpp::Publisher<radar_interface::msg::MatchResult>::SharedPtr match_result_pub;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr vis_pub;
    rclcpp::TimerBase::SharedPtr vis_timer;
    rclcpp::TimerBase::SharedPtr pos_reinforce_timer;

    TargetValueMap targets_value_map;
    radar_interface::msg::MatchResult result;
    radar_interface::msg::TargetArray last_targets;

    void target_callback(const radar_interface::msg::TargetArray::SharedPtr msg);
    void detected_target_callback(const radar_interface::msg::DetectedTargetArray::SharedPtr msg);
    void feedback_callback(const radar_interface::msg::FeedbackTargetArray::SharedPtr msg);
    void match_and_pub(const radar_interface::msg::TargetArray::SharedPtr msg);
    void vis_timer_callback();
    void pos_reinforce_timer_callback();

    int pos_reinforce(float x, float y);
    int inner_pos_reinforce(float x, float y);

public:
    MatcherNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
};
