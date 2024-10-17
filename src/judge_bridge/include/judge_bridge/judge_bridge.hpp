#pragma once

#include <atomic>
#include <cstdint>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/subscription_base.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <std_msgs/msg/u_int16.hpp>
#include <std_msgs/msg/bool.hpp>
#include <stdexcept>

#include "judge_bridge/protocol.hpp"
#include "judge_bridge/serial.hpp"
#include "judge_bridge/crc.hpp"

#include <radar_interface/msg/radar_info.hpp>
#include <radar_interface/msg/radar_cmd.hpp>
#include <radar_interface/msg/radar_mark_data.hpp>
#include <radar_interface/msg/map_robot_data.hpp>
#include <radar_interface/msg/map_command.hpp>
#include <radar_interface/msg/match_result.hpp>
#include <radar_interface/msg/matched_target.hpp>
#include <radar_interface/msg/game_robot_hp.hpp>
#include <radar_interface/msg/uwb_data.hpp>
#include <radar_interface/team_color.hpp>


using namespace JudgeBridge;
using namespace radar_interface;

class JudgeBridgeNode : public rclcpp::Node
{
private:
    std::unique_ptr<JudgeSerial> judge_serial;
    rclcpp::Publisher<radar_interface::msg::RadarMarkData>::SharedPtr pub_radar_mark_data;
    rclcpp::Publisher<radar_interface::msg::RadarInfo>::SharedPtr pub_radar_info;
    rclcpp::Publisher<radar_interface::team_color::msg>::SharedPtr pub_color;
    rclcpp::Publisher<std_msgs::msg::UInt16>::SharedPtr pub_remain_time;
    rclcpp::Publisher<radar_interface::msg::GameRobotHP>::SharedPtr pub_game_robot_hp;
    rclcpp::Publisher<radar_interface::msg::MapCommand>::SharedPtr pub_map_keyboard;
    rclcpp::Publisher<radar_interface::msg::UwbData>::SharedPtr pub_uwb_data;

    std::thread read_thread;

    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr sub_radar_cmd;
    rclcpp::Subscription<radar_interface::msg::MatchResult>::SharedPtr sub_match_result;
    rclcpp::Subscription<radar_interface::msg::MatchResult>::SharedPtr sub_map_robot_data;
    std::atomic<team_color::ENUM> color { team_color::UNKNOWN };

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_custom_info;

    void init_serial();
    void filter_handler(JudgeSerial::JudgePair message);
    void send_map_robot_data(const radar_interface::msg::MatchResult& topic_message);

    void send_custom_info(const std::string& str);
    void map_command_callback(const map_command_t& cmd);
    void robot_status_callback(const robot_status_t& status);
    void game_status_callback(const game_status_t& status);
    void game_robot_hp_callback(const game_robot_HP_t& hp);
    void interaction_data_callback(const std::vector<uint8_t>& data);

    void send_sentry_data(const radar_interface::msg::MatchResult& topic_message);
    void send_radar_cmd(const std_msgs::msg::UInt8 &radar_cmd);

public:
    JudgeBridgeNode();
};