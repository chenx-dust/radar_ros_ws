#pragma once

#include <queue>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <std_msgs/msg/u_int16.hpp>
#include <std_msgs/msg/bool.hpp>
#include <radar_interface/team_color.hpp>
#include <radar_interface/msg/radar_info.hpp>
#include <radar_interface/msg/match_result.hpp>
#include <radar_interface/msg/game_robot_hp.hpp>
#include <radar_interface/msg/map_command.hpp>
#include <radar_interface/msg/radar_mark_data.hpp>


class DvTriggerNode : public rclcpp::Node
{
private:
    struct dv_context_t {
        uint8_t now_chances;
        bool is_dv_trigered;
        uint8_t used_chances;
        unsigned waiting_for_check; // 不为 0 时表示等待确认易伤，每检查一次减 1，如果到 0 都不确认说明失败
    } dv_context;

    struct radar_cmd_t {
        uint8_t radar_cmd;
    };

    radar_interface::team_color::ENUM color;
    radar_interface::msg::GameRobotHP last_hp;
    std::queue<radar_interface::msg::GameRobotHP> hp_queue;
    bool in_battle = false;

    std::array<bool, 6> red_bot_in_blue, blue_bot_in_red;
    std::set<long> bot_ignore;
    bool is_highlight = false;

    bool dv_available();
    void check_trigger();
    void trigger_dv(const std::string_view& reason = "");
    void check_battle();
    void radar_info_callback(const radar_interface::msg::RadarInfo& info);
    void color_callback(const radar_interface::team_color::msg& color);
    void calc_robots_count(const radar_interface::msg::MatchResult& result);
    void hp_callback(const radar_interface::msg::GameRobotHP& hp);
    void time_callback(const std_msgs::msg::UInt16& time);
    void map_keyboard_callback(const radar_interface::msg::MapCommand& key);
    void radar_mark_callback(const radar_interface::msg::RadarMarkData& mark);

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_custom_info;
    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr pub_radar_cmd;
    rclcpp::Subscription<radar_interface::msg::RadarInfo>::SharedPtr sub_radar_info;
    rclcpp::Subscription<radar_interface::team_color::msg>::SharedPtr sub_team_color;
    rclcpp::Subscription<radar_interface::msg::MatchResult>::SharedPtr sub_match_result;
    rclcpp::Subscription<radar_interface::msg::GameRobotHP>::SharedPtr sub_hp;
    rclcpp::Subscription<std_msgs::msg::UInt16>::SharedPtr sub_time;
    rclcpp::Subscription<radar_interface::msg::MapCommand>::SharedPtr sub_key;
    rclcpp::Subscription<radar_interface::msg::RadarMarkData>::SharedPtr sub_radar_mark;

public:
    DvTriggerNode();
};

