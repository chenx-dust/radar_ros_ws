#pragma once

#include <rclcpp/rclcpp.hpp>
#include <radar_interface/msg/match_result.hpp>
#include <radar_interface/msg/target_array.hpp>
#include <radar_interface/msg/radar_mark_data.hpp>
#include <radar_interface/msg/map_robot_data.hpp>
#include <radar_interface/msg/feedback_target_array.hpp>
#include <radar_interface/team_color.hpp>


namespace target_multiplexer {
using team_color = radar_interface::team_color::ENUM;

// 用于盲猜的编解码
constexpr size_t decode_idx(int64_t x) { return -x - 2; }
constexpr int64_t encode_idx(size_t x) { return -x - 2; }

enum class FULL_HIGHLIGHT_STATUS {
    NONE,
    HIGHLIGHT,
    SKIPPED
};

class MultiplexerNode : public rclcpp::Node {
private:
    rclcpp::Subscription<radar_interface::msg::MatchResult>::SharedPtr match_result_sub;
    rclcpp::Subscription<radar_interface::msg::TargetArray>::SharedPtr detected_sub;
    rclcpp::Subscription<radar_interface::msg::RadarMarkData>::SharedPtr radar_mark_sub;
    rclcpp::Subscription<radar_interface::team_color::msg>::SharedPtr team_color_sub;

    rclcpp::Publisher<radar_interface::msg::MapRobotData>::SharedPtr map_pub;
    rclcpp::Publisher<radar_interface::msg::FeedbackTargetArray>::SharedPtr feedback_pub;

    rclcpp::TimerBase::SharedPtr pub_map_timer;

    radar_interface::msg::MatchResult last_match_result;
    radar_interface::msg::TargetArray last_detected;
    radar_interface::msg::RadarMarkData last_mark;

    // id : lasting_time
    std::map<int64_t, size_t> banned_targets;
    std::array<int64_t, 6> last_pub_id;
    // std::array<std::vector<std::pair<float, f-loat>>, 6> blind_guess;
    // std::array<bool, 6> keep_guess;
    unsigned last_hl_num = 0;
    std::array<FULL_HIGHLIGHT_STATUS, 6> full_high_light;

    team_color color = team_color::UNKNOWN;
    int64_t guessing_id = -1;
    int robot_num = 6;

    void radar_mark_callback(const radar_interface::msg::RadarMarkData& msg);
    void team_color_callback(const radar_interface::team_color::msg& msg);

    void multiplexer();
    void update_banned();
    void update_guessing();

    // void load_blind_guess();

    // 用于 MapRobotData 的 ID
    static uint16_t get_map_id(unsigned ori_id, team_color color_);

public:
    MultiplexerNode();
};

}
