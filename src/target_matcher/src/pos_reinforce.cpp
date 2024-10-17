#include <target_matcher/matcher_node.hpp>
#include <target_matcher/utils.hpp>
#include <radar_interface/msg/armor.hpp>


constexpr float square(float a, float b) { return a * a + b * b; }



void MatcherNode::pos_reinforce_timer_callback()
{
    for (const auto& tg : last_targets.targets) {
        int reinforce_idx = pos_reinforce(tg.position[0], tg.position[1]);
        if (reinforce_idx == -1)
            continue;
        auto mapped = targets_value_map.find(tg.id);
        if (mapped != targets_value_map.end()) {
            RCLCPP_DEBUG(get_logger(), "Pos Reinforce tgid: %ld, idx: %d", mapped->first, reinforce_idx);
            auto& value = mapped->second;
            long pos_reinforce_max = get_parameter("pos_reinforce_max").as_int();
            if (value[reinforce_idx] < pos_reinforce_max)
                value[reinforce_idx] = std::min(value[reinforce_idx] + get_parameter("pos_reinforce_inc").as_int(), pos_reinforce_max);
        }
    }
}

int MatcherNode::pos_reinforce(float x, float y)
{
    int type = inner_pos_reinforce(x, y);
    if (type != -1)
        return to_idx(radar_interface::msg::Armor::COLOR_RED, type);
    type = inner_pos_reinforce(28.f - x, 15.f - y);
    if (type != -1)
        return to_idx(radar_interface::msg::Armor::COLOR_BLUE, type);
    return -1;
}

int MatcherNode::inner_pos_reinforce(float x, float y)
{
    // 本函数只做红方机器人的定位检测
    // 哨兵检测
    if (x > 4 && x < 7.2 && y > 4.7 && y < 10.3)
        return radar_interface::msg::Armor::TYPE_SENTRY;

    // 工程检测
    constexpr float ENG_SQUARE_THRES = 0.6 * 0.6;
    // 大资源岛
    constexpr float ENG_BIG_X = 13.4, ENG_BIG_Y = 6.9;
    if (square(x - ENG_BIG_X, y - ENG_BIG_Y) < ENG_SQUARE_THRES)
        return radar_interface::msg::Armor::TYPE_ENGINEER;
    // 小资源岛
    constexpr float ENG_SMALL_X = 9.1, ENG_SMALL_Y = 10.9;
    if (square(x - ENG_SMALL_X, y - ENG_SMALL_Y) < ENG_SQUARE_THRES)
        return radar_interface::msg::Armor::TYPE_ENGINEER;
    // 兑矿区
    constexpr float ENG_REDEEM_X = 1.5, ENG_REDEEM_Y = 3.2;
    if (square(x - ENG_REDEEM_X, y - ENG_REDEEM_Y) < ENG_SQUARE_THRES)
        return radar_interface::msg::Armor::TYPE_ENGINEER;
    // 英雄前哨站
    constexpr float HERO_SQUARE_THRES = 1 * 1;
    constexpr float HERO_X = 10.4, HERO_Y = 14.;
    if (square(x - HERO_X, y - HERO_Y) < HERO_SQUARE_THRES)
        return radar_interface::msg::Armor::TYPE_HERO;

    return -1;
}
